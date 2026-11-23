// SX1262 <-> USB serial bridge for the Heltec WiFi LoRa 32 V4 and its clones (ESP32-S3,
// SX1262 with a 1.8 V TCXO, RF front end, native USB). It speaks the serial protocol of
// firmware/cc1101_bridge v2.11 byte for byte, so the unmodified freedo daemon drives it.
//
// This sketch is cc1101_bridge.ino with the radio layer replaced. The radio functions keep
// their cc* and rx* names so that the two sketches diff cleanly; sx1262.h holds the driver.
// The host still loads its CC1101 register set: the bridge keeps it in a register file
// (ccEmulXfer), answers reads from it and maps the strobes the host sends to the SX1262.
// The SX1262 always runs the tado PHY of sx1262.h. A build without -DSX1262_TX=1 receives
// only: no path reaches SetTx.
//
// The RECV-shaped blobs of RECV, of AUTOACK and of the CSLQUEUE inbound record
// carry FREQEST as one trailing byte after the LQI/CRC byte. The SX1262 has no
// frequency error readout for GFSK, so it is always 0. Every request ends
// with a CRC-8. See firmware/cc1101_bridge/README.md for the wire format.
//
// Structure:
//   serialTask (core 0) parses requests and writes every response byte.
//   radioTask  (core 1) owns SPI and all SX1262 state. SPI is never touched
//              anywhere else.
//   qCmd moves requests to the radio task, qOut moves response bytes back.
//   The DIO1 interrupt (RxDone) timestamps the end of a frame. The SX1262 holds
//   the whole frame in its buffer, so there is no FIFO to drain.
//   One CSL state machine (IDLE/RX/WAKE/ACK/FOLLOW) runs every transmit job:
//   the per-neighbour queue, the synchronous CSLTX and the Router Advertisement
//   burst all schedule a job on it, so no path can strobe the radio on top of a
//   transaction that is in flight.
//   While the host keeps a pair window open (CMD_PAIRWIN), the idle drain and
//   CMD_AUTOACK answer an Enhanced Beacon Request with an Enhanced Beacon. The
//   CSL state machine never does.
//   The host can switch on a listen before talk (CMD_LBT). It runs before the first
//   frame of a train and of a pair beacon, and never before an Enhanced ACK.

#include <Arduino.h>
#include <SPI.h>
#include <atomic>
#include <string.h>

#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mbedtls/aes.h"
#include "sx1262.h"

// --------------------------------------------------------------- serial layout

// Serial is the native USB CDC of the ESP32-S3, which ignores the line rate. The host opens
// it at 230400 like a CC1101 bridge.
static const uint32_t SERIAL_BAUD = 230400;
static const size_t SERIAL_RX_BUF = 2048;
static const size_t SERIAL_TX_BUF = 2048;

// PING banner. The level is the serial protocol level. A release build adds the release
// version with -DFIRMWARE_BUILD=<version> (unquoted), a local build reads "sx1262-dev".
// build.sh sets sx1262-rx-<commit> or, with transmit, sx1262-<commit>.
#ifndef FIRMWARE_BUILD
#define FIRMWARE_BUILD sx1262-dev
#endif
#define BANNER_STR_(x) #x
#define BANNER_STR(x) BANNER_STR_(x)
static const char BANNER[] = "CC1101BRIDGE v2.11 (" BANNER_STR(FIRMWARE_BUILD) ")";

static const uint8_t REQ_MAGIC = 0xA5;
static const uint8_t RSP_MAGIC = 0x5A;

enum : uint8_t {
  CMD_PING = 0x01,
  CMD_XFER = 0x02,
  CMD_GDO = 0x03,
  CMD_RESET = 0x04,
  CMD_STROBE = 0x05,
  CMD_RECV = 0x06,
  CMD_STREAM = 0x07,
  CMD_TXTRAIN = 0x08,
  CMD_RESPOND = 0x09,
  CMD_AUTOACK = 0x0A,
  CMD_RABURST = 0x0B,
  CMD_CSLTX = 0x0C,
  CMD_CSLQUEUE = 0x0D,
  CMD_PAIRWIN = 0x0E,
  CMD_CONFIGURED = 0x0F,
  CMD_AIRTIME = 0x10,
  CMD_LBT = 0x11
};

// Status bytes of the response header. The host maps every one of them.
enum : uint8_t {
  ST_OK = 0x00,
  ST_NOT_READY = 0x10,
  ST_NO_FRAME = 0x11,
  ST_ACK_SKIPPED = 0x12,
  ST_ACK_FAILED = 0x13,
  ST_INPUT = 0x14,
  ST_TXFAIL = 0x15,
  ST_NOACK = 0x16,
  ST_BADACK = 0x17,
  ST_NOANCHOR = 0x18,
  ST_QUEUE_FULL = 0x19,
  ST_NEIGHBOR_FULL = 0x1A,
  ST_BUSY = 0x1B,
  ST_UNCONFIGURED = 0x1C,
  ST_AIRTIME = 0x1D,
  ST_BAD_CMD = 0xE0,
  ST_BAD_LEN = 0xE1,
  ST_BAD_PAYLOAD = 0xE2,
  ST_BAD_ARGS = 0xE3,
  ST_BAD_CRC = 0xE4
};

// Streaming markers (CMD_STREAM, CMD_RESPOND). One frame is
//   5A F1 <n:1> <n bytes> <freqest:1> <xor8>
// with xor8 over the n bytes and freqest. End of window: 5A F2 00. Heartbeat:
// 5A F3 00, so the host tells a live-but-quiet link from a dead one and never
// times out on a silent CSL gap. freqest is the FREQEST register, the residual
// carrier offset of this packet, which the host folds back into FSCTRL0.
static const uint8_t STREAM_FRAME = 0xF1;
static const uint8_t STREAM_END = 0xF2;
static const uint8_t STREAM_ALIVE = 0xF3;
static const uint32_t STREAM_HEARTBEAT_MS = 500;
static const uint32_t STREAM_DEFAULT_MS = 10000;

static const size_t MAX_PAYLOAD = 1024;
static const uint32_t SERIAL_INTERBYTE_MS = 1000;
static const uint32_t SERIAL_REQUEST_MS = 3000;  // total budget for one request
// The host sends a request at least every 1.25 s. Without a valid one for this
// long the bridge forgets the coordinator, the pair window and the queue.
static const uint32_t HOST_TIMEOUT_MS = 5000;

// ---------------------------------------------------- emulated CC1101 register

// The host speaks to a CC1101 (cc1101/cc1101.go). The bridge keeps the configuration
// registers in ccRegs and answers the status registers from the SX1262.
static const uint8_t CC_IOCFG2 = 0x00;
static const uint8_t CC_IOCFG0 = 0x02;
static const uint8_t CC_FSCAL1 = 0x25;
static const uint8_t CC_CONFIG_REGS = 0x2F;  // 0x00..0x2E
static const uint8_t CC_PARTNUM = 0x30;
static const uint8_t CC_VERSION = 0x31;
static const uint8_t CC_RSSI = 0x34;
static const uint8_t CC_MARCSTATE = 0x35;
static const uint8_t CC_PATABLE = 0x3E;
static const uint8_t CC_FIFO = 0x3F;
static const uint8_t CC_SRES = 0x30;
static const uint8_t CC_SCAL = 0x33;
static const uint8_t CC_SRX = 0x34;
static const uint8_t CC_SIDLE = 0x36;
static const uint8_t CC_SFRX = 0x3A;
static const uint8_t CC_LAST_STROBE = 0x3D;
static const uint8_t READ_BIT = 0x80;
static const uint8_t BURST_BIT = 0x40;
static const uint8_t ADDR_MASK = 0x3F;

// The value the CC1101 firmware writes to IOCFG0 after a reset. The host loads 0x07 and reads
// IOCFG0 back to tell a configured bridge from one that reset (cc1101.go Configured).
static const uint8_t GDO_CFG_LOW = 0x2F;
// FSCAL1 after SCAL: 0x3F is the CC1101 "no lock", which the host reports as ErrNoSynthLock.
// The bridge reads it after a failed SX1262 calibration.
static const uint8_t FSCAL1_LOCKED = 0x00;
static const uint8_t FSCAL1_NO_LOCK = 0x3F;
// PARTNUM and VERSION of a CC1101, which the bench tools check.
static const uint8_t CC_PARTNUM_VALUE = 0x00;
static const uint8_t CC_VERSION_VALUE = 0x14;
static const uint8_t MARC_IDLE = 0x01;
static const uint8_t MARC_RX = 0x0D;
static const uint8_t MARC_FSTXON = 0x12;
static const uint8_t MARC_TX = 0x13;
static const uint8_t CC_STATE_SHIFT = 4;  // chip status byte: state in bits 6:4
static const uint8_t CC_STATE_RX = 1;
static const uint8_t CC_STATE_TX = 2;
static const uint8_t CC_STATUS_FIFO = 0x0F;
static const uint8_t CRC_OK_BIT = 0x80;

static const uint8_t MAX_TX_FRAME = 140;  // longest sub-frame payload of a train
static const uint8_t MAX_RX_RAW = 1 + MAX_TX_FRAME + 2;
static const uint8_t MAX_TRAIN = 48;  // sub-frames per CMD_RESPOND response
static const uint8_t MAX_MATCH = 64;
static const uint32_t TX_TMO_MS = 100;

// ------------------------------------------------------------------ CSL timing

static const uint32_t CSL_PERIOD_US = 1000000;
static const uint32_t CSL_UNIT_US = 320;  // one CSL phase unit, 10 symbols
static const uint32_t CSL_SAMPLE_GUARD_US = 40000;
// A train without an anchor covers one whole CSL period, as the commercial bridge does: the
// first frame announces 3125 units and the train lasts about 1.003 s there. That is 3 ms over
// the 1 s of EN 300 220 Ton_max, and a shorter train would miss the device that samples in
// the gap.
static const uint32_t CSL_BOOTSTRAP_US = 1000000;
static const uint16_t RA_PERIOD_UNITS = 3125;  // 1 s of CSL phase units
static const uint32_t RA_PERIOD_US = 1000000;
// CMD_TXTRAIN and CMD_RESPOND stop before a frame that would end more than this after the
// first one started, so that a train of long frames stays under the 1 s of Ton_max.
static const uint32_t HOST_TRAIN_MAX_US = 980000;
static const uint8_t CSL_WAKE_CAP = 253;
static const uint16_t CSL_ACK_TMO_MAX_MS = 1000;
static const uint16_t CSL_FOLLOW_TMO_MS = 500;
static const uint8_t CSL_FOLLOW_MAX = 6;
static const uint8_t CSL_WAKE_BUDGET = 4;  // wake frames per state machine step
// Between the end of the last wake frame and the start of the Data frame. The
// commercial coordinator leaves 9 to 24 ms, median 12 ms (docs/PROTOCOL.md).
static const uint32_t CSL_DATA_GAP_US = 12000;
static const uint32_t CSL_GUARD_MARGIN_MS = 400;
static const uint8_t CSL_ANCHOR_MISS_LIMIT = 4;
static const uint8_t RA_WAKE_CAP = 253;

static const uint8_t CSL_ANCHORS = 16;
// The guard grows with the time since the last refresh, for the clock error of
// two crystals at 20 and 10 ppm. At half a period the train covers every sample
// instant, as the bootstrap train does. There is no drift estimate: the
// commercial one divides by 60 periods samples stamped to 32 us, and a stamp
// error of a few ms here, divided the same way, moves a projection of some
// hundred periods off the sample.
static const uint64_t CSL_GUARD_PPM = 30;
static const uint32_t CSL_GUARD_MAX_US = CSL_PERIOD_US / 2;

static const uint8_t CSL_NEIGHBORS = 16;
static const uint8_t CSL_NEIGHBOR_DEPTH = 32;  // one fragmented datagram batch
static const uint8_t CSL_JOB_POOL = 128;
static const uint8_t CSL_JOB_NONE = 0xFF;
static const uint8_t CSL_COMPLETE_DEPTH = 8;
static const uint8_t CSL_INBOUND_DEPTH = 32;

// A batch of CSL_NEIGHBOR_DEPTH jobs for one neighbour has to be admitted while
// other neighbours still hold work, so the shared pool keeps room for several
// full batches at once.
static_assert(CSL_JOB_POOL >= 4 * CSL_NEIGHBOR_DEPTH, "job pool too small for a full batch");
static_assert(CSL_JOB_POOL < CSL_JOB_NONE, "job index does not fit the pool sentinel");

// -------------------------------------------------------------------- airtime

// One frame on air is preamble 4, sync 4, length 1, payload and CRC 2 bytes, and a
// byte takes 160 us at 50 kBaud. The window is 61 buckets of one minute, so it is 60 to
// 61 minutes long and always covers a full real hour. The firmware always counts. It refuses a
// transmission only when the host set a budget (CMD_AIRTIME with a budget): the Tcum_on of polite
// spectrum access, 36 s per hour with no listen and 100 s with it (ETSI EN 300 220-2 clause 4.6).
static const uint32_t AIR_BYTE_US = 160;
static const uint8_t AIR_FRAME_OVERHEAD = 4 + 4 + 1 + 2;
static const uint8_t AIR_BUCKETS = 61;
static const uint64_t AIR_BUCKET_US = 60ULL * 1000000ULL;
static const uint16_t AIR_BUCKET_MAX_MS = 60000;

// ---------------------------------------------------------------- listen before talk

// Polite spectrum access (ETSI EN 300 220-2 clause 4.6): listen at least 160 us, defer by a
// random multiple of the listen time, give up after a few tries. The commercial bridge uses
// a backoff of 0 to 6 x 640 us.
static const uint32_t LBT_LISTEN_US = 160;
static const uint8_t LBT_TRIES = 3;
static const uint32_t LBT_BACKOFF_UNIT_US = 640;
static const uint8_t LBT_BACKOFF_MAX_UNITS = 6;
// The time after the start of RX from which the RSSI counts. The CC1101 value; for the SX1262
// it is not measured.
static const uint32_t LBT_SETTLE_US = 250;
// A listen that starts with the receiver settled takes about 190 us, one that waits for the
// RSSI up to LBT_SETTLE_US more, plus the SPI reads. The anchored train starts its listen
// this long before its first frame.
static const uint32_t LBT_LEAD_US = 500;
static const int8_t LBT_THRESHOLD_DEFAULT_DBM = -80;
// The host reads RSSI in the CC1101 format: raw = 2 * (dBm + 74) (CC1101 datasheet table 31).
static const int RSSI_OFFSET_DBM = 74;

// ----------------------------------------------------------------- pair window

// Frame sizes of tado/beacon.go. The CCM* of the link takes the nonce from
// frame[0:13] and the aad from frame[0:16]; the rest up to the 4-byte MIC is
// ciphertext.
static const uint8_t EBR_LEN = 27;
static const uint8_t EB_LEN = 32;
static const uint8_t CCM_NONCE_LEN = 13;
static const uint8_t CCM_AAD_LEN = 16;
static const uint8_t CCM_MIC_LEN = 4;
static const uint8_t MAC_CMD_BEACON_REQ = 0x07;
static const uint8_t SEC_CTRL = 0x04;
static const uint8_t PAIRWIN_OPEN_LEN = 2 + 16 + 8 + 1 + 4 + 1;
static const uint16_t PAIRWIN_MAX_S = 600;
// The commercial beacon starts 2.4 to 7.4 ms after the request ends
// (the pairing capture), after its 124-tick listen of 3.968 ms.
static const uint32_t EB_GAP_US = 3968;

// ------------------------------------------------------------- task parameters

static const uint32_t RADIO_STACK = 8192;
static const uint32_t SERIAL_STACK = 4096;
static const UBaseType_t RADIO_PRIO = 3;
static const UBaseType_t SERIAL_PRIO = 3;
static const int RADIO_CORE = 1;
static const int SERIAL_CORE = 0;
static const uint32_t WDT_TIMEOUT_MS = 2000;

static const uint8_t CMD_POOL = 2;
static const uint8_t OUT_POOL = 4;
static const size_t OUT_BUF_MAX = 4 + MAX_PAYLOAD;

static const uint32_t RADIO_IDLE_TICK_MS = 8;
static const uint32_t RX_ASSEMBLE_TICK_MS = 5;
// A lost DIO1 edge leaves this tick as the only pump. The next frame overwrites a frame
// that was not taken, and it needs at least 1.3 ms of preamble and sync word.
static const uint32_t RX_HOUSEKEEP_TICK_MS = 8;
static const uint32_t HOST_SPI_HOLDOFF_MS = 500;  // host drives the radio itself

// ------------------------------------------------------------ shared structures

struct CmdMsg {
  uint8_t cmd;
  uint16_t len;
  uint8_t payload[MAX_PAYLOAD];
};

struct OutMsg {
  uint16_t len;
  uint8_t data[OUT_BUF_MAX];
};

struct CslFollow {
  uint8_t raw[CSL_FOLLOW_MAX][MAX_RX_RAW];
  uint8_t len[CSL_FOLLOW_MAX];
  uint8_t count;
  uint8_t dropped;
};

struct CslJob {
  uint16_t id;
  uint16_t ackTmoMs;
  uint16_t phase;
  uint8_t flags;
  uint8_t kind;
  uint8_t shortAddr[2];
  uint8_t dataLen;
  uint8_t data[MAX_TX_FRAME];
  uint8_t next;
};

struct CslNeighbor {
  bool used;
  uint8_t shortAddr[2];
  uint8_t head;
  uint8_t tail;
  uint8_t count;
};

struct CslAnchor {
  bool valid;
  uint8_t shortAddr[2];
  uint8_t misses;
  uint64_t sampleUs;
  uint64_t refreshUs;
};

struct CslCompletion {
  uint16_t id;
  uint8_t flags;
  uint8_t status;
  uint8_t sent;
  uint8_t ackLen;
  uint8_t ack[MAX_RX_RAW];
  CslFollow follow;
};

struct CslInbound {
  uint8_t raw[MAX_RX_RAW];
  uint8_t len;
  uint8_t freqest;
};

enum : uint8_t { JOB_QUEUED = 0, JOB_SYNC_CSL = 1, JOB_RA = 2 };

enum : uint8_t {
  CSL_IDLE = 0,
  CSL_RX = 1,
  CSL_WAKE = 2,
  CSL_ACK = 3,
  CSL_FOLLOW = 4,
  CSL_LISTEN = 5
};

enum : uint8_t {
  OUT_OK = 0,
  OUT_TXFAIL = 1,
  OUT_NOACK = 2,
  OUT_BADACK = 3,
  OUT_INPUT = 4,
  OUT_NOANCHOR = 5,
  OUT_AIRTIME = 6,
  OUT_BUSY = 7
};

enum : uint8_t { CSLQ_ENQUEUE = 0, CSLQ_POLL = 1, CSLQ_FLUSH = 2 };
enum : uint8_t { CSL_FLAG_BOOTSTRAP = 1, CSL_FLAG_FOLLOW = 2 };
enum : uint8_t { AUTOACK_NONE = 0, AUTOACK_SENT = 1, AUTOACK_FAILED = 2 };

struct CslTxn {
  bool active;
  bool outcomeReady;
  uint8_t state;
  uint8_t outcome;
  uint8_t sent;
  bool sawInvalid;
  bool targetStored;
  bool anchored;     // the train is timed on an anchor, so its end is fixed
  bool lbtPending;   // the listen before the first frame is not done yet
  bool gated;        // the first frame leaves from RX, after a clear listen
  uint8_t lbtTries;  // listens that found the channel busy
  CslJob job;
  uint64_t listenAtUs;
  uint64_t startUs;
  uint64_t wakeEndUs;
  uint64_t windowEndUs;
  uint64_t guardUs;
  uint8_t ack[MAX_RX_RAW];
  uint8_t ackLen;
  CslFollow follow;
};

// The frame the receiver finished, as the CC1101 blob [len][payload][rssi][lqi | crc].
struct RxAssembler {
  uint8_t buf[MAX_RX_RAW];
  uint8_t freqest;
  uint8_t got;
  bool ready;
};

// ------------------------------------------------------------ task plumbing

static QueueHandle_t qCmd;      // serialTask -> radioTask, one request each
static QueueHandle_t qCmdFree;  // free CmdMsg slots
static QueueHandle_t qOut;      // radioTask -> serialTask, bytes to put on the wire
static QueueHandle_t qOutFree;  // free OutMsg slots

static CmdMsg cmdPool[CMD_POOL];
static OutMsg outPool[OUT_POOL];

static TaskHandle_t radioTaskHandle = nullptr;

// The host aborts a running stream by sending any byte. serialTask counts every
// byte it reads, the stream loop compares against the count it started with.
static std::atomic<uint32_t> hostByteSeq(0);
// Bumped by the DIO1 interrupt. The radio task takes the frame when it moves.
static std::atomic<uint32_t> rxIrqSeq(0);
// Low 32 bits of the end-of-frame instant. A 32-bit store is atomic on this
// core, a 64-bit one is not, and the high word is rebuilt from the clock.
static std::atomic<uint32_t> rxDoneStampLo(0);
// Milliseconds (low 32 bits) of the last valid request. serialTask stores it on
// every request with a good CRC, radioTask stores it again when a command ends,
// because a long RECV or STREAM leaves the host waiting for the reply.
static std::atomic<uint32_t> hostRequestMs(0);

static const uint32_t NOTIFY_RADIO = 0x1;
static const uint32_t NOTIFY_ALL_BITS = 0xFFFFFFFFUL;

// ------------------------------------------------------- radio task owned state

// The CC1101 register file the host loads and reads back. PATABLE takes the burst
// writes to 0x3E.
static uint8_t ccRegs[CC_CONFIG_REGS];
static uint8_t ccPatable[8];

// Budget in ms set by CMD_AIRTIME. 0 means no cap. No cap after boot and after CMD_RESET.
static uint32_t airBudgetMs = 0;
// Listen before talk, set by CMD_LBT. Off after boot and after CMD_RESET.
static bool lbtOn = false;
static int8_t lbtThresholdDbm = LBT_THRESHOLD_DEFAULT_DBM;
// The instant from which the RSSI of the receiver is valid: set when the chip enters RX.
static uint64_t rxValidUs = 0;

static RxAssembler rx;
static uint32_t rxIrqSeenSeq = 0;
static uint64_t lastPumpUs = 0;
static uint64_t hostSpiHoldoffUs = 0;

static CslJob cslJobs[CSL_JOB_POOL];
static uint8_t cslJobFree;
static uint8_t cslJobFreeCount;
static CslNeighbor cslNeighbors[CSL_NEIGHBORS];
static uint8_t cslCursor;
static CslAnchor cslAnchors[CSL_ANCHORS];
static CslCompletion cslComplete[CSL_COMPLETE_DEPTH];
static uint8_t cslCompleteHead, cslCompleteTail, cslCompleteCount;
static CslInbound cslInbound[CSL_INBOUND_DEPTH];
static uint8_t cslInboundHead, cslInboundTail, cslInboundCount;
static CslTxn txn;
static bool exclusiveWaiting;

// Last coordinator seen on a request. An uplink caught between transactions is
// answered with it, the same Enhanced ACK the host would send one poll later.
// A passive receive, a flush or a reset forgets it: a host that stops
// transmitting must not leave the bridge answering on its behalf.
static uint8_t coordEUI[8];
static bool coordKnown;
static uint16_t coordPhase;

// False at boot, after manualReset and after a chip reset that the idle tick
// detects. The host sets it with CMD_CONFIGURED once its register set is loaded.
// A chip with reset values must not transmit: it is not on the tado channel.
static bool configured;
// The last sxInit or calibration of the SX1262 succeeded.
static bool radioOk;
static bool hostLapsed;
static uint64_t lastIocfgCheckUs;

// Airtime of the last hour, in 61 one-minute buckets.
static uint32_t airBucket[AIR_BUCKETS];
static uint64_t airMinute;
static uint32_t airSumUs;

// The pair window the host opened with CMD_PAIRWIN. The beacons take seq and
// frame counter from the block the host reserved for them.
struct PairWindow {
  bool active;
  uint64_t endUs;
  mbedtls_aes_context aes;
  uint8_t coord[8];
  uint8_t seq;
  uint32_t ctr;
  uint8_t left;
  uint8_t sent;
};
static PairWindow pairWin;

static uint8_t respBuf[MAX_PAYLOAD];

// ----------------------------------------------------------------- small tools

static inline uint64_t nowUs() {
  return (uint64_t)esp_timer_get_time();
}

// Rebuild a full 64-bit instant from the low word an interrupt stored. Exact for
// anything the clock passed in the last wrap period (about 71 minutes).
static uint64_t stampFromLo(uint32_t lo) {
  uint64_t now = nowUs();
  uint64_t full = (now & ~0xFFFFFFFFULL) | (uint64_t)lo;
  if (full > now) full -= 0x100000000ULL;
  return full;
}

static uint16_t macFCS(const uint8_t *p, size_t n) {
  uint16_t crc = 0;
  for (size_t i = 0; i < n; i++) {
    crc ^= p[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 1) ? ((crc >> 1) ^ 0x8408) : (crc >> 1);
    }
  }
  return crc;
}

// CRC-8, polynomial 0x07, init 0x00, no reflection, no final xor.
static uint8_t crc8(uint8_t crc, const uint8_t *p, size_t n) {
  for (size_t i = 0; i < n; i++) {
    crc ^= p[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static inline uint32_t nowMs32() {
  return (uint32_t)(nowUs() / 1000ULL);
}

// ------------------------------------------------------------------------ airtime

// Move the window to the current minute and drop the buckets that left it.
static void airAdvance() {
  uint64_t minute = nowUs() / AIR_BUCKET_US;
  if (minute <= airMinute) return;
  uint64_t steps = minute - airMinute;
  if (steps >= AIR_BUCKETS) {
    memset(airBucket, 0, sizeof(airBucket));
    airSumUs = 0;
  } else {
    for (uint64_t i = 1; i <= steps; i++) {
      uint32_t *b = &airBucket[(airMinute + i) % AIR_BUCKETS];
      airSumUs -= *b;
      *b = 0;
    }
  }
  airMinute = minute;
}

// Time on air of one frame of n payload bytes.
static uint32_t airFrameUs(uint8_t n) {
  return ((uint32_t)n + AIR_FRAME_OVERHEAD) * AIR_BYTE_US;
}

// Count one transmitted frame of n payload bytes.
static void airCount(uint8_t n) {
  airAdvance();
  uint32_t us = airFrameUs(n);
  airBucket[airMinute % AIR_BUCKETS] += us;
  airSumUs += us;
}

// Load per-minute values in ms, oldest first, the last one for the current minute. The
// array with the larger total wins, whole: a preload sent to a bridge that did not reset
// must not erase what it counted, and a bucket by bucket maximum would copy one busy
// minute into every bucket as the host and the window shift against each other.
static void airPreload(const uint8_t *p) {
  airAdvance();
  uint32_t sumUs = 0;
  for (uint8_t k = 0; k < AIR_BUCKETS; k++) {
    uint16_t ms = ((uint16_t)p[2 * k] << 8) | p[2 * k + 1];
    if (ms > AIR_BUCKET_MAX_MS) ms = AIR_BUCKET_MAX_MS;
    sumUs += (uint32_t)ms * 1000U;
  }
  if (sumUs <= airSumUs) return;
  for (uint8_t k = 0; k < AIR_BUCKETS; k++) {
    uint16_t ms = ((uint16_t)p[2 * k] << 8) | p[2 * k + 1];
    if (ms > AIR_BUCKET_MAX_MS) ms = AIR_BUCKET_MAX_MS;
    uint8_t age = AIR_BUCKETS - 1 - k;
    airBucket[(airMinute % AIR_BUCKETS + AIR_BUCKETS - age) % AIR_BUCKETS] = (uint32_t)ms * 1000U;
  }
  airSumUs = sumUs;
}

static uint32_t airMs() {
  airAdvance();
  return airSumUs / 1000U;
}

// With a budget set, a transmission that is not an Enhanced ACK starts only below the budget.
// The frames of a job that started go out whole, so one job can cross the budget.
static bool airAllowed() {
  return airBudgetMs == 0 || airMs() < airBudgetMs;
}

// ------------------------------------------------------------------ interrupts

static void IRAM_ATTR dio1Isr() {
  // DIO1 carries RxDone only, so the rising edge is the end-of-frame instant. It stays high
  // until the radio task clears the flag: a frame that ends before that leaves no edge.
  rxDoneStampLo.store((uint32_t)esp_timer_get_time(), std::memory_order_relaxed);
  rxIrqSeq.fetch_add(1, std::memory_order_relaxed);
  if (radioTaskHandle != nullptr) {
    BaseType_t woken = pdFALSE;
    xTaskNotifyFromISR(radioTaskHandle, NOTIFY_RADIO, eSetBits, &woken);
    if (woken) portYIELD_FROM_ISR();
  }
}

// ------------------------------------------------------------- radio primitives

static void rxReset();

// Continuous RX from any state. A frame on the air is lost.
static void ccArmRx() {
  sxRxStart(MAX_TX_FRAME);
  rxValidUs = nowUs() + LBT_SETTLE_US;
}

// Arm the receiver only when it is not receiving, so a frame on the air or waiting in the
// buffer survives.
static void ccEnsureRx() {
  if (sxMode() == SX_MODE_RX) return;
  ccArmRx();
  rxReset();
}

// Receiver off, where the CC1101 goes IDLE.
static void ccIdle() {
  sxStandby(SX_STANDBY_XOSC);
}

static void ccRegsReset() {
  memset(ccRegs, 0, sizeof(ccRegs));
  memset(ccPatable, 0, sizeof(ccPatable));
  ccRegs[CC_IOCFG0] = GDO_CFG_LOW;
  ccRegs[CC_IOCFG2] = GDO_CFG_LOW;
}

// Hardware reset of the SX1262 with the tado PHY, and a register file as the CC1101 firmware
// leaves it after its reset. The chip waits in STDBY_XOSC.
static void manualReset() {
  configured = false;
  ccRegsReset();
  radioOk = sxInit(MAX_TX_FRAME);
}

// SCAL calibrates the chip, or starts it again after a failed start. FSCAL1 carries the outcome
// as a CC1101 synthesizer lock.
static void ccCalibrate() {
  radioOk = radioOk ? sxCalibrate() : sxInit(MAX_TX_FRAME);
  ccRegs[CC_FSCAL1] = radioOk ? FSCAL1_LOCKED : FSCAL1_NO_LOCK;
}

// RSSI in the CC1101 format from an SX1262 reading, dBm = -raw / 2.
static uint8_t ccRssiRaw(uint8_t sxRaw) {
  int v = 2 * RSSI_OFFSET_DBM - (int)sxRaw;
  if (v > 127) v = 127;
  if (v < -128) v = -128;
  return (uint8_t)(int8_t)v;
}

// The CC1101 chip status byte: state in bits 6:4, FIFO room in bits 3:0.
static uint8_t ccChipStatus() {
  uint8_t m = sxMode();
  uint8_t state = (m == SX_MODE_RX) ? CC_STATE_RX : (m == SX_MODE_TX) ? CC_STATE_TX : 0;
  return (uint8_t)(state << CC_STATE_SHIFT) | CC_STATUS_FIFO;
}

static uint8_t ccStatusReg(uint8_t addr) {
  switch (addr) {
    case CC_PARTNUM: return CC_PARTNUM_VALUE;
    case CC_VERSION: return CC_VERSION_VALUE;
    case CC_RSSI: return ccRssiRaw(sxRssiInst());
    case CC_MARCSTATE: {
      uint8_t m = sxMode();
      if (m == SX_MODE_RX) return MARC_RX;
      if (m == SX_MODE_TX) return MARC_TX;
      if (m == SX_MODE_FS) return MARC_FSTXON;
      return MARC_IDLE;
    }
    default: return 0;  // FREQEST, LQI, PKTSTATUS and the FIFO counts have no SX1262 value
  }
}

// STX and the strobes not listed do nothing. The host sends from the TX FIFO only in a bench
// tool (Bridge.Send); this bridge transmits from its own frame paths alone.
static void ccEmulStrobe(uint8_t addr) {
  switch (addr) {
    case CC_SRES:
      manualReset();
      rxReset();
      break;
    case CC_SCAL: ccCalibrate(); break;
    case CC_SRX:
      ccArmRx();
      rxReset();
      break;
    case CC_SIDLE: ccIdle(); break;
    case CC_SFRX:
      sxClearIrq(SX_IRQ_RX_ALL);
      rxReset();
      break;
    default: break;
  }
}

static uint8_t *ccRegPtr(uint8_t addr, uint8_t k) {
  if (addr == CC_PATABLE) return &ccPatable[k % sizeof(ccPatable)];
  if (addr == CC_FIFO) return nullptr;
  uint16_t idx = (uint16_t)addr + k;
  return idx < CC_CONFIG_REGS ? &ccRegs[idx] : nullptr;
}

// A CC1101 SPI transaction on the register file, in place (CC1101 datasheet 10): a header
// byte returns the chip status, a data byte the register. Writes change only the register
// file. FSCTRL0 is stored and not applied: the TCXO needs no crystal correction, and FREQEST
// reads 0.
static void ccEmulXfer(uint8_t *p, size_t n) {
  size_t i = 0;
  while (i < n) {
    uint8_t h = p[i];
    bool read = (h & READ_BIT) != 0;
    bool burst = (h & BURST_BIT) != 0;
    uint8_t addr = h & ADDR_MASK;
    p[i++] = ccChipStatus();
    if (addr >= CC_SRES && addr <= CC_LAST_STROBE) {
      if (!burst) ccEmulStrobe(addr);
      else if (i < n) p[i++] = ccStatusReg(addr);
      continue;
    }
    uint8_t k = 0;
    do {
      if (i >= n) break;
      uint8_t *reg = ccRegPtr(addr, k++);
      if (read) p[i] = (reg != nullptr) ? *reg : 0;
      else if (reg != nullptr) *reg = p[i];
      i++;
    } while (burst);
  }
}

// ------------------------------------------------------------- listen before talk

// One sample shows a signal when the RSSI is at or above the threshold or a sync word was
// found. The SX1262 reads -2 * dBm, so the comparison runs in half dB steps.
static bool lbtSampleBusy(uint8_t rssiRaw, uint16_t irq) {
  return -(int)rssiRaw >= 2 * (int)lbtThresholdDbm || (irq & SX_IRQ_SYNC_WORD) != 0;
}

// The instant to start the listen so that it ends by the start of the first frame.
static uint64_t lbtListenAt(uint64_t startUs, uint64_t now) {
  return (startUs > now + LBT_LEAD_US) ? startUs - LBT_LEAD_US : now;
}

static uint32_t lbtBackoffUs() {
  return (esp_random() % (LBT_BACKOFF_MAX_UNITS + 1U)) * LBT_BACKOFF_UNIT_US;
}

// One listen: put the chip in RX if it is not, wait until its RSSI is valid, then sample the
// RSSI and the sync word flag over LBT_LISTEN_US. The channel is clear when no sample shows a
// signal. A sync word flag from before the listen belongs to a frame that ended or that the
// RSSI still shows, so it is cleared first.
static bool lbtListenOnce() {
  ccEnsureRx();
  while (nowUs() < rxValidUs) {
  }
  sxClearIrq(SX_IRQ_SYNC_WORD);
  uint64_t end = nowUs() + LBT_LISTEN_US;
  do {
    if (lbtSampleBusy(sxRssiInst(), sxIrq())) return false;
  } while (nowUs() < end);
  return true;
}

// Listen from `notBeforeUs` on, with a blocking backoff between the tries. True when a
// listen found the channel clear.
static bool lbtClear(uint64_t notBeforeUs) {
  for (uint8_t i = 0; i < LBT_TRIES; i++) {
    uint64_t from = (i == 0) ? notBeforeUs : nowUs() + lbtBackoffUs();
    while (nowUs() < from) {
      esp_task_wdt_reset();
    }
    if (lbtListenOnce()) return true;
  }
  return false;
}

// ------------------------------------------------------------------ frame sends

// An SX1262 that reset alone has lost the tado PHY. The register file goes back to its reset
// state, so the host sees IOCFG0 change and runs Configure again; until then the bridge
// stays unconfigured. Returns true on such a reset.
static bool ccQuietIfReset() {
  if (!sxLostConfig()) return false;
  ccRegsReset();
  configured = false;
  rxReset();
  return true;
}

enum : uint8_t { SEND_OK = 0, SEND_FAIL = 1, SEND_BUSY = 2 };

// Transmit one variable-length frame ([len][payload], the chip appends the CRC).
//
// fromRx is the first frame after a listen. The CC1101 gates it with its hardware CCA; here
// the receiver is checked once more: a sync word, a frame waiting in the buffer or an RSSI at
// the threshold end in SEND_BUSY and send nothing. A chip that is not in RX takes the normal
// path.
// thenRx puts the receiver back after the frame, as TXOFF_MODE=RX does on the CC1101. A wake
// train passes false: its frames leave back to back from FS, and the receiver returns after
// the Data frame.
static uint8_t ccSendFrameEx(const uint8_t *payload, uint8_t n, bool fromRx, bool thenRx = true) {
  if (!configured || n == 0 || n > MAX_TX_FRAME) return SEND_FAIL;
  // A receive-only build leaves the receiver alone.
  if (!SX_TX_ENABLED) return SEND_FAIL;
  // Every transmission passes here, so a chip that reset in the middle of a job
  // chain never sends with reset registers.
  if (ccQuietIfReset()) return SEND_FAIL;
  if (fromRx && sxMode() == SX_MODE_RX) {
    uint16_t irq = sxIrq();
    if ((irq & (SX_IRQ_SYNC_WORD | SX_IRQ_RX_DONE)) != 0 || lbtSampleBusy(sxRssiInst(), 0)) return SEND_BUSY;
  }
  rxReset();
  bool ok = sxTransmit(payload, n, TX_TMO_MS * 1000UL);
  if (ok) airCount(n);
  if (!ok || thenRx) ccArmRx();
  return ok ? SEND_OK : SEND_FAIL;
}

static bool ccSendFrame(const uint8_t *payload, uint8_t n) {
  return ccSendFrameEx(payload, n, false) == SEND_OK;
}

// True when a frame of n payload bytes that starts at startUs still ends within
// HOST_TRAIN_MAX_US of the first one.
static bool trainFits(uint64_t firstUs, uint64_t startUs, uint8_t n) {
  return startUs + airFrameUs(n) - firstUs <= HOST_TRAIN_MAX_US;
}

// Transmit `count` packed [len][bytes] sub-frames back-to-back, with no serial
// round-trip between them: the gap is SPI plus state machine time, not the
// ~2.5 ms USB serial latency a host-driven burst pays. The train stops before a frame that
// would end more than HOST_TRAIN_MAX_US after the first one started. `gated` sends the first
// frame from RX, and `busy` reports that the chip refused it. Returns the frames sent.
static uint8_t ccSendTrain(const uint8_t *p, size_t len, uint8_t count, bool gated, bool *busy) {
  uint8_t sent = 0;
  size_t off = 0;
  uint64_t firstUs = 0;
  for (uint8_t i = 0; i < count; i++) {
    if (off >= len) break;
    uint8_t n = p[off];
    if (n == 0 || n > MAX_TX_FRAME || off + 1 + (size_t)n > len) break;
    if (sent == 0) firstUs = nowUs();
    else if (!trainFits(firstUs, nowUs(), n)) break;
    uint8_t r = ccSendFrameEx(p + off + 1, n, gated && sent == 0, false);
    if (r != SEND_OK) {
      if (r == SEND_BUSY && busy != nullptr) *busy = true;
      break;
    }
    off += 1 + (size_t)n;
    sent++;
  }
  if (sent != 0) ccEnsureRx();
  return sent;
}

// ------------------------------------------------------------------- auto-ACK

static uint8_t ccAutoAck(const uint8_t *raw, size_t rawLen, const uint8_t *coordinator, uint16_t phase) {
  if (rawLen < 19 || !(raw[rawLen - 1] & CRC_OK_BIT)) return AUTOACK_NONE;
  uint8_t macLen = raw[0];
  if (macLen < 16 || rawLen != (size_t)1 + macLen + 2) return AUTOACK_NONE;
  const uint8_t *mac = raw + 1;
  if (!((mac[0] == 0x69 || mac[0] == 0x79) && mac[1] == 0xEC) || memcmp(mac + 3, coordinator, 8)) {
    return AUTOACK_NONE;
  }
  uint8_t ack[29] = {0x42, 0xEE, mac[2]};
  memcpy(ack + 3, mac + 11, 5);
  ack[8] = 0xC5;
  ack[9] = 0x1B;
  ack[10] = 0x00;
  memcpy(ack + 11, coordinator, 8);
  ack[19] = 0x04;
  ack[20] = 0x0D;
  ack[21] = (uint8_t)phase;
  ack[22] = (uint8_t)(phase >> 8);
  ack[23] = 0x35;
  ack[24] = 0x0C;
  ack[25] = 0x80;
  ack[26] = 0x3F;
  uint16_t fcs = macFCS(ack, 27);
  ack[27] = (uint8_t)fcs;
  ack[28] = (uint8_t)(fcs >> 8);
  return ccSendFrame(ack, sizeof(ack)) ? AUTOACK_SENT : AUTOACK_FAILED;
}

static void coordFromData(const uint8_t *data, uint8_t *out) {
  memcpy(out, data + 11, 5);
  out[5] = 0xC5;
  out[6] = 0x1B;
  out[7] = 0x00;
}

static void coordRemember(const uint8_t *eui, uint16_t phase) {
  // A CSLTX without the follow flag carries no phase. Keep the last one known
  // for this coordinator instead of advertising phase 0.
  if (phase == 0 && coordKnown && coordPhase != 0 && memcmp(coordEUI, eui, 8) == 0) {
    phase = coordPhase;
  }
  memcpy(coordEUI, eui, 8);
  coordPhase = phase;
  coordKnown = true;
}

// --------------------------------------------------------------- RX assembling

static void rxReset() {
  rx.got = 0;
  rx.freqest = 0;
  rx.ready = false;
}

// The SX1262 hands over whole frames, so no frame is ever half assembled.
static bool rxAssembling() {
  return false;
}

// Take the frame the receiver finished into the assembler as the CC1101 blob
// [len][payload][rssi][lqi | crc]. Returns true when a frame is ready. The receiver
// stays in RX, so a frame that follows is not lost while this one is read. FREQEST is 0:
// the SX1262 measures no frequency error in GFSK.
static bool rxPump() {
  lastPumpUs = nowUs();
  if (rx.ready) return true;
  uint8_t n = 0;
  bool crcOk = false;
  uint8_t rssi = 0;
  if (!sxTakeFrame(rx.buf + 1, MAX_TX_FRAME, &n, &crcOk, &rssi) || n == 0) return false;
  rx.buf[0] = n;
  rx.buf[1 + n] = ccRssiRaw(rssi);
  rx.buf[2 + n] = crcOk ? CRC_OK_BIT : 0;
  rx.got = (uint8_t)(n + 3);
  rx.freqest = 0;
  rx.ready = true;
  return true;
}

// Hand the assembled frame to the caller, with the FREQEST of that frame.
static void rxTake(uint8_t *dst, size_t *len, uint64_t *stampUs, uint8_t *freqest) {
  *len = rx.got;
  memcpy(dst, rx.buf, rx.got);
  if (stampUs != nullptr) *stampUs = stampFromLo(rxDoneStampLo.load(std::memory_order_relaxed));
  if (freqest != nullptr) *freqest = rx.freqest;
  rxReset();
}

static bool rxCrcOk(const uint8_t *raw, size_t len) {
  return len >= 3 && (raw[len - 1] & CRC_OK_BIT);
}

static bool rxDuePump(uint64_t now) {
  uint32_t seq = rxIrqSeq.load(std::memory_order_relaxed);
  if (seq != rxIrqSeenSeq) {
    rxIrqSeenSeq = seq;
    return true;
  }
  uint32_t tickMs = rxAssembling() ? RX_ASSEMBLE_TICK_MS : RX_HOUSEKEEP_TICK_MS;
  return now - lastPumpUs >= (uint64_t)tickMs * 1000ULL;
}

static void radioWait(uint32_t ms);

// Wait for one complete frame on an already armed receiver.
static bool rxWaitFrame(uint32_t tmoMs, uint8_t *dst, size_t *len, uint64_t *stampUs,
                        uint8_t *freqest) {
  uint64_t deadline = nowUs() + (uint64_t)tmoMs * 1000ULL;
  for (;;) {
    uint64_t now = nowUs();
    if (rxDuePump(now) && rxPump()) {
      rxTake(dst, len, stampUs, freqest);
      return true;
    }
    now = nowUs();
    if (now >= deadline) return false;
    uint64_t left = (deadline - now) / 1000ULL;
    uint32_t tickMs = rxAssembling() ? RX_ASSEMBLE_TICK_MS : RX_HOUSEKEEP_TICK_MS;
    radioWait(left < tickMs ? (uint32_t)left : tickMs);
    esp_task_wdt_reset();
  }
}

// ------------------------------------------------------------ pair window TX

static void pairAes(const uint8_t *in, uint8_t *out) {
  mbedtls_aes_crypt_ecb(&pairWin.aes, MBEDTLS_AES_ENCRYPT, in, out);
}

// XOR the CCM* keystream (counter blocks A1, A2, ...) over n bytes at p.
static void ccmCrypt(const uint8_t *frame, uint8_t *p, uint8_t n) {
  uint8_t a[16], s[16];
  a[0] = 0x01;  // L - 1
  memcpy(a + 1, frame, CCM_NONCE_LEN);
  for (uint8_t off = 0, i = 1; off < n; off += 16, i++) {
    a[14] = 0;
    a[15] = i;
    pairAes(a, s);
    for (uint8_t j = 0; j < 16 && off + j < n; j++) p[off + j] ^= s[j];
  }
}

// The CCM* MIC (M=4, L=2) over the aad frame[0:16] and n plaintext bytes.
static void ccmMic(const uint8_t *frame, const uint8_t *pt, uint8_t n, uint8_t *mic) {
  uint8_t x[16], b[16];
  b[0] = 0x49;  // Adata, M=4, L=2
  memcpy(b + 1, frame, CCM_NONCE_LEN);
  b[14] = 0;
  b[15] = n;
  pairAes(b, x);
  uint8_t aad[32] = {0x00, CCM_AAD_LEN};
  memcpy(aad + 2, frame, CCM_AAD_LEN);
  for (uint8_t off = 0; off < sizeof(aad); off += 16) {
    for (uint8_t j = 0; j < 16; j++) b[j] = x[j] ^ aad[off + j];
    pairAes(b, x);
  }
  for (uint8_t off = 0; off < n; off += 16) {
    for (uint8_t j = 0; j < 16; j++) b[j] = x[j] ^ ((off + j < n) ? pt[off + j] : 0);
    pairAes(b, x);
  }
  b[0] = 0x01;
  memcpy(b + 1, frame, CCM_NONCE_LEN);
  b[14] = 0;
  b[15] = 0;
  uint8_t s0[16];
  pairAes(b, s0);
  for (uint8_t j = 0; j < CCM_MIC_LEN; j++) mic[j] = x[j] ^ s0[j];
}

// Answer an Enhanced Beacon Request of a joining device with an Enhanced
// Beacon (tado/beacon.go). The commercial bridge answers with no wake train,
// 2.4 to 7.4 ms after the request, while the device still listens; a round
// trip through the host is slower than that. raw is the received blob that
// was taken last, so the last RxDone stamp is its end.
static void pairAnswer(const uint8_t *raw, size_t rawLen) {
  if (!pairWin.active) return;
  uint64_t now = nowUs();
  if (now >= pairWin.endUs) {
    pairWin.active = false;
    return;
  }
  uint64_t stampUs = stampFromLo(rxDoneStampLo.load(std::memory_order_relaxed));
  if (!airAllowed()) return;
  if (pairWin.left == 0 || rawLen != (size_t)1 + EBR_LEN + 2 || raw[0] != EBR_LEN || !rxCrcOk(raw, rawLen)) return;
  const uint8_t *req = raw + 1;
  if (req[0] != 0x0B || req[1] != 0xE8 || req[15] != SEC_CTRL) return;
  if (memcmp(req + 7, pairWin.coord, 5) == 0) return;  // our own address is no joiner
  // Sealed part: counter(4) command(1) FCS(2).
  uint8_t pt[EBR_LEN - CCM_AAD_LEN - CCM_MIC_LEN];
  memcpy(pt, req + CCM_AAD_LEN, sizeof(pt));
  ccmCrypt(req, pt, sizeof(pt));
  uint8_t mic[CCM_MIC_LEN];
  ccmMic(req, pt, sizeof(pt), mic);
  if (memcmp(mic, req + EBR_LEN - CCM_MIC_LEN, CCM_MIC_LEN) != 0 || pt[4] != MAC_CMD_BEACON_REQ) return;
  uint8_t fcsIn[CCM_AAD_LEN + 5];
  memcpy(fcsIn, req, CCM_AAD_LEN);
  memcpy(fcsIn + CCM_AAD_LEN, pt, 5);
  uint16_t fcs = macFCS(fcsIn, sizeof(fcsIn));
  if (pt[5] != (uint8_t)fcs || pt[6] != (uint8_t)(fcs >> 8)) return;

  uint8_t eb[EB_LEN] = {0x18, 0xEC, pairWin.seq, 0xCD, 0xAB};
  memcpy(eb + 5, req + 7, 8);
  memcpy(eb + 13, pairWin.coord, 8);
  eb[21] = SEC_CTRL;
  eb[22] = (uint8_t)pairWin.ctr;
  eb[23] = (uint8_t)(pairWin.ctr >> 8);
  eb[24] = (uint8_t)(pairWin.ctr >> 16);
  eb[25] = (uint8_t)(pairWin.ctr >> 24);
  fcs = macFCS(eb, 26);
  eb[26] = (uint8_t)fcs;
  eb[27] = (uint8_t)(fcs >> 8);
  ccmMic(eb, eb + CCM_AAD_LEN, EB_LEN - CCM_AAD_LEN - CCM_MIC_LEN, eb + EB_LEN - CCM_MIC_LEN);
  ccmCrypt(eb, eb + CCM_AAD_LEN, EB_LEN - CCM_AAD_LEN - CCM_MIC_LEN);
  // The listen runs in the gap before the beacon, after the request has faded from the
  // RSSI. A busy channel, or a chip that refuses the STX, sends nothing and keeps the seq
  // and the counter: the device asks again 8 s later.
  if (lbtOn && !lbtClear(stampUs + LBT_SETTLE_US)) return;
  // A stamp from an earlier frame lies in the past and waits for nothing.
  uint64_t at = stampUs + EB_GAP_US;
  while (nowUs() < at) {
  }
  uint8_t sentState = ccSendFrameEx(eb, EB_LEN, lbtOn);
  if (sentState == SEND_BUSY) return;
  pairWin.seq++;
  pairWin.ctr++;
  pairWin.left--;
  if (sentState == SEND_OK && pairWin.sent != 0xFF) pairWin.sent++;
}

static void pairClose() {
  pairWin.active = false;
  pairWin.sent = 0;
}

// ------------------------------------------------------------- serial out queue

static OutMsg *outAcquire() {
  OutMsg *m = nullptr;
  while (xQueueReceive(qOutFree, &m, pdMS_TO_TICKS(100)) != pdTRUE) {
    esp_task_wdt_reset();
  }
  return m;
}

static void outPost(OutMsg *m) {
  while (xQueueSend(qOut, &m, pdMS_TO_TICKS(100)) != pdTRUE) {
    esp_task_wdt_reset();
  }
}

static void sendResp(uint8_t status, const uint8_t *p, size_t n) {
  if (n > MAX_PAYLOAD) n = MAX_PAYLOAD;
  OutMsg *m = outAcquire();
  m->data[0] = RSP_MAGIC;
  m->data[1] = status;
  m->data[2] = (uint8_t)(n >> 8);
  m->data[3] = (uint8_t)(n & 0xFF);
  if (n) memcpy(m->data + 4, p, n);
  m->len = (uint16_t)(4 + n);
  outPost(m);
}

static void sendStreamMarker(uint8_t marker) {
  OutMsg *m = outAcquire();
  m->data[0] = RSP_MAGIC;
  m->data[1] = marker;
  m->data[2] = 0x00;
  m->len = 3;
  outPost(m);
}

// One received frame on the stream: 5A F1 <n> <n bytes> <freqest> <xor8>.
static void sendStreamFrame(const uint8_t *p, uint8_t n, uint8_t freqest) {
  OutMsg *m = outAcquire();
  m->data[0] = RSP_MAGIC;
  m->data[1] = STREAM_FRAME;
  m->data[2] = n;
  memcpy(m->data + 3, p, n);
  m->data[3 + n] = freqest;
  uint8_t x = freqest;
  for (uint8_t i = 0; i < n; i++) x ^= p[i];
  m->data[4 + n] = x;
  m->len = (uint16_t)(5 + n);
  outPost(m);
}

// ------------------------------------------------------------------- CSL anchors

// An anchor has no age limit: only CSL_ANCHOR_MISS_LIMIT misses in a row drop
// it, as in the commercial bridge. The guard grows with its age instead.
static int8_t anchorIndex(const uint8_t *shortAddr) {
  for (uint8_t i = 0; i < CSL_ANCHORS; i++) {
    CslAnchor *a = &cslAnchors[i];
    if (!a->valid) continue;
    if (a->shortAddr[0] != shortAddr[0] || a->shortAddr[1] != shortAddr[1]) continue;
    return (int8_t)i;
  }
  return -1;
}

static void anchorStore(const uint8_t *shortAddr, const uint8_t *ack, uint64_t stampUs) {
  uint16_t phase = (uint16_t)ack[21] | ((uint16_t)ack[22] << 8);
  int8_t idx = anchorIndex(shortAddr);
  if (idx < 0) {
    int8_t pick = -1;
    for (uint8_t i = 0; i < CSL_ANCHORS; i++) {
      if (!cslAnchors[i].valid) {
        pick = (int8_t)i;
        break;
      }
    }
    if (pick < 0) {  // evict the least recently refreshed, never a fixed slot
      pick = 0;
      for (uint8_t i = 1; i < CSL_ANCHORS; i++) {
        if (cslAnchors[i].refreshUs < cslAnchors[pick].refreshUs) pick = (int8_t)i;
      }
    }
    idx = pick;
  }
  CslAnchor *a = &cslAnchors[idx];
  a->shortAddr[0] = shortAddr[0];
  a->shortAddr[1] = shortAddr[1];
  // A missed DIO1 edge leaves a stale timestamp.
  uint64_t now = nowUs();
  if (stampUs > now || now - stampUs > CSL_PERIOD_US) stampUs = now;
  // The phase field counts back to a sample the peer woke on, and every later
  // sample is one CSL period apart, so only the phase modulo one period counts.
  // A phase of up to 21 s can exceed the uptime: the anchor then moves one
  // period forward, instead of an underflow to a sample that never comes.
  uint64_t back = ((uint64_t)phase * CSL_UNIT_US) % CSL_PERIOD_US;
  a->sampleUs = (stampUs >= back) ? stampUs - back : stampUs + CSL_PERIOD_US - back;
  a->refreshUs = stampUs;
  a->misses = 0;
  a->valid = true;
}

static void anchorMiss(const uint8_t *shortAddr) {
  int8_t idx = anchorIndex(shortAddr);
  if (idx < 0) return;
  if (++cslAnchors[idx].misses >= CSL_ANCHOR_MISS_LIMIT) cslAnchors[idx].valid = false;
}

// A bootstrap job skips an anchor that has missed once. A device that rebooted
// samples at a new phase, and its uplinks carry no CSL IE that would show it.
static int8_t anchorForJob(const CslJob *job) {
  int8_t idx = anchorIndex(job->shortAddr);
  if (idx >= 0 && (job->flags & CSL_FLAG_BOOTSTRAP) && cslAnchors[idx].misses != 0) return -1;
  return idx;
}

// Half the wake train around a sample: the base guard plus the clock error that
// can build up since the last refresh.
static uint32_t anchorGuardUs(const CslAnchor *a, uint64_t sampleUs) {
  uint64_t age = (sampleUs > a->refreshUs) ? sampleUs - a->refreshUs : 0;
  uint64_t guard = CSL_SAMPLE_GUARD_US + age * CSL_GUARD_PPM / 1000000ULL;
  return (guard > CSL_GUARD_MAX_US) ? CSL_GUARD_MAX_US : (uint32_t)guard;
}

// The first sample that leaves the whole guard before it. A sample closer than
// the guard would get a train that starts late, and a peer whose clock drifted
// early would sample before it: the job waits one more period instead.
static uint64_t anchorNextSample(int8_t idx, uint64_t now, uint32_t *guardUs) {
  const CslAnchor *a = &cslAnchors[idx];
  uint64_t sample = a->sampleUs;
  uint64_t earliest = now + CSL_SAMPLE_GUARD_US;
  if (sample < earliest) sample += ((earliest - sample + CSL_PERIOD_US - 1) / CSL_PERIOD_US) * CSL_PERIOD_US;
  // The guard is below half a period, so this takes at most one more period.
  for (;;) {
    uint32_t guard = anchorGuardUs(a, sample);
    if (sample >= now + guard) {
      *guardUs = guard;
      return sample;
    }
    sample += CSL_PERIOD_US;
  }
}

// ------------------------------------------------------------- CSL job storage

static void cslPoolInit() {
  for (uint8_t i = 0; i < CSL_JOB_POOL; i++) {
    cslJobs[i].next = (i + 1 < CSL_JOB_POOL) ? (uint8_t)(i + 1) : CSL_JOB_NONE;
  }
  cslJobFree = 0;
  cslJobFreeCount = CSL_JOB_POOL;
  for (uint8_t i = 0; i < CSL_NEIGHBORS; i++) {
    cslNeighbors[i].used = false;
    cslNeighbors[i].head = CSL_JOB_NONE;
    cslNeighbors[i].tail = CSL_JOB_NONE;
    cslNeighbors[i].count = 0;
  }
}

static uint8_t cslJobAlloc() {
  if (cslJobFree == CSL_JOB_NONE) return CSL_JOB_NONE;
  uint8_t idx = cslJobFree;
  cslJobFree = cslJobs[idx].next;
  cslJobFreeCount--;
  cslJobs[idx].next = CSL_JOB_NONE;
  return idx;
}

static void cslJobRelease(uint8_t idx) {
  cslJobs[idx].next = cslJobFree;
  cslJobFree = idx;
  cslJobFreeCount++;
}

static CslNeighbor *cslNeighborFind(const uint8_t *shortAddr, bool create) {
  CslNeighbor *freeSlot = nullptr;
  for (uint8_t i = 0; i < CSL_NEIGHBORS; i++) {
    CslNeighbor *e = &cslNeighbors[i];
    if (e->used && e->shortAddr[0] == shortAddr[0] && e->shortAddr[1] == shortAddr[1]) return e;
    if (!e->used && freeSlot == nullptr) freeSlot = e;
  }
  if (!create || freeSlot == nullptr) return nullptr;
  freeSlot->used = true;
  freeSlot->shortAddr[0] = shortAddr[0];
  freeSlot->shortAddr[1] = shortAddr[1];
  freeSlot->head = CSL_JOB_NONE;
  freeSlot->tail = CSL_JOB_NONE;
  freeSlot->count = 0;
  return freeSlot;
}

static void cslNeighborPush(CslNeighbor *e, uint8_t jobIdx) {
  cslJobs[jobIdx].next = CSL_JOB_NONE;
  if (e->tail == CSL_JOB_NONE) e->head = jobIdx;
  else cslJobs[e->tail].next = jobIdx;
  e->tail = jobIdx;
  e->count++;
}

static uint8_t cslNeighborPop(CslNeighbor *e) {
  uint8_t idx = e->head;
  if (idx == CSL_JOB_NONE) return CSL_JOB_NONE;
  e->head = cslJobs[idx].next;
  if (e->head == CSL_JOB_NONE) e->tail = CSL_JOB_NONE;
  e->count--;
  if (e->count == 0) e->used = false;
  return idx;
}

static void cslQueueClear() {
  for (uint8_t i = 0; i < CSL_NEIGHBORS; i++) {
    CslNeighbor *e = &cslNeighbors[i];
    if (e->used) {
      while (e->head != CSL_JOB_NONE) {
        uint8_t idx = e->head;
        e->head = cslJobs[idx].next;
        cslJobRelease(idx);
      }
    }
    e->used = false;
    e->head = CSL_JOB_NONE;
    e->tail = CSL_JOB_NONE;
    e->count = 0;
  }
  memset(cslComplete, 0, sizeof(cslComplete));
  memset(cslInbound, 0, sizeof(cslInbound));
  cslCompleteHead = cslCompleteTail = cslCompleteCount = 0;
  cslInboundHead = cslInboundTail = cslInboundCount = 0;
  // A synchronous CSLTX or a Router Advertisement still owes the host a reply,
  // so only a queued transaction is cancelled here.
  if (!txn.active || txn.job.kind == JOB_QUEUED) memset(&txn, 0, sizeof(txn));
}

// -------------------------------------------------------------- CSL validation

static bool cslAckValid(const uint8_t *raw, size_t rawLen, const uint8_t *data) {
  if (rawLen != 32 || raw[0] != 29) return false;
  const uint8_t *ack = raw + 1;
  if (ack[0] != 0x42 || ack[1] != 0xEE || ack[2] != data[2]) return false;
  if (memcmp(ack + 3, data + 11, 5) || ack[8] != 0xC5 || ack[9] != 0x1B || ack[10] != 0x00) return false;
  if (memcmp(ack + 11, data + 3, 8)) return false;
  uint16_t fcs = macFCS(ack, 27);
  return ack[27] == (uint8_t)fcs && ack[28] == (uint8_t)(fcs >> 8) && (raw[31] & CRC_OK_BIT);
}

static bool cslFollowMatch(const uint8_t *raw, size_t rawLen, const uint8_t *data) {
  if (rawLen < 19 || !(raw[rawLen - 1] & CRC_OK_BIT)) return false;
  if (rawLen != (size_t)1 + raw[0] + 2 || raw[0] < 16) return false;
  const uint8_t *mac = raw + 1;
  return (mac[0] == 0x69 || mac[0] == 0x79) && mac[1] == 0xEC &&
         memcmp(mac + 3, data + 11, 5) == 0 && mac[8] == 0xC5 && mac[9] == 0x1B &&
         mac[10] == 0x00 && mac[11] == data[3] && mac[12] == data[4];
}

// A full ring gets no Enhanced ACK: the device retransmits an uplink it sees
// unacknowledged, and an acknowledged uplink the ring cannot hold is lost.
static void cslStoreInbound(const uint8_t *raw, size_t rawLen, const uint8_t *coordinator,
                            uint16_t phase, uint8_t freqest) {
  if (rawLen > MAX_RX_RAW || cslInboundCount == CSL_INBOUND_DEPTH) return;
  if (coordinator != nullptr) ccAutoAck(raw, rawLen, coordinator, phase);
  if (!rxCrcOk(raw, rawLen) || raw[1] == 0x25) return;
  CslInbound *slot = &cslInbound[cslInboundTail];
  memcpy(slot->raw, raw, rawLen);
  slot->len = (uint8_t)rawLen;
  slot->freqest = freqest;
  cslInboundTail = (uint8_t)((cslInboundTail + 1) % CSL_INBOUND_DEPTH);
  cslInboundCount++;
}

static void cslStoreInboundForJob(const uint8_t *raw, size_t rawLen, const CslJob *job,
                                  uint8_t freqest) {
  uint8_t coordinator[8];
  coordFromData(job->data, coordinator);
  cslStoreInbound(raw, rawLen, coordinator, job->phase, freqest);
}

// ------------------------------------------------------------ CSL state machine

static uint8_t cslWireStatus(uint8_t outcome) {
  if (outcome == OUT_OK) return ST_OK;
  if (outcome == OUT_INPUT) return ST_INPUT;
  if (outcome == OUT_NOACK) return ST_NOACK;
  if (outcome == OUT_BADACK) return ST_BADACK;
  if (outcome == OUT_NOANCHOR) return ST_NOANCHOR;
  if (outcome == OUT_AIRTIME) return ST_AIRTIME;
  if (outcome == OUT_BUSY) return ST_BUSY;
  return ST_TXFAIL;
}

static void cslSetOutcome(uint8_t outcome) {
  txn.outcome = outcome;
  txn.outcomeReady = true;
}

static size_t cslAppendFollowBody(uint8_t *out, size_t off, const uint8_t *ack, uint8_t ackLen,
                                  const CslFollow *follow) {
  out[off++] = ackLen;
  memcpy(out + off, ack, ackLen);
  off += ackLen;
  out[off++] = follow->count;
  for (uint8_t i = 0; i < follow->count; i++) {
    out[off++] = follow->len[i];
    memcpy(out + off, follow->raw[i], follow->len[i]);
    off += follow->len[i];
  }
  out[off++] = follow->dropped;
  return off;
}

// Reply to a synchronous CMD_CSLTX.
static void cslReplySync() {
  uint8_t status = cslWireStatus(txn.outcome);
  if (txn.outcome != OUT_OK) {
    respBuf[0] = txn.sent;
    sendResp(status, respBuf, 1);
    return;
  }
  respBuf[0] = txn.sent;
  if ((txn.job.flags & CSL_FLAG_FOLLOW) == 0) {
    memcpy(respBuf + 1, txn.ack, txn.ackLen);
    sendResp(ST_OK, respBuf, (size_t)txn.ackLen + 1);
    return;
  }
  size_t off = cslAppendFollowBody(respBuf, 1, txn.ack, txn.ackLen, &txn.follow);
  sendResp(ST_OK, respBuf, off);
}

static void cslReplyRA() {
  respBuf[0] = txn.sent;
  sendResp(cslWireStatus(txn.outcome), respBuf, 1);
}

// Park a queued outcome in the completion ring. Returns false while the ring is
// full, in which case the transaction keeps its outcome until a poll drains one.
static bool cslPark() {
  if (cslCompleteCount == CSL_COMPLETE_DEPTH) return false;
  CslCompletion *done = &cslComplete[cslCompleteTail];
  memset(done, 0, sizeof(*done));
  done->id = txn.job.id;
  done->flags = txn.job.flags;
  done->status = cslWireStatus(txn.outcome);
  done->sent = txn.sent;
  done->ackLen = txn.ackLen;
  if (done->ackLen) memcpy(done->ack, txn.ack, done->ackLen);
  done->follow = txn.follow;
  cslCompleteTail = (uint8_t)((cslCompleteTail + 1) % CSL_COMPLETE_DEPTH);
  cslCompleteCount++;
  return true;
}

// Deliver a decided outcome. A parked OK never turns into a failure later: the
// state machine looks at the outcome before it looks at the transaction guard.
static void cslDeliver() {
  if (txn.job.kind == JOB_QUEUED) {
    if (!cslPark()) return;
  } else if (txn.job.kind == JOB_SYNC_CSL) {
    cslReplySync();
  } else {
    cslReplyRA();
  }
  memset(&txn, 0, sizeof(txn));
  ccEnsureRx();
}

static void cslCompleteNow(uint8_t outcome) {
  cslSetOutcome(outcome);
  cslDeliver();
}

static uint8_t cslSendWake(const CslJob *job, uint16_t rendez, bool gated) {
  uint8_t dst0 = job->shortAddr[0];
  uint8_t dst1 = job->shortAddr[1];
  if (job->kind == JOB_RA) {  // the Router Advertisement wakes every neighbour
    dst0 = 0xFF;
    dst1 = 0xFF;
  }
  uint8_t wake[12] = {0x25, job->data[2], 0xCD, 0xAB, dst0, dst1, 0x82, 0x0E,
                      (uint8_t)rendez, (uint8_t)(rendez >> 8), 0x80, 0x3F};
  return ccSendFrameEx(wake, sizeof(wake), gated, false);
}

static void cslSendData() {
  if (txn.sent == 0 || cslSendWake(&txn.job, 0, false) != SEND_OK) {
    cslCompleteNow(OUT_TXFAIL);
    return;
  }
  // The last wake frame announces rendezvous 0, as the commercial coordinator
  // does, and the Data frame follows after the commercial gap.
  uint64_t dataAt = nowUs() + CSL_DATA_GAP_US;
  while (nowUs() < dataAt) {
  }
  // The receiver comes back right after the Data frame, before the Enhanced ACK.
  bool ok = ccSendFrame(txn.job.data, txn.job.dataLen);
  if (!ok) {
    cslCompleteNow(OUT_TXFAIL);
    return;
  }
  txn.sent += 2;
  if (txn.job.kind == JOB_RA) {
    cslCompleteNow(OUT_OK);
    return;
  }
  rxReset();
  txn.windowEndUs = nowUs() + (uint64_t)txn.job.ackTmoMs * 1000ULL;
  txn.state = CSL_ACK;
}

// The guard follows the parameters of this job, so a legitimate worst case never trips it:
// whole wake window, ACK timeout, follow window and margin.
static void cslSetGuard() {
  uint64_t followUs = (txn.job.flags & CSL_FLAG_FOLLOW) ? (uint64_t)CSL_FOLLOW_TMO_MS * 1000ULL : 0;
  txn.guardUs = txn.wakeEndUs + (uint64_t)txn.job.ackTmoMs * 1000ULL + followUs +
                (uint64_t)CSL_GUARD_MARGIN_MS * 1000ULL;
}

// Take an uplink that lands in the window before the burst. A Router
// Advertisement carries no peer address, so it never answers one.
static bool cslDrainWindow(uint64_t now) {
  if (txn.job.kind == JOB_RA) return false;
  if (!rxDuePump(now) || !rxPump()) return false;
  uint8_t raw[MAX_RX_RAW];
  size_t rawLen = 0;
  uint8_t freqest = 0;
  rxTake(raw, &rawLen, nullptr, &freqest);
  cslStoreInboundForJob(raw, rawLen, &txn.job, freqest);
  return true;
}

static void cslStepRx(uint64_t now) {
  if (now >= txn.wakeEndUs) {
    cslSendData();
    return;
  }
  if (cslDrainWindow(now)) return;
  if (now < (txn.lbtPending ? txn.listenAtUs : txn.startUs)) return;
  txn.state = txn.lbtPending ? CSL_LISTEN : CSL_WAKE;
}

// One listen, or the wait for the backoff of the last one. An uplink that lands meanwhile
// is answered as in the other states. A clear channel goes back to CSL_RX, which holds the
// first frame until the start instant. A job that is busy LBT_TRIES times, or that has no
// window left, ends with ST_BUSY.
static void cslStepListen(uint64_t now) {
  if (now >= txn.wakeEndUs) {
    cslCompleteNow(OUT_BUSY);
    return;
  }
  if (cslDrainWindow(now)) return;
  if (now < txn.listenAtUs) return;
  if (lbtListenOnce()) {
    txn.lbtPending = false;
    txn.gated = true;
    if (!txn.anchored) {
      txn.startUs = nowUs();
      txn.wakeEndUs = txn.startUs + (txn.job.kind == JOB_RA ? RA_PERIOD_US : CSL_BOOTSTRAP_US);
      cslSetGuard();
    }
    txn.state = CSL_RX;
    return;
  }
  if (++txn.lbtTries >= LBT_TRIES) {
    cslCompleteNow(OUT_BUSY);
    return;
  }
  txn.listenAtUs = nowUs() + lbtBackoffUs();
}

static void cslStepWake(uint64_t now) {
  // The wake window closes on time even with an uplink waiting in the buffer:
  // the Data frame has to land on the peer's sample.
  if (now >= txn.wakeEndUs) {
    cslSendData();
    return;
  }
  if (cslDrainWindow(now)) return;
  uint8_t cap = (txn.job.kind == JOB_RA) ? RA_WAKE_CAP : CSL_WAKE_CAP;
  for (uint8_t i = 0; i < CSL_WAKE_BUDGET && txn.sent < cap; i++) {
    int64_t delta = (int64_t)(txn.wakeEndUs - nowUs());
    if (delta < (int64_t)CSL_UNIT_US) break;
    uint32_t rendez = (uint32_t)delta / CSL_UNIT_US;
    if (!txn.anchored) {
      // The first frame of a train without an anchor announces a whole period, the rest count down.
      if (txn.sent == 0) rendez = RA_PERIOD_UNITS;
      else if (rendez > RA_PERIOD_UNITS) rendez = RA_PERIOD_UNITS;
    }
    uint8_t r = cslSendWake(&txn.job, (uint16_t)rendez, txn.gated);
    txn.gated = false;
    if (r != SEND_OK) {
      cslCompleteNow(r == SEND_BUSY ? OUT_BUSY : OUT_TXFAIL);
      return;
    }
    txn.sent++;
  }
  if (nowUs() < txn.wakeEndUs) return;
  cslSendData();
}

static void cslStepAck(uint64_t now) {
  if (now >= txn.windowEndUs) {
    anchorMiss(txn.job.shortAddr);
    cslCompleteNow(txn.sawInvalid ? OUT_BADACK : OUT_NOACK);
    return;
  }
  if (!rxDuePump(now) || !rxPump()) return;
  uint8_t raw[MAX_RX_RAW];
  size_t rawLen = 0;
  uint64_t stampUs = 0;
  uint8_t freqest = 0;
  rxTake(raw, &rawLen, &stampUs, &freqest);
  if (!cslAckValid(raw, rawLen, txn.job.data)) {
    // Only a frame that can be a damaged Enhanced ACK turns NOACK into BADACK:
    // a CRC failure or the Enhanced ACK frame control. An uplink of another
    // node says nothing about this Data frame.
    if (!rxCrcOk(raw, rawLen) || (raw[1] == 0x42 && raw[2] == 0xEE)) txn.sawInvalid = true;
    // No auto-ACK in this window: its TX would cover the Enhanced ACK of the
    // target. The other node retransmits an uplink it sees unacknowledged.
    cslStoreInbound(raw, rawLen, nullptr, 0, freqest);
    return;
  }
  memcpy(txn.ack, raw, rawLen);
  txn.ackLen = (uint8_t)rawLen;
  // A DIO1 edge from before the Data frame is not the edge of this ACK: an
  // anchor placed on it misses by up to a period.
  if (stampUs < txn.windowEndUs - (uint64_t)txn.job.ackTmoMs * 1000ULL) stampUs = nowUs();
  anchorStore(txn.job.shortAddr, raw + 1, stampUs);
  if ((txn.job.flags & CSL_FLAG_FOLLOW) == 0) {
    cslCompleteNow(OUT_OK);
    return;
  }
  txn.windowEndUs = nowUs() + (uint64_t)CSL_FOLLOW_TMO_MS * 1000ULL;
  txn.state = CSL_FOLLOW;
}

static void cslStepFollow(uint64_t now) {
  if (now >= txn.windowEndUs) {
    cslCompleteNow(OUT_OK);
    return;
  }
  if (!rxDuePump(now) || !rxPump()) return;
  uint8_t raw[MAX_RX_RAW];
  size_t rawLen = 0;
  uint8_t freqest = 0;
  rxTake(raw, &rawLen, nullptr, &freqest);
  if (!rxCrcOk(raw, rawLen)) {  // a corrupted frame must not take a follow slot
    if (txn.follow.dropped != 0xFF) txn.follow.dropped++;
    return;
  }
  bool match = cslFollowMatch(raw, rawLen, txn.job.data);
  // Hold one slot for the addressed peer until its own answer arrives.
  bool keep = match ? txn.follow.count < CSL_FOLLOW_MAX
                    : txn.follow.count < (txn.targetStored ? CSL_FOLLOW_MAX : CSL_FOLLOW_MAX - 1);
  if (keep) {
    memcpy(txn.follow.raw[txn.follow.count], raw, rawLen);
    txn.follow.len[txn.follow.count++] = (uint8_t)rawLen;
    if (match) txn.targetStored = true;
    uint8_t coordinator[8];
    coordFromData(txn.job.data, coordinator);
    ccAutoAck(raw, rawLen, coordinator, txn.job.phase);
    return;
  }
  if (txn.follow.dropped != 0xFF) txn.follow.dropped++;
  cslStoreInboundForJob(raw, rawLen, &txn.job, freqest);
}

// Set up the timing of one job and open its receive window. With listen before talk the
// listen starts LBT_LEAD_US before the first frame, so an anchored train starts at the same
// instant as without it. A train without an anchor starts when the listen ends.
static void cslStart(const CslJob *job) {
  memset(&txn, 0, sizeof(txn));
  txn.active = true;
  txn.job = *job;
  uint64_t now = nowUs();
  if (job->kind == JOB_RA) {
    txn.startUs = now;
    txn.wakeEndUs = now + RA_PERIOD_US;
  } else {
    int8_t idx = anchorForJob(job);
    if (idx >= 0) {
      uint32_t guardUs = 0;
      uint64_t sample = anchorNextSample(idx, now, &guardUs);
      txn.anchored = true;
      txn.startUs = sample - guardUs;
      txn.wakeEndUs = sample + guardUs;
    } else {
      txn.startUs = now;
      txn.wakeEndUs = now + CSL_BOOTSTRAP_US;
    }
    ccEnsureRx();
  }
  cslSetGuard();
  txn.lbtPending = lbtOn;
  uint64_t gateUs = txn.startUs;
  if (txn.lbtPending) {
    txn.listenAtUs = lbtListenAt(txn.startUs, now);
    gateUs = txn.listenAtUs;
  }
  txn.state = (now < gateUs) ? CSL_RX : (txn.lbtPending ? CSL_LISTEN : CSL_WAKE);
}

static bool cslCanSchedule() {
  return configured && !txn.active && !exclusiveWaiting && cslCompleteCount < CSL_COMPLETE_DEPTH;
}

// Pick the neighbour whose wake window opens first, round-robin on a tie.
static bool cslScheduleNext() {
  CslNeighbor *best = nullptr;
  uint64_t bestStart = 0;
  uint64_t now = nowUs();
  for (uint8_t step = 0; step < CSL_NEIGHBORS; step++) {
    uint8_t i = (uint8_t)((cslCursor + step) % CSL_NEIGHBORS);
    CslNeighbor *e = &cslNeighbors[i];
    if (!e->used || e->count == 0) continue;
    CslJob *job = &cslJobs[e->head];
    int8_t idx = anchorForJob(job);
    uint64_t start = now;
    if (idx >= 0) {
      uint32_t guardUs = 0;
      start = anchorNextSample(idx, now, &guardUs) - guardUs;
    } else if ((job->flags & CSL_FLAG_BOOTSTRAP) == 0) {
      memset(&txn, 0, sizeof(txn));
      txn.active = true;
      txn.job = *job;
      cslJobRelease(cslNeighborPop(e));
      cslCursor = (uint8_t)((i + 1) % CSL_NEIGHBORS);
      cslCompleteNow(OUT_NOANCHOR);
      return true;
    }
    if (best == nullptr || start < bestStart) {
      best = e;
      bestStart = start;
    }
  }
  if (best == nullptr) return false;
  uint8_t jobIdx = cslNeighborPop(best);
  cslCursor = (uint8_t)(((best - cslNeighbors) + 1) % CSL_NEIGHBORS);
  if (!airAllowed()) {
    memset(&txn, 0, sizeof(txn));
    txn.active = true;
    txn.job = cslJobs[jobIdx];
    cslJobRelease(jobIdx);
    cslCompleteNow(OUT_AIRTIME);
    return true;
  }
  cslStart(&cslJobs[jobIdx]);
  cslJobRelease(jobIdx);
  return true;
}

static void cslStep() {
  if (!txn.active) {
    if (cslCanSchedule()) cslScheduleNext();
    return;
  }
  if (txn.outcomeReady) {  // outcome first, the guard must never rewrite it
    cslDeliver();
    return;
  }
  uint64_t now = nowUs();
  if (now >= txn.guardUs) {
    cslCompleteNow(OUT_TXFAIL);
    return;
  }
  switch (txn.state) {
    case CSL_RX: cslStepRx(now); break;
    case CSL_LISTEN: cslStepListen(now); break;
    case CSL_WAKE: cslStepWake(now); break;
    case CSL_ACK: cslStepAck(now); break;
    default: cslStepFollow(now); break;
  }
}

// ------------------------------------------------------------------- idle drain

// An SX1262 that resets alone (a brown-out) loses the tado PHY. Leave it
// unconfigured until the host loads its register set again.
static void ccWatchReset(uint64_t now) {
  if (now - lastIocfgCheckUs < (uint64_t)RADIO_IDLE_TICK_MS * 1000ULL) return;
  lastIocfgCheckUs = now;
  ccQuietIfReset();
}

// A host that stopped talking leaves no coordinator, no pair window and no job:
// the bridge must not acknowledge or transmit on its behalf.
static void hostWatch() {
  bool lapsed = (uint32_t)(nowMs32() - hostRequestMs.load(std::memory_order_relaxed)) >= HOST_TIMEOUT_MS;
  if (!lapsed) {
    hostLapsed = false;
    return;
  }
  if (hostLapsed) return;
  hostLapsed = true;
  coordKnown = false;
  pairClose();
  cslQueueClear();
}

// Between transactions the chip stays in continuous RX. Take its frames on the DIO1
// interrupt. While the host transmits, answer and store them for its next
// CSLQUEUE poll, otherwise throw them away.
static void idleRxService() {
  uint64_t now = nowUs();
  if (now < hostSpiHoldoffUs) return;  // the host is driving the radio itself
  ccWatchReset(now);
  if (!configured) return;
  if (!rxDuePump(now)) return;
  if (!rxPump()) return;
  uint8_t raw[MAX_RX_RAW];
  size_t rawLen = 0;
  uint8_t freqest = 0;
  rxTake(raw, &rawLen, nullptr, &freqest);
  if (!rxCrcOk(raw, rawLen)) return;
  if (!coordKnown) return;
  pairAnswer(raw, rawLen);
  cslStoreInbound(raw, rawLen, coordEUI, coordPhase, freqest);
}

// ---------------------------------------------------------------- stream window

// Continuous RX for durMs: keep the radio armed and push every complete frame to the
// host as it arrives. The receiver stays in RX between frames, so back-to-back frames
// such as a CSL wake-up burst are all captured.
//
// With matchLen or respCount set it also acts as the auto-responder: a frame
// whose payload matches `match` at matchOff triggers the packed response train
// before the frame is reported. matchOff counts from the first payload byte, the
// same bytes the host sees. matchLen 0 responds to every frame. The match does
// not check the CRC, so a corrupted frame can trigger the train.
static void streamWindow(uint32_t durMs, uint8_t matchOff, const uint8_t *match, uint8_t matchLen,
                         const uint8_t *resp, size_t respLen, uint8_t respCount) {
  uint32_t baseSeq = hostByteSeq.load(std::memory_order_relaxed);
  coordKnown = false;
  ccArmRx();
  rxReset();
  uint64_t now = nowUs();
  uint64_t end = now + (uint64_t)durMs * 1000ULL;
  uint64_t lastTx = now;
  while ((now = nowUs()) < end) {
    esp_task_wdt_reset();
    if (hostByteSeq.load(std::memory_order_relaxed) != baseSeq) break;
    if (now - lastTx >= (uint64_t)STREAM_HEARTBEAT_MS * 1000ULL) {
      sendStreamMarker(STREAM_ALIVE);
      lastTx = nowUs();
      continue;
    }
    if (rxDuePump(now) && rxPump()) {
      uint8_t raw[MAX_RX_RAW];
      size_t rawLen = 0;
      uint8_t freqest = 0;
      rxTake(raw, &rawLen, nullptr, &freqest);
      uint8_t pktLen = raw[0];
      // Respond before reporting: the peer's RX window closes long before a host
      // round-trip could answer. TX drops out of RX, so re-arm afterwards.
      if (respCount && (matchLen == 0 || ((size_t)matchOff + matchLen <= (size_t)pktLen &&
                                          memcmp(raw + 1 + matchOff, match, matchLen) == 0))) {
        if (airAllowed()) ccSendTrain(resp, respLen, respCount, false, nullptr);
        ccArmRx();
        rxReset();
      }
      sendStreamFrame(raw, (uint8_t)rawLen, freqest);
      lastTx = nowUs();
      continue;
    }
    uint64_t left = (end - now) / 1000ULL;
    uint32_t tickMs = rxAssembling() ? RX_ASSEMBLE_TICK_MS : RX_HOUSEKEEP_TICK_MS;
    if (left < tickMs) tickMs = (uint32_t)left;
    radioWait(tickMs);
  }
  ccIdle();
  sendStreamMarker(STREAM_END);
}

// -------------------------------------------------------------- CSLTX requests

// Shape of one CSLTX record: ackTmoHi ackTmoLo flags dst0 dst1 [phaseLo phaseHi]
// followed by the Data frame. The phase pair is present only with the follow flag.
static bool cslParseTx(const uint8_t *tx, size_t txLen, CslJob *job, uint16_t recordPhase) {
  if (txLen < 21) return false;
  uint16_t ackTmoMs = ((uint16_t)tx[0] << 8) | tx[1];
  uint8_t flags = tx[2];
  bool waitFollow = (flags & CSL_FLAG_FOLLOW) != 0;
  size_t off = waitFollow ? 7 : 5;
  if ((flags & ~(CSL_FLAG_BOOTSTRAP | CSL_FLAG_FOLLOW)) != 0) return false;
  if (txLen < off + 16) return false;
  const uint8_t *data = tx + off;
  size_t dataLen = txLen - off;
  if (ackTmoMs == 0 || ackTmoMs > CSL_ACK_TMO_MAX_MS || dataLen > MAX_TX_FRAME) return false;
  if (!((data[0] == 0x69 || data[0] == 0x79) && data[1] == 0xEC)) return false;
  if (tx[3] != data[4] || tx[4] != data[3]) return false;
  job->ackTmoMs = ackTmoMs;
  job->flags = flags;
  job->shortAddr[0] = tx[3];
  job->shortAddr[1] = tx[4];
  job->phase = recordPhase;
  job->dataLen = (uint8_t)dataLen;
  memcpy(job->data, data, dataLen);
  return true;
}

static void cmdCSLTx(const uint8_t *p, uint16_t len) {
  uint8_t sent = 0;
  CslJob job;
  memset(&job, 0, sizeof(job));
  job.kind = JOB_SYNC_CSL;
  // Only a follow request carries the coordinator phase, otherwise it stays 0.
  uint16_t phase = 0;
  if (len >= 7 && (p[2] & CSL_FLAG_FOLLOW)) phase = (uint16_t)p[5] | ((uint16_t)p[6] << 8);
  if (!cslParseTx(p, len, &job, phase)) {
    sendResp(ST_INPUT, &sent, 1);
    return;
  }
  uint8_t coordinator[8];
  coordFromData(job.data, coordinator);
  coordRemember(coordinator, job.phase);
  if (anchorIndex(job.shortAddr) < 0 && (job.flags & CSL_FLAG_BOOTSTRAP) == 0) {
    sendResp(ST_NOANCHOR, &sent, 1);
    return;
  }
  if (!airAllowed()) {
    sendResp(ST_AIRTIME, &sent, 1);
    return;
  }
  cslStart(&job);
}

// A pair window sends the advertisement twice, the first copy with Frame
// Pending (0x59E8).
static void cmdRABurst(const uint8_t *p, uint16_t len) {
  uint8_t sent = 0;
  if (len < 3 || len > MAX_TX_FRAME || (p[0] != 0x49 && p[0] != 0x59) || p[1] != 0xE8) {
    sendResp(ST_INPUT, &sent, 1);
    return;
  }
  if (!airAllowed()) {
    sendResp(ST_AIRTIME, &sent, 1);
    return;
  }
  CslJob job;
  memset(&job, 0, sizeof(job));
  job.kind = JOB_RA;
  job.dataLen = (uint8_t)len;
  memcpy(job.data, p, len);
  cslStart(&job);
}

// Request: durHi durLo seconds, then for a window that opens key[16]
// coordinator[8] seq counter[4 LE] count. A zero duration closes the window.
// The reply carries the beacons sent in the window the request ends.
static void cmdPairWin(const uint8_t *p, uint16_t len) {
  uint8_t sent = pairWin.sent;
  uint16_t durS = (len >= 2) ? (uint16_t)(((uint16_t)p[0] << 8) | p[1]) : 0;
  if (len == 2 && durS == 0) {
    pairClose();
    sendResp(ST_OK, &sent, 1);
    return;
  }
  if (len != PAIRWIN_OPEN_LEN || durS == 0 || durS > PAIRWIN_MAX_S || p[31] == 0) {
    sendResp(ST_INPUT, &sent, 1);
    return;
  }
  pairClose();
  mbedtls_aes_setkey_enc(&pairWin.aes, p + 2, 128);
  memcpy(pairWin.coord, p + 18, 8);
  pairWin.seq = p[26];
  pairWin.ctr = (uint32_t)p[27] | ((uint32_t)p[28] << 8) | ((uint32_t)p[29] << 16) | ((uint32_t)p[30] << 24);
  pairWin.left = p[31];
  pairWin.endUs = nowUs() + (uint64_t)durS * 1000000ULL;
  pairWin.active = true;
  sendResp(ST_OK, &sent, 1);
}

// -------------------------------------------------------------- CSLQUEUE command

static void cmdQueueEnqueue(const uint8_t *p, uint16_t len) {
  if (len < 2 || p[1] == 0 || p[1] > CSL_NEIGHBOR_DEPTH) {
    sendResp(ST_INPUT, nullptr, 0);
    return;
  }
  uint8_t count = p[1];
  size_t pos = 2;
  uint8_t shortAddr[2] = {0, 0};
  CslJob probe;
  for (uint8_t i = 0; i < count; i++) {
    if (pos + 6 > len) {
      sendResp(ST_INPUT, nullptr, 0);
      return;
    }
    size_t txLen = ((size_t)p[pos + 2] << 8) | p[pos + 3];
    if (pos + 6 + txLen > len) {
      sendResp(ST_INPUT, nullptr, 0);
      return;
    }
    uint16_t recordPhase = (uint16_t)p[pos + 4] | ((uint16_t)p[pos + 5] << 8);
    memset(&probe, 0, sizeof(probe));
    if (!cslParseTx(p + pos + 6, txLen, &probe, recordPhase)) {
      sendResp(ST_INPUT, nullptr, 0);
      return;
    }
    if (i == 0) {
      shortAddr[0] = probe.shortAddr[0];
      shortAddr[1] = probe.shortAddr[1];
    } else if (shortAddr[0] != probe.shortAddr[0] || shortAddr[1] != probe.shortAddr[1]) {
      sendResp(ST_INPUT, nullptr, 0);
      return;
    }
    pos += 6 + txLen;
  }
  if (pos != len) {
    sendResp(ST_INPUT, nullptr, 0);
    return;
  }
  if (!airAllowed()) {
    sendResp(ST_AIRTIME, nullptr, 0);
    return;
  }
  CslNeighbor *entry = cslNeighborFind(shortAddr, false);
  if (entry == nullptr) entry = cslNeighborFind(shortAddr, true);
  if (entry == nullptr) {
    sendResp(ST_NEIGHBOR_FULL, nullptr, 0);
    return;
  }
  if ((uint16_t)entry->count + count > CSL_NEIGHBOR_DEPTH || cslJobFreeCount < count) {
    if (entry->count == 0) entry->used = false;
    sendResp(ST_QUEUE_FULL, nullptr, 0);
    return;
  }
  pos = 2;
  for (uint8_t i = 0; i < count; i++) {
    size_t txLen = ((size_t)p[pos + 2] << 8) | p[pos + 3];
    uint16_t recordPhase = (uint16_t)p[pos + 4] | ((uint16_t)p[pos + 5] << 8);
    uint8_t idx = cslJobAlloc();
    CslJob *job = &cslJobs[idx];
    memset(job, 0, sizeof(*job));
    job->kind = JOB_QUEUED;
    job->id = ((uint16_t)p[pos] << 8) | p[pos + 1];
    cslParseTx(p + pos + 6, txLen, job, recordPhase);
    cslNeighborPush(entry, idx);
    pos += 6 + txLen;
  }
  uint8_t coordinator[8];
  coordFromData(cslJobs[entry->tail].data, coordinator);
  coordRemember(coordinator, cslJobs[entry->tail].phase);
  sendResp(ST_OK, &count, 1);
}

static void cmdQueuePoll() {
  if (cslCompleteCount != 0) {
    CslCompletion *done = &cslComplete[cslCompleteHead];
    size_t out = 0;
    respBuf[out++] = 0;
    respBuf[out++] = (uint8_t)(done->id >> 8);
    respBuf[out++] = (uint8_t)done->id;
    respBuf[out++] = done->flags;
    respBuf[out++] = done->sent;
    if (done->status == ST_OK) {
      if ((done->flags & CSL_FLAG_FOLLOW) == 0) {
        memcpy(respBuf + out, done->ack, done->ackLen);
        out += done->ackLen;
      } else {
        out = cslAppendFollowBody(respBuf, out, done->ack, done->ackLen, &done->follow);
      }
    }
    uint8_t status = done->status;
    cslCompleteHead = (uint8_t)((cslCompleteHead + 1) % CSL_COMPLETE_DEPTH);
    cslCompleteCount--;
    sendResp(status, respBuf, out);
    return;
  }
  if (cslInboundCount != 0) {
    CslInbound *inbound = &cslInbound[cslInboundHead];
    respBuf[0] = 1;
    memcpy(respBuf + 1, inbound->raw, inbound->len);
    uint8_t n = inbound->len;
    respBuf[1 + n] = inbound->freqest;
    cslInboundHead = (uint8_t)((cslInboundHead + 1) % CSL_INBOUND_DEPTH);
    cslInboundCount--;
    sendResp(ST_OK, respBuf, (size_t)n + 2);
    return;
  }
  sendResp(ST_OK, nullptr, 0);
}

static void cmdQueue(const uint8_t *p, uint16_t len) {
  if (len < 1) {
    sendResp(ST_INPUT, nullptr, 0);
    return;
  }
  if (p[0] == CSLQ_ENQUEUE) {
    cmdQueueEnqueue(p, len);
    return;
  }
  if (p[0] == CSLQ_POLL) {
    if (len != 1) sendResp(ST_INPUT, nullptr, 0);
    else cmdQueuePoll();
    return;
  }
  if (p[0] == CSLQ_FLUSH) {
    if (len != 1) {
      sendResp(ST_INPUT, nullptr, 0);
      return;
    }
    cslQueueClear();
    coordKnown = false;
    pairClose();
    if (!txn.active) ccEnsureRx();  // a job that still owes a reply keeps the receiver
    sendResp(ST_OK, nullptr, 0);
    return;
  }
  sendResp(ST_INPUT, nullptr, 0);
}

// ------------------------------------------------------------ command execution

// Only PING, GDO and CSLQUEUE run while a transaction is in flight. Everything
// else waits for the state machine, so no host strobe lands mid transaction.
static bool cmdNeedsRadio(uint8_t cmd) {
  return cmd != CMD_PING && cmd != CMD_GDO && cmd != CMD_CSLQUEUE && cmd != CMD_CONFIGURED &&
         cmd != CMD_AIRTIME && cmd != CMD_LBT;
}

// PING, XFER, GDO, RESET, STROBE, CONFIGURED, AIRTIME and LBT run on a chip with
// reset values. An unknown command stays BAD_CMD.
static bool cmdNeedsConfigured(uint8_t cmd) {
  switch (cmd) {
    case CMD_RECV:
    case CMD_STREAM:
    case CMD_TXTRAIN:
    case CMD_RESPOND:
    case CMD_AUTOACK:
    case CMD_RABURST:
    case CMD_CSLTX:
    case CMD_CSLQUEUE:
    case CMD_PAIRWIN: return true;
    default: return false;
  }
}

static void execCommand(CmdMsg *msg) {
  uint8_t *p = msg->payload;
  uint16_t len = msg->len;
  if (!configured && cmdNeedsConfigured(msg->cmd)) {
    sendResp(ST_UNCONFIGURED, nullptr, 0);
    return;
  }
  switch (msg->cmd) {
    case CMD_PING: {
      const char *s = BANNER;
      sendResp(ST_OK, (const uint8_t *)s, strlen(s));
      break;
    }
    case CMD_XFER: {
      hostSpiHoldoffUs = nowUs() + (uint64_t)HOST_SPI_HOLDOFF_MS * 1000ULL;
      rxValidUs = nowUs() + LBT_SETTLE_US;  // the host may have strobed SRX
      ccEmulXfer(p, len);
      sendResp(ST_OK, p, len);
      break;
    }
    case CMD_STROBE: {
      if (len < 1) {
        sendResp(ST_BAD_ARGS, nullptr, 0);
        break;
      }
      hostSpiHoldoffUs = nowUs() + (uint64_t)HOST_SPI_HOLDOFF_MS * 1000ULL;
      rxValidUs = nowUs() + LBT_SETTLE_US;
      ccEmulXfer(p, 1);
      sendResp(ST_OK, p, 1);
      break;
    }
    case CMD_GDO: {
      // DIO1 (RxDone) takes the place of GDO0; there is no GDO2.
      uint8_t g = digitalRead(PIN_SX_DIO1) ? 1 : 0;
      sendResp(ST_OK, &g, 1);
      break;
    }
    case CMD_RESET: {
      hostSpiHoldoffUs = nowUs() + (uint64_t)HOST_SPI_HOLDOFF_MS * 1000ULL;
      coordKnown = false;
      pairClose();
      // The host loads its register set next. A job a crashed session left in
      // the queue would transmit on a chip that is half configured.
      cslQueueClear();
      manualReset();
      rxReset();
      lbtOn = false;
      lbtThresholdDbm = LBT_THRESHOLD_DEFAULT_DBM;
      airBudgetMs = 0;
      sendResp(ST_OK, nullptr, 0);
      break;
    }
    case CMD_RECV: {
      uint32_t tmo = (len >= 2) ? (((uint32_t)p[0] << 8) | p[1]) : 2000;
      coordKnown = false;
      ccEnsureRx();
      uint8_t pkt[MAX_RX_RAW + 1];
      size_t plen = 0;
      uint8_t freqest = 0;
      bool got = rxWaitFrame(tmo, pkt, &plen, nullptr, &freqest);
      // The radio stays in continuous RX and the idle drain takes over any
      // frame that already started.
      if (!got) {
        sendResp(ST_NO_FRAME, nullptr, 0);
        break;
      }
      pkt[plen++] = freqest;
      sendResp(ST_OK, pkt, plen);
      break;
    }
    case CMD_AUTOACK: {
      if (len != 12) {
        sendResp(ST_BAD_ARGS, nullptr, 0);
        break;
      }
      uint32_t tmo = ((uint32_t)p[0] << 8) | p[1];
      uint16_t phase = (uint16_t)p[10] | ((uint16_t)p[11] << 8);
      coordRemember(p + 2, phase);
      ccEnsureRx();
      uint8_t pkt[MAX_RX_RAW + 1];
      size_t plen = 0;
      uint8_t freqest = 0;
      bool got = rxWaitFrame(tmo, pkt, &plen, nullptr, &freqest);
      // The radio stays in RX: continuous RX after a frame, RX again after
      // the Enhanced ACK, and a failed ACK re-arms RX. Until the next host
      // window the idle drain answers and stores the uplinks addressed to the
      // coordinator this request set, a following fragment included.
      if (!got) {
        sendResp(ST_NO_FRAME, nullptr, 0);
        break;
      }
      uint8_t outcome = ccAutoAck(pkt, plen, p + 2, phase);
      if (outcome == AUTOACK_NONE) pairAnswer(pkt, plen);
      pkt[plen++] = freqest;
      if (outcome == AUTOACK_SENT) sendResp(ST_OK, pkt, plen);
      else if (outcome == AUTOACK_FAILED) sendResp(ST_ACK_FAILED, pkt, plen);
      else sendResp(ST_ACK_SKIPPED, pkt, plen);
      break;
    }
    case CMD_STREAM: {
      // Streams frames directly (5A F1 ...) then an end marker (5A F2 00), with
      // no response header. The host reads this window with its stream reader.
      uint32_t dur = (len >= 2) ? (((uint32_t)p[0] << 8) | p[1]) : STREAM_DEFAULT_MS;
      streamWindow(dur, 0, nullptr, 0, nullptr, 0, 0);
      break;
    }
    case CMD_RESPOND: {
      // Request: durHi durLo matchOff matchLen match[matchLen] respCount, then
      // respCount sub-frames [len:1][bytes:len]. Output is the CMD_STREAM stream.
      if (len < 5) {
        sendResp(ST_BAD_ARGS, nullptr, 0);
        break;
      }
      uint32_t dur = ((uint32_t)p[0] << 8) | p[1];
      uint8_t matchOff = p[2];
      uint8_t matchLen = p[3];
      if (matchLen > MAX_MATCH || (size_t)5 + matchLen > len) {
        sendResp(ST_BAD_ARGS, nullptr, 0);
        break;
      }
      uint8_t respCount = p[4 + matchLen];
      if (respCount > MAX_TRAIN) {
        sendResp(ST_BAD_ARGS, nullptr, 0);
        break;
      }
      if (respCount != 0 && !airAllowed()) {
        sendResp(ST_AIRTIME, nullptr, 0);
        break;
      }
      streamWindow(dur, matchOff, p + 4, matchLen, p + 5 + matchLen,
                   len - (5 + (size_t)matchLen), respCount);
      break;
    }
    case CMD_TXTRAIN: {
      // Request: count[1] then count sub-frames [len:1][bytes:len]. Each one is
      // a complete on-air frame. Response payload is the frames actually sent.
      if (len < 1) {
        sendResp(ST_BAD_ARGS, nullptr, 0);
        break;
      }
      uint8_t sent = 0;
      if (!airAllowed()) {
        sendResp(ST_AIRTIME, &sent, 1);
        break;
      }
      // CMD_RESPOND answers a frame it just received, within the window of the peer, and
      // takes no listen.
      if (lbtOn && !lbtClear(0)) {
        sendResp(ST_BUSY, &sent, 1);
        break;
      }
      bool blocked = false;
      sent = ccSendTrain(p + 1, len - 1, p[0], lbtOn, &blocked);
      sendResp(blocked ? ST_BUSY : ST_OK, &sent, 1);
      break;
    }
    case CMD_RABURST: cmdRABurst(p, len); break;
    case CMD_CSLTX: cmdCSLTx(p, len); break;
    case CMD_CSLQUEUE: cmdQueue(p, len); break;
    case CMD_PAIRWIN: cmdPairWin(p, len); break;
    case CMD_CONFIGURED: {
      if (len != 0) {
        sendResp(ST_BAD_ARGS, nullptr, 0);
        break;
      }
      configured = true;
      sendResp(ST_OK, nullptr, 0);
      break;
    }
    case CMD_LBT: {
      if (len != 2) {
        sendResp(ST_BAD_LEN, nullptr, 0);
        break;
      }
      if (p[0] > 1) {
        sendResp(ST_BAD_ARGS, nullptr, 0);
        break;
      }
      lbtOn = p[0] != 0;
      lbtThresholdDbm = (int8_t)p[1];
      sendResp(ST_OK, p, 2);
      break;
    }
    case CMD_AIRTIME: {
      // 122 bytes load the buckets. 126 bytes load them and set the budget in ms, uint32 big endian.
      if (len != 0 && len != 2 * AIR_BUCKETS && len != 2 * AIR_BUCKETS + 4) {
        sendResp(ST_BAD_LEN, nullptr, 0);
        break;
      }
      if (len != 0) airPreload(p);
      if (len == 2 * AIR_BUCKETS + 4) {
        const uint8_t *b = p + 2 * AIR_BUCKETS;
        airBudgetMs = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
      }
      uint32_t ms = airMs();
      uint8_t out[8 + 2 * AIR_BUCKETS] = {(uint8_t)(ms >> 24), (uint8_t)(ms >> 16), (uint8_t)(ms >> 8),
                                         (uint8_t)ms, (uint8_t)(airBudgetMs >> 24),
                                         (uint8_t)(airBudgetMs >> 16), (uint8_t)(airBudgetMs >> 8),
                                         (uint8_t)airBudgetMs};
      for (uint8_t k = 0; k < AIR_BUCKETS; k++) {
        uint8_t age = AIR_BUCKETS - 1 - k;
        uint16_t b = airBucket[(airMinute % AIR_BUCKETS + AIR_BUCKETS - age) % AIR_BUCKETS] / 1000U;
        out[8 + 2 * k] = (uint8_t)(b >> 8);
        out[9 + 2 * k] = (uint8_t)b;
      }
      sendResp(ST_OK, out, sizeof(out));
      break;
    }
    default: sendResp(ST_BAD_CMD, nullptr, 0); break;
  }
}

// ------------------------------------------------------------------- radio task

static void radioWait(uint32_t ms) {
  uint32_t bits = 0;
  xTaskNotifyWait(0, NOTIFY_ALL_BITS, &bits, pdMS_TO_TICKS(ms));
}

static uint32_t radioWaitMs() {
  if (txn.active) {
    // The outcome is decided but the completion ring is full: wait for a poll.
    if (txn.outcomeReady) return RADIO_IDLE_TICK_MS;
    if (txn.state == CSL_WAKE) return 0;
    uint64_t next = txn.windowEndUs;
    if (txn.state == CSL_RX) next = txn.lbtPending ? txn.listenAtUs : txn.startUs;
    else if (txn.state == CSL_LISTEN) next = txn.listenAtUs;
    uint64_t now = nowUs();
    uint32_t cap = RADIO_IDLE_TICK_MS;
    if (next > now) {
      uint64_t left = (next - now) / 1000ULL;
      if (left < cap) cap = (uint32_t)left;
    } else {
      cap = 0;
    }
    if (rxAssembling() && cap > RX_ASSEMBLE_TICK_MS) cap = RX_ASSEMBLE_TICK_MS;
    return cap;
  }
  if (cslCanSchedule()) {
    for (uint8_t i = 0; i < CSL_NEIGHBORS; i++) {
      if (cslNeighbors[i].used && cslNeighbors[i].count != 0) return 0;
    }
  }
  return rxAssembling() ? RX_ASSEMBLE_TICK_MS : RADIO_IDLE_TICK_MS;
}

static void radioTask(void *) {
  esp_task_wdt_add(nullptr);
  rxReset();  // the chip stays where the reset left it until the host sets it up
  for (;;) {
    esp_task_wdt_reset();
    CmdMsg *msg = nullptr;
    bool have = xQueuePeek(qCmd, &msg, 0) == pdTRUE;
    exclusiveWaiting = have && cmdNeedsRadio(msg->cmd);
    hostWatch();
    cslStep();
    if (have && (!exclusiveWaiting || !txn.active)) {
      xQueueReceive(qCmd, &msg, 0);
      exclusiveWaiting = false;
      execCommand(msg);
      hostRequestMs.store(nowMs32(), std::memory_order_relaxed);
      xQueueSend(qCmdFree, &msg, portMAX_DELAY);
      continue;
    }
    if (!txn.active) idleRxService();
    radioWait(radioWaitMs());
  }
}

// ------------------------------------------------------------------ serial task

// A host that stops reading fills the 2048-byte TX buffer, and a blocking
// Serial.write there would let the 2 s task watchdog reboot the bridge.
static void serialWriteAll(const uint8_t *p, size_t n) {
  size_t off = 0;
  while (off < n) {
    esp_task_wdt_reset();
    int room = Serial.availableForWrite();
    if (room <= 0) {
      vTaskDelay(1);
      continue;
    }
    size_t chunk = n - off;
    if (chunk > (size_t)room) chunk = (size_t)room;
    Serial.write(p + off, chunk);
    off += chunk;
  }
}

static void serialDrainOut() {
  OutMsg *m = nullptr;
  while (xQueueReceive(qOut, &m, 0) == pdTRUE) {
    serialWriteAll(m->data, m->len);
    xQueueSend(qOutFree, &m, portMAX_DELAY);
  }
}

// Parser errors answer from the serial task itself. Taking an out buffer here
// would deadlock: the serial task is the only consumer that frees one.
static void serialReject(uint8_t status) {
  serialDrainOut();
  uint8_t hdr[4] = {RSP_MAGIC, status, 0x00, 0x00};
  serialWriteAll(hdr, 4);
}

// Read one byte, servicing the outbound queue while the host is quiet. Returns
// -1 once the deadline passes.
static int serialReadByte(uint64_t deadlineUs) {
  static uint64_t lastNotifyUs = 0;
  for (;;) {
    int c = Serial.read();
    if (c >= 0) {
      hostByteSeq.fetch_add(1, std::memory_order_relaxed);
      // The radio task only needs to know the host spoke, so one wake per
      // millisecond is enough and a 1 kByte payload does not flood it.
      uint64_t t = nowUs();
      if (radioTaskHandle != nullptr && t - lastNotifyUs >= 1000) {
        lastNotifyUs = t;
        xTaskNotify(radioTaskHandle, NOTIFY_RADIO, eSetBits);
      }
      return c;
    }
    serialDrainOut();
    if (nowUs() >= deadlineUs) return -1;
    esp_task_wdt_reset();
    OutMsg *m = nullptr;
    if (xQueueReceive(qOut, &m, 1) == pdTRUE) {
      serialWriteAll(m->data, m->len);
      xQueueSend(qOutFree, &m, portMAX_DELAY);
    }
  }
}

// Read n bytes with an inter-byte timeout and a total budget for the request.
static bool serialReadN(uint8_t *p, size_t n, uint64_t budgetUs) {
  uint64_t total = nowUs() + budgetUs;
  for (size_t i = 0; i < n; i++) {
    uint64_t byteDeadline = nowUs() + (uint64_t)SERIAL_INTERBYTE_MS * 1000ULL;
    if (byteDeadline > total) byteDeadline = total;
    int c = serialReadByte(byteDeadline);
    if (c < 0) return false;
    p[i] = (uint8_t)c;
  }
  return true;
}

static CmdMsg *cmdAcquire() {
  CmdMsg *m = nullptr;
  while (xQueueReceive(qCmdFree, &m, 0) != pdTRUE) {
    serialDrainOut();
    esp_task_wdt_reset();
    vTaskDelay(1);
  }
  return m;
}

static void serialTask(void *) {
  esp_task_wdt_add(nullptr);
  for (;;) {
    esp_task_wdt_reset();
    serialDrainOut();
    int c = serialReadByte(nowUs() + 10000ULL);
    if (c < 0) continue;
    if ((uint8_t)c != REQ_MAGIC) continue;  // resync: drop stray bytes until magic

    uint8_t head[3];
    if (!serialReadN(head, 3, (uint64_t)SERIAL_REQUEST_MS * 1000ULL)) continue;
    uint8_t crc = crc8(0, head, sizeof(head));
    size_t len = ((size_t)head[1] << 8) | head[2];
    if (len > MAX_PAYLOAD) {
      serialReject(ST_BAD_LEN);
      continue;
    }
    CmdMsg *msg = cmdAcquire();
    msg->cmd = head[0];
    msg->len = (uint16_t)len;
    if (len && !serialReadN(msg->payload, len, (uint64_t)SERIAL_REQUEST_MS * 1000ULL)) {
      xQueueSend(qCmdFree, &msg, portMAX_DELAY);
      serialReject(ST_BAD_PAYLOAD);
      continue;
    }
    crc = crc8(crc, msg->payload, len);
    uint8_t wantCrc = 0;
    if (!serialReadN(&wantCrc, 1, (uint64_t)SERIAL_REQUEST_MS * 1000ULL)) {
      xQueueSend(qCmdFree, &msg, portMAX_DELAY);
      serialReject(ST_BAD_PAYLOAD);
      continue;
    }
    if (wantCrc != crc) {
      xQueueSend(qCmdFree, &msg, portMAX_DELAY);
      serialReject(ST_BAD_CRC);
      continue;
    }
    hostRequestMs.store(nowMs32(), std::memory_order_relaxed);
    xQueueSend(qCmd, &msg, portMAX_DELAY);
    if (radioTaskHandle != nullptr) xTaskNotify(radioTaskHandle, NOTIFY_RADIO, eSetBits);
  }
}

// ------------------------------------------------------------------------ setup

static void queuesInit() {
  qCmd = xQueueCreate(CMD_POOL, sizeof(CmdMsg *));
  qCmdFree = xQueueCreate(CMD_POOL, sizeof(CmdMsg *));
  qOut = xQueueCreate(OUT_POOL, sizeof(OutMsg *));
  qOutFree = xQueueCreate(OUT_POOL, sizeof(OutMsg *));
  for (uint8_t i = 0; i < CMD_POOL; i++) {
    CmdMsg *m = &cmdPool[i];
    xQueueSend(qCmdFree, &m, 0);
  }
  for (uint8_t i = 0; i < OUT_POOL; i++) {
    OutMsg *m = &outPool[i];
    xQueueSend(qOutFree, &m, 0);
  }
}

static void wdtInit() {
  // idle_core_mask 0: the radio task busy-waits on core 1 during a transaction,
  // so only the two sketch tasks are watched.
  esp_task_wdt_config_t cfg = {};
  cfg.timeout_ms = WDT_TIMEOUT_MS;
  cfg.idle_core_mask = 0;
  cfg.trigger_panic = true;
  if (esp_task_wdt_reconfigure(&cfg) != ESP_OK) esp_task_wdt_init(&cfg);
}

void setup() {
  Serial.setRxBufferSize(SERIAL_RX_BUF);
  Serial.setTxBufferSize(SERIAL_TX_BUF);
  // With no host on the USB port a write must not wait: serialWriteAll waits for room itself.
  Serial.setTxTimeoutMs(0);
  Serial.begin(SERIAL_BAUD);

  sxBegin();
  manualReset();

  cslPoolInit();
  cslQueueClear();
  rxReset();
  mbedtls_aes_init(&pairWin.aes);
  wdtInit();
  queuesInit();

  xTaskCreatePinnedToCore(radioTask, "radio", RADIO_STACK, nullptr, RADIO_PRIO, &radioTaskHandle,
                          RADIO_CORE);
  xTaskCreatePinnedToCore(serialTask, "serial", SERIAL_STACK, nullptr, SERIAL_PRIO, nullptr,
                          SERIAL_CORE);

  attachInterrupt(digitalPinToInterrupt(PIN_SX_DIO1), dio1Isr, RISING);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
