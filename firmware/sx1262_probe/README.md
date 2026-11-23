# freedo SX1262 probe

Empfangstest: Hört ein SX1262 (diymore/Heltec ESP32 LoRa V4) das 868-MHz-Funknetz von tado V3+?
Der Sketch sendet nie etwas. Er zeigt nur an, was er empfängt. Die Funkparameter stammen aus
`tadoRegs` in freedo `cc1101/cc1101.go`.

## Bauen (Homelab, im Container)

    ./build.sh

Ergebnis: `build/sx1262_probe.ino.merged.bin` (Bootloader, Partitionen und App in einer Datei,
Offset 0x0). Werkzeuge: arduino-cli 1.6.0, ESP32-Core 3.3.11 (wie bei der freedo-Bridge-Firmware)
und RadioLib 7.8.1.

## Flashen (nur am PC)

    scp <homelab>:<checkout>/firmware/sx1262_probe/build/sx1262_probe.ino.merged.bin .
    esptool --chip esp32s3 --port <PORT> write-flash 0x0 sx1262_probe.ino.merged.bin

Ohne esptool geht es im Browser mit https://espressif.github.io/esptool-js/ (Chrome/Edge,
Adresse 0x0). Findet esptool das Board nicht: PRG gedrückt halten, RST kurz drücken, PRG loslassen.

## Ausgabe

Serieller Monitor auf dem USB-Port, 115200 Baud. Die Ausgabe geht zusätzlich auf UART0.

- `# ...` sind Statusmeldungen. Alle 60 s kommen Konfiguration und Zähler.
- Ein empfangenes Paket: `<Sekunden> [CRC-OK rssi=-71 mode=HW] <Hex>`. Das Format entspricht
  `freedo-dev rx`, freedos Werkzeuge können es also lesen.
- PRG schaltet zwischen `HW` (der SX1262 prüft die CRC) und `RAW` (der Sketch prüft sie).
- `beginFSK failed: -2`: Auf dem SPI-Bus antwortet kein SX1262, die Pinbelegung des Boards weicht ab.

Ein Mitschnitt enthält die Adressen der eigenen Geräte. So nicht öffentlich posten,
vorher unkenntlich machen (siehe freedo `CONTRIBUTING.md`).
