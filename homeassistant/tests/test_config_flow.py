"""Home Assistant tests of the config flow: the discovery step, which the
freedo add-on feeds through the Supervisor, the user step and the reauth.

They run only with pytest-homeassistant-custom-component installed.
"""

from unittest.mock import patch

import pytest

pytest.importorskip("pytest_homeassistant_custom_component")

from homeassistant import config_entries  # noqa: E402
from homeassistant.const import CONF_HOST, CONF_PORT, CONF_SSL  # noqa: E402
from homeassistant.data_entry_flow import FlowResultType  # noqa: E402
from homeassistant.helpers.service_info.hassio import HassioServiceInfo  # noqa: E402
from pytest_homeassistant_custom_component.common import MockConfigEntry  # noqa: E402

from custom_components.freedo.api import FreedoAuthError, FreedoConnectionError  # noqa: E402
from test_ha import auto_enable_custom_integrations  # noqa: E402,F401

DOMAIN = "freedo"
HOST = "a1b2c3d4-freedo"
PORT = 18080
TOKEN = "synthetic-token"
DATA = {CONF_HOST: HOST, CONF_PORT: PORT, "token": TOKEN}


def discovery(config=None) -> HassioServiceInfo:
    return HassioServiceInfo(
        config=config or DATA,
        name="freedo",
        slug="a1b2c3d4_freedo",
        uuid="0123456789abcdef0123456789abcdef",
    )


class DaemonClient:
    """The client of the config flow: home() answers or raises."""

    error: Exception | None = None
    # (host, port, token, ssl) of each client the flow made.
    made: list[tuple] = []

    def __init__(self, session, host, port, token, ssl=False) -> None:
        self.made.append((host, port, token, ssl))

    async def home(self):
        if self.error:
            raise self.error
        return {}


async def start(hass):
    return await hass.config_entries.flow.async_init(
        DOMAIN, context={"source": config_entries.SOURCE_HASSIO}, data=discovery()
    )


async def test_discovery_creates_the_entry(hass):
    result = await start(hass)
    assert result["type"] is FlowResultType.FORM
    assert result["step_id"] == "hassio_confirm"
    with (
        patch("custom_components.freedo.config_flow.FreedoClient", DaemonClient),
        patch("custom_components.freedo.async_setup_entry", return_value=True),
    ):
        result = await hass.config_entries.flow.async_configure(result["flow_id"], {})
    assert result["type"] is FlowResultType.CREATE_ENTRY
    assert result["data"] == DATA
    assert result["result"].unique_id == f"{HOST}:{PORT}"


@pytest.mark.parametrize(
    ("error", "key"),
    [
        (FreedoConnectionError("down"), "cannot_connect"),
        (FreedoAuthError("401"), "invalid_auth"),
    ],
)
async def test_discovery_shows_the_error_and_lets_the_user_retry(hass, error, key):
    result = await start(hass)
    DaemonClient.error = error
    try:
        with patch("custom_components.freedo.config_flow.FreedoClient", DaemonClient):
            result = await hass.config_entries.flow.async_configure(result["flow_id"], {})
    finally:
        DaemonClient.error = None
    assert result["type"] is FlowResultType.FORM
    assert result["errors"] == {"base": key}


async def test_discovery_updates_the_token_of_a_known_add_on(hass):
    entry = MockConfigEntry(
        domain=DOMAIN,
        data={CONF_HOST: HOST, CONF_PORT: PORT, "token": "old-token"},
        unique_id=f"{HOST}:{PORT}",
    )
    entry.add_to_hass(hass)
    with patch("custom_components.freedo.async_setup_entry", return_value=True):
        result = await start(hass)
        await hass.async_block_till_done()
    assert result["type"] is FlowResultType.ABORT
    assert result["reason"] == "already_configured"
    assert entry.data["token"] == TOKEN


@pytest.mark.parametrize(
    ("known", "started"),
    [(HOST, f"{HOST}-beta"), (f"{HOST}-beta", HOST)],
)
async def test_discovery_moves_the_entry_between_the_stable_and_the_beta_add_on(
    hass, known, started
):
    entry = MockConfigEntry(
        domain=DOMAIN,
        data={CONF_HOST: known, CONF_PORT: PORT, "token": "old-token"},
        unique_id=f"{HOST}:{PORT}",
    )
    entry.add_to_hass(hass)
    config = {CONF_HOST: started, CONF_PORT: PORT, "token": TOKEN, "started": "1"}
    with patch("custom_components.freedo.async_setup_entry", return_value=True):
        result = await hass.config_entries.flow.async_init(
            DOMAIN,
            context={"source": config_entries.SOURCE_HASSIO},
            data=discovery(config),
        )
        await hass.async_block_till_done()
    assert result["type"] is FlowResultType.ABORT
    assert result["reason"] == "already_configured"
    assert entry.data == {CONF_HOST: started, CONF_PORT: PORT, "token": TOKEN}
    assert len(hass.config_entries.async_entries(DOMAIN)) == 1


async def test_discovery_without_a_token_sends_an_empty_one(hass):
    config = {CONF_HOST: HOST, CONF_PORT: str(PORT)}
    result = await hass.config_entries.flow.async_init(
        DOMAIN,
        context={"source": config_entries.SOURCE_HASSIO},
        data=discovery(config),
    )
    with (
        patch("custom_components.freedo.config_flow.FreedoClient", DaemonClient),
        patch("custom_components.freedo.async_setup_entry", return_value=True),
    ):
        result = await hass.config_entries.flow.async_configure(result["flow_id"], {})
    assert result["data"] == {CONF_HOST: HOST, CONF_PORT: PORT, "token": ""}


@pytest.mark.parametrize(("chosen", "ssl"), [({}, False), ({CONF_SSL: True}, True)])
async def test_the_user_step_connects_over_the_chosen_scheme_and_stores_it(hass, chosen, ssl):
    result = await hass.config_entries.flow.async_init(
        DOMAIN, context={"source": config_entries.SOURCE_USER}
    )
    assert result["step_id"] == "user"
    DaemonClient.made.clear()
    with (
        patch("custom_components.freedo.config_flow.FreedoClient", DaemonClient),
        patch("custom_components.freedo.async_setup_entry", return_value=True),
    ):
        result = await hass.config_entries.flow.async_configure(
            result["flow_id"],
            {CONF_HOST: "freedo.example", CONF_PORT: 443, "token": TOKEN, **chosen},
        )
    assert result["type"] is FlowResultType.CREATE_ENTRY
    assert result["data"] == {
        CONF_HOST: "freedo.example", CONF_PORT: 443, "token": TOKEN, CONF_SSL: ssl
    }
    assert DaemonClient.made == [("freedo.example", 443, TOKEN, ssl)]


async def test_the_reauth_checks_the_token_over_https_when_the_entry_uses_it(hass):
    entry = MockConfigEntry(
        domain=DOMAIN,
        data={CONF_HOST: "freedo.example", CONF_PORT: 443, "token": "old-token", CONF_SSL: True},
        unique_id="freedo.example:443",
    )
    entry.add_to_hass(hass)
    result = await entry.start_reauth_flow(hass)
    assert result["step_id"] == "reauth_confirm"
    DaemonClient.made.clear()
    with (
        patch("custom_components.freedo.config_flow.FreedoClient", DaemonClient),
        patch("custom_components.freedo.async_setup_entry", return_value=True),
    ):
        result = await hass.config_entries.flow.async_configure(
            result["flow_id"], {"token": TOKEN}
        )
    assert result["type"] is FlowResultType.ABORT
    assert result["reason"] == "reauth_successful"
    assert DaemonClient.made == [("freedo.example", 443, TOKEN, True)]
    assert entry.data["token"] == TOKEN
