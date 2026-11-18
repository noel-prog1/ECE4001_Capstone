#!/usr/bin/env python3
"""
laptop_hub.py  -  Bluetooth communication hub (no control logic)

PURPOSE
-------
Moves data between three kinds of device. It never decides anything; your own
control code decides what to send and what to do with what arrives.

    smartphone ──camera data──▶ LAPTOP ◀──text lines both ways──▶ micro:bit 1..N

The laptop plays two BLE roles at once:
  * PERIPHERAL (GATT server, `bless`) - the phone connects to it and writes
    camera packets.
  * CENTRAL    (GATT client, `bleak`) - it connects to every micro:bit through
    the Nordic UART Service (serial-over-BLE).

USE AS A LIBRARY
----------------
    import asyncio
    from laptop_hub import Hub

    def on_camera(pkt):                       # called for every phone packet
        print(pkt.raw, pkt.scene)

    def on_bot_message(bot, line):            # called for every line a bot sends
        print(f"bot {bot}: {line}")

    async def main():
        hub = Hub(on_camera=on_camera, on_bot_message=on_bot_message)
        await hub.start()                     # connects bots, then advertises to phone
        await hub.send(1, "HELLO")            # one bot
        await hub.send("all", "GO")           # every bot
        await asyncio.sleep(60)
        await hub.stop()

    asyncio.run(main())

Callbacks may be normal functions or `async def`; they run on the asyncio loop.

RUN AS A TEST TOOL
------------------
    python laptop_hub.py [--names tuzov gatep] [--scan 8]
Prints everything received and lets you type:  send <n|all> <text> | list | camera | quit

MESSAGE FORMAT (laptop <-> micro:bit)
-------------------------------------
Plain ASCII text lines ending in "\\n". A BLE write carries at most 20 bytes,
so each message is limited to 19 characters plus the newline. send() raises
ValueError for anything longer.

PHONE -> LAPTOP PACKET (written by phone_camera.html)
-----------------------------------------------------
    byte 0 = N (number of bots), then 3 + 4*N bytes in total:
    [N, targetX, targetY, rear1X, rear1Y .. rearNX, rearNY, front1X, front1Y .. frontNX, frontNY]
Coordinates are 0..254 (left/top .. right/bottom), 255 = marker not seen.
`CameraPacket.raw` is always the untouched bytes; `CameraPacket.scene` is the
decoded form (None if the packet is malformed).

WINDOWS NOTES
-------------
  * `pip install bleak` (Python 3.9+; bless is only needed on macOS/Linux).
    The phone link needs a Bluetooth 4.0+ adapter that supports the
    peripheral role.
  * Windows advertises the PC's own name; the phone finds the laptop by service
    UUID, not by name.
  * Bots are connected BEFORE advertising starts (more reliable on Windows).
"""

import argparse          # command-line options for the test tool
import asyncio           # BLE libraries are asynchronous
import inspect           # to detect async callbacks
import logging           # diagnostics
import sys               # pick the phone-link backend per OS
import time              # timestamps for received data
import uuid              # WinRT wants UUID objects
from dataclasses import dataclass
from typing import Any, Callable, Optional, Union

from bleak import BleakClient, BleakScanner          # BLE central role
# BLE peripheral role: on Windows the WinRT bindings that bleak already
# installs are used directly (bless does not install on Python 3.13 there);
# elsewhere `bless` is imported lazily in BlessPhoneLink.

log = logging.getLogger("hub")

# --------------------------------------------------------------------------
# 1. CONSTANTS
# --------------------------------------------------------------------------

# Nordic UART Service (fixed by the micro:bit firmware). Names are from the
# micro:bit's point of view: we WRITE to RX and receive NOTIFICATIONS from TX.
UART_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   # laptop -> micro:bit
UART_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   # micro:bit -> laptop

# Custom service/characteristic for the phone. Must equal phone_camera.html.
PHONE_SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
PHONE_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef1"
ADVERTISED_NAME = "CamHub"                 # used on Linux/macOS; Windows ignores it

MICROBIT_NAME_PREFIX = "BBC micro:bit"     # every micro:bit advertises this prefix
MAX_MESSAGE_BYTES = 19                     # + 1 newline = one 20-byte BLE write
NO_VALUE = 255                             # "marker not seen" in a camera packet
MAX_BOTS = 4                               # limited by the 19-byte camera packet
ASPECT = 4.0 / 3.0                         # image width / height (phone crops to 4:3)

Point = tuple[float, float]                # (x, y) in image units: height = 1.0
Callback = Callable[..., Any]


# --------------------------------------------------------------------------
# 2. CAMERA DATA
# --------------------------------------------------------------------------

@dataclass
class Scene:
    """Decoded camera packet. Coordinates are in image units (x: 0..4/3,
    y: 0..1, y grows downward) so distances/angles are not distorted."""
    target: Optional[Point]            # target marker, None if not seen
    rears: list[Optional[Point]]       # rears[i] = rear marker of bot i+1
    fronts: list[Point]                # every front marker seen (unlabelled)


@dataclass
class CameraPacket:
    """One packet from the phone."""
    raw: bytes                         # exact bytes received
    scene: Optional[Scene]             # decoded form, None if malformed
    timestamp: float                   # laptop time.monotonic() on arrival


def _decode_point(xb: int, yb: int) -> Optional[Point]:
    """Two packet bytes -> image units; None if the marker was not seen."""
    if xb == NO_VALUE or yb == NO_VALUE:
        return None
    return (xb / 254.0 * ASPECT, yb / 254.0)


def parse_scene(data: bytes) -> Optional[Scene]:
    """Decode a phone packet; None if malformed."""
    if len(data) < 1:
        return None
    n = data[0]
    if not 1 <= n <= MAX_BOTS or len(data) < 3 + 4 * n:
        return None
    target = _decode_point(data[1], data[2])
    rears = [_decode_point(data[3 + 2 * i], data[4 + 2 * i]) for i in range(n)]
    base = 3 + 2 * n
    fronts = [p for j in range(n)
              if (p := _decode_point(data[base + 2 * j], data[base + 2 * j + 1])) is not None]
    return Scene(target, rears, fronts)


# --------------------------------------------------------------------------
# 3. CALLBACK HELPER
# --------------------------------------------------------------------------

def _invoke(callback: Optional[Callback], *args) -> None:
    """Call a user callback (sync or async) without letting its errors
    propagate into the Bluetooth machinery."""
    if callback is None:
        return
    try:
        result = callback(*args)
        if inspect.isawaitable(result):
            asyncio.ensure_future(result)       # run async callbacks as tasks
    except Exception:                           # noqa: BLE001
        log.exception("error in user callback")


# --------------------------------------------------------------------------
# 4. ONE MICRO:BIT (BLE central side)
# --------------------------------------------------------------------------

class BotLink:
    """One BLE connection to one micro:bit."""

    def __init__(self, device, index: int, name: str,
                 on_message: Callable[[int, str], None],
                 on_disconnect: Callable[[int], None]):
        self.index = index                    # bot number, 1-based
        self.name = name                      # e.g. "BBC micro:bit [tuzov]"
        self.connected = False
        self._on_message = on_message
        self._on_disconnect_cb = on_disconnect
        self._rx_buffer = bytearray()         # partial line received from the bot
        self._write_lock = asyncio.Lock()     # one write at a time per bot
        self.client = BleakClient(device, disconnected_callback=self._handle_disconnect)

    async def connect(self) -> None:
        """Open the connection and subscribe to the bot's outgoing data."""
        await self.client.connect()
        await self.client.start_notify(UART_TX_UUID, self._handle_notify)
        self.connected = True

    async def disconnect(self) -> None:
        """Close the connection; ignore errors if already closed."""
        try:
            await self.client.disconnect()
        except Exception:                     # noqa: BLE001 - best effort
            pass
        self.connected = False

    def _handle_disconnect(self, _client) -> None:
        """bleak calls this when the link drops."""
        was_connected, self.connected = self.connected, False
        if was_connected:
            log.warning("bot %d (%s) disconnected", self.index, self.name)
            _invoke(self._on_disconnect_cb, self.index)

    def _handle_notify(self, _characteristic, data: bytearray) -> None:
        """Reassemble lines: one line may arrive split over several
        notifications, and one notification may hold several lines."""
        self._rx_buffer.extend(data)
        while b"\n" in self._rx_buffer:
            line, _, rest = self._rx_buffer.partition(b"\n")
            self._rx_buffer = bytearray(rest)
            _invoke(self._on_message, self.index,
                    line.decode("ascii", errors="replace").rstrip("\r"))

    async def send_line(self, text: str) -> bool:
        """Write one already-validated line; False if the bot is unreachable."""
        if not self.connected:
            return False
        async with self._write_lock:
            try:
                # response=False -> "write without response": lower latency.
                await self.client.write_gatt_char(
                    UART_RX_UUID, (text + "\n").encode("ascii"), response=False)
                return True
            except Exception as exc:          # noqa: BLE001
                log.warning("send to bot %d failed: %s", self.index, exc)
                self.connected = False
                return False


# --------------------------------------------------------------------------
# 5. THE PHONE LINK (BLE peripheral side)
# --------------------------------------------------------------------------

class WinRTPhoneLink:
    """GATT server for the phone page, built straight on Windows' WinRT
    GattServiceProvider (the same API bless wraps)."""

    def __init__(self, on_packet: Callable[[bytes], None]) -> None:
        self._on_packet = on_packet
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._provider = None
        self._char = None
        self._tokens: list = []

    async def start(self) -> None:
        from winrt.windows.devices.bluetooth import BluetoothError
        from winrt.windows.devices.bluetooth.genericattributeprofile import (
            GattCharacteristicProperties, GattLocalCharacteristicParameters,
            GattProtectionLevel, GattServiceProvider,
            GattServiceProviderAdvertisingParameters)

        self._loop = asyncio.get_running_loop()
        made = await GattServiceProvider.create_async(uuid.UUID(PHONE_SERVICE_UUID))
        if made.error != BluetoothError.SUCCESS:
            raise RuntimeError(f"cannot create phone GATT service: {made.error!r}")
        self._provider = made.service_provider

        params = GattLocalCharacteristicParameters()
        params.characteristic_properties = (GattCharacteristicProperties.WRITE
                                            | GattCharacteristicProperties.WRITE_WITHOUT_RESPONSE)
        params.write_protection_level = GattProtectionLevel.PLAIN
        params.user_description = "camera packets"
        made = await self._provider.service.create_characteristic_async(
            uuid.UUID(PHONE_CHAR_UUID), params)
        if made.error != BluetoothError.SUCCESS:
            raise RuntimeError(f"cannot create phone characteristic: {made.error!r}")
        self._char = made.characteristic
        self._tokens.append(self._char.add_write_requested(self._on_write_requested))
        self._tokens.append(self._provider.add_advertisement_status_changed(self._on_adv_status))

        adv = GattServiceProviderAdvertisingParameters()
        adv.is_discoverable = True                    # puts the service UUID in the advert
        adv.is_connectable = True
        self._provider.start_advertising_with_parameters(adv)
        log.info("advertising the camera service - connect from the phone page")

    def _on_adv_status(self, _provider, args) -> None:
        log.info("phone service advertisement: %s", args.status.name)
        if args.error and int(args.error):
            log.error("advertising error: %r", args.error)

    def _on_write_requested(self, _char, args) -> None:
        """Called on a WinRT thread; finish the request on the asyncio loop."""
        deferral = args.get_deferral()
        asyncio.run_coroutine_threadsafe(self._finish_write(args, deferral), self._loop)

    async def _finish_write(self, args, deferral) -> None:
        from winrt.windows.devices.bluetooth.genericattributeprofile import GattWriteOption
        try:
            request = await args.get_request_async()
            if request is None:                       # access denied by the OS
                return
            data = bytes(request.value)
            if request.option == GattWriteOption.WRITE_WITH_RESPONSE:
                request.respond()
            self._on_packet(data)
        except Exception:                             # noqa: BLE001
            log.exception("phone write failed")
        finally:
            deferral.complete()

    async def stop(self) -> None:
        if self._provider is not None:
            try:
                self._provider.stop_advertising()
            except Exception:                         # noqa: BLE001 - best effort
                pass


class BlessPhoneLink:
    """GATT server that the phone's web page writes camera packets into
    (macOS / Linux, via bless)."""

    def __init__(self, on_packet: Callable[[bytes], None]) -> None:
        self._on_packet = on_packet
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self.server = None

    async def start(self) -> None:
        """Create the GATT service and begin advertising."""
        from bless import (BlessServer, GATTAttributePermissions,
                           GATTCharacteristicProperties)
        self._loop = asyncio.get_running_loop()
        self.server = BlessServer(name=ADVERTISED_NAME, loop=self._loop)
        # bless calls this whenever the phone writes to the characteristic.
        self.server.write_request_func = self._on_write

        await self.server.add_new_service(PHONE_SERVICE_UUID)
        properties = (GATTCharacteristicProperties.write
                      | GATTCharacteristicProperties.write_without_response)
        await self.server.add_new_characteristic(
            PHONE_SERVICE_UUID, PHONE_CHAR_UUID, properties, None,
            GATTAttributePermissions.writeable)
        await self.server.start()             # begin advertising
        log.info("advertising the camera service - connect from the phone page")

    def _on_write(self, characteristic, value, **_kwargs) -> None:
        """bless may call this from a non-asyncio thread (Windows), so the
        packet is handed to the asyncio loop thread-safely."""
        characteristic.value = value
        data = bytes(value)
        if self._loop is not None:
            self._loop.call_soon_threadsafe(self._on_packet, data)

    async def stop(self) -> None:
        if self.server is not None:
            await self.server.stop()


PhoneLink = WinRTPhoneLink if sys.platform == "win32" else BlessPhoneLink


# --------------------------------------------------------------------------
# 6. THE HUB (public API)
# --------------------------------------------------------------------------

class Hub:
    """Connects to the bots and the phone and relays data between them and
    your code."""

    def __init__(self,
                 on_camera: Optional[Callable[[CameraPacket], Any]] = None,
                 on_bot_message: Optional[Callable[[int, str], Any]] = None,
                 on_bot_disconnect: Optional[Callable[[int], Any]] = None,
                 scan_seconds: float = 8.0,
                 names: Optional[list[str]] = None,
                 max_bots: int = MAX_BOTS) -> None:
        """
        on_camera(packet)          every phone packet (CameraPacket)
        on_bot_message(bot, line)  every line a bot sends (bot is 1-based)
        on_bot_disconnect(bot)     a bot's link dropped
        names                      micro:bit name fragments; their ORDER defines
                                   bot 1, 2, ... (otherwise alphabetical by name)
        """
        self._on_camera = on_camera
        self._on_bot_message = on_bot_message
        self._on_bot_disconnect = on_bot_disconnect
        self.scan_seconds = scan_seconds
        self.names = names or []
        self.max_bots = min(max_bots, MAX_BOTS)

        self.bots: list[BotLink] = []                  # connected bots
        self.latest_camera: Optional[CameraPacket] = None   # most recent packet
        self._phone = PhoneLink(self._handle_phone_packet)

    # -- lifecycle ---------------------------------------------------------

    async def start(self) -> None:
        """Scan for and connect to the micro:bits, then start advertising to
        the phone."""
        if self.max_bots > 0:                          # max_bots=0: phone only
            await self._connect_bots()
            if not self.bots:
                log.error("no micro:bits connected; continuing so the phone link can be used")
        await self._phone.start()

    async def stop(self) -> None:
        """Disconnect everything."""
        await asyncio.gather(*(b.disconnect() for b in self.bots))
        await self._phone.stop()

    # -- sending -----------------------------------------------------------

    async def send(self, bot: Union[int, str], text: str) -> int:
        """Send one text line to bot number `bot` (1-based) or to "all".

        Returns how many bots accepted the write. Raises ValueError if the
        text is not plain ASCII, contains a newline, or exceeds
        MAX_MESSAGE_BYTES.
        """
        if not text.isascii() or "\n" in text or "\r" in text:
            raise ValueError("message must be ASCII without line breaks")
        if len(text) > MAX_MESSAGE_BYTES:
            raise ValueError(f"message is {len(text)} bytes; limit is {MAX_MESSAGE_BYTES}")

        targets = self.bots if bot == "all" else [b for b in self.bots if b.index == bot]
        results = await asyncio.gather(*(b.send_line(text) for b in targets))
        return sum(results)

    # -- internals ---------------------------------------------------------

    def _handle_phone_packet(self, data: bytes) -> None:
        """Runs on the asyncio loop for every phone packet."""
        packet = CameraPacket(raw=data, scene=parse_scene(data), timestamp=time.monotonic())
        self.latest_camera = packet
        _invoke(self._on_camera, packet)

    async def _connect_bots(self) -> None:
        """Scan, order and connect to the micro:bits one at a time (parallel
        connection attempts are unreliable on some Bluetooth stacks)."""
        log.info("scanning for micro:bits for %.0f s ...", self.scan_seconds)
        found = await BleakScanner.discover(timeout=self.scan_seconds, return_adv=True)

        candidates = []                                  # (sort_key, device, name)
        for _addr, (device, adv) in found.items():
            name = adv.local_name or device.name or ""
            if not name.startswith(MICROBIT_NAME_PREFIX):
                continue                                 # not a micro:bit
            if self.names:
                hits = [k for k, f in enumerate(self.names) if f.lower() in name.lower()]
                if not hits:
                    continue                             # not a requested bot
                candidates.append((hits[0], device, name))
            else:
                candidates.append((name, device, name))
        candidates.sort(key=lambda c: c[0])
        log.info("found %d micro:bit(s)", len(candidates))

        for _key, device, name in candidates[:self.max_bots]:
            bot = BotLink(device, len(self.bots) + 1, name,
                          self._handle_bot_message, self._handle_bot_disconnect)
            for attempt in (1, 2, 3):                    # BLE connects often fail once
                try:
                    await bot.connect()
                    self.bots.append(bot)
                    log.info("connected: bot %d = %s", bot.index, name)
                    break
                except Exception as exc:                 # noqa: BLE001
                    log.warning("connect to %s failed (attempt %d): %s", name, attempt, exc)
                    await asyncio.sleep(1.0)

    def _handle_bot_message(self, bot: int, line: str) -> None:
        _invoke(self._on_bot_message, bot, line)

    def _handle_bot_disconnect(self, bot: int) -> None:
        _invoke(self._on_bot_disconnect, bot)


# --------------------------------------------------------------------------
# 7. TEST TOOL (python laptop_hub.py)
# --------------------------------------------------------------------------

async def _demo() -> None:
    parser = argparse.ArgumentParser(description="BLE communication hub test tool")
    parser.add_argument("--scan", type=float, default=8.0, help="scan time in seconds")
    parser.add_argument("--names", nargs="*", default=[],
                        help="micro:bit name fragments; their order defines bot 1, 2, ...")
    parser.add_argument("--max-bots", type=int, default=MAX_BOTS)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")

    hub = Hub(
        on_bot_message=lambda bot, line: print(f"\n[bot {bot}] {line}"),
        on_bot_disconnect=lambda bot: print(f"\n[bot {bot}] DISCONNECTED"),
        scan_seconds=args.scan, names=args.names, max_bots=args.max_bots)

    try:
        await hub.start()
        print("commands: send <n|all> <text> | list | camera | quit")
        while True:
            # input() blocks, so run it in a worker thread to keep BLE alive.
            parts = (await asyncio.to_thread(input, "> ")).split(maxsplit=2)
            if not parts:
                continue
            cmd = parts[0].lower()
            if cmd in ("quit", "exit"):
                break
            elif cmd == "list":
                for b in hub.bots:
                    print(f"  bot {b.index}: {b.name} {'connected' if b.connected else 'DISCONNECTED'}")
            elif cmd == "camera":
                p = hub.latest_camera
                print("  no camera data yet" if p is None else
                      f"  age {time.monotonic() - p.timestamp:.2f}s raw={list(p.raw)}\n  {p.scene}")
            elif cmd == "send" and len(parts) == 3:
                target = "all" if parts[1] == "all" else (int(parts[1]) if parts[1].isdigit() else None)
                if target is None:
                    print("  target must be a bot number or 'all'")
                    continue
                try:
                    print(f"  delivered to {await hub.send(target, parts[2])} bot(s)")
                except ValueError as exc:
                    print(f"  not sent: {exc}")
            else:
                print("  commands: send <n|all> <text> | list | camera | quit")
    finally:
        await hub.stop()


if __name__ == "__main__":
    try:
        asyncio.run(_demo())
    except KeyboardInterrupt:
        pass
