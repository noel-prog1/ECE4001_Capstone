"""
links.py - laptop -> Cutebot transports.

    link = SimLink()                                   # no hardware: fake bots
    link = BleDirectLink(["tuzov", "gatep", "zavog"])  # robot 0, 1, 2 by micro:bit name

    await link.start()
    link.add_listener(lambda line: print("bot said", line))
    await link.send_line(protocol.encode_host_packets(host.dispatch(...)))
    await link.stop()

Pinger(link).ping(robot_id) measures round-trip time on either link.
"""
from __future__ import annotations

import asyncio
import itertools
import logging
import random
import time
from typing import Callable, Optional

import protocol

log = logging.getLogger("links")
Listener = Callable[[str], None]


class Link:
    """Base class: fan incoming bot lines out to listeners."""

    def __init__(self) -> None:
        self._listeners: list[Listener] = []

    def add_listener(self, fn: Listener) -> None:
        self._listeners.append(fn)

    def _emit(self, line: str) -> None:
        for fn in self._listeners:
            try:
                fn(line)
            except Exception:                       # noqa: BLE001
                log.exception("listener failed")

    async def start(self) -> None: ...
    async def stop(self) -> None: ...

    async def send_line(self, line: str) -> None:
        """Send one protocol line to every connected bot."""
        raise NotImplementedError


# --------------------------------------------------------------------------
# 1. Simulated bots (no hardware)
# --------------------------------------------------------------------------

class SimLink(Link):
    """Three in-process BotReceivers behind a link with configurable one-way
    latency, jitter and packet loss. Note: on Windows asyncio timers have
    ~15 ms granularity, so simulated round trips read longer than configured."""

    def __init__(self, latency_ms: float = 8.0, jitter_ms: float = 4.0,
                 loss: float = 0.0, seed: Optional[int] = None) -> None:
        super().__init__()
        self.bots = [protocol.BotReceiver() for _ in range(protocol.N_BOTS)]
        self.latency_ms, self.jitter_ms, self.loss = latency_ms, jitter_ms, loss
        self._rng = random.Random(seed)
        self._watchdog: Optional[asyncio.Task] = None

    @staticmethod
    def now_ms() -> float:
        return time.monotonic() * 1000.0

    def _delay(self) -> float:
        return max(0.0, self.latency_ms + self._rng.uniform(-1, 1) * self.jitter_ms) / 1000.0

    async def start(self) -> None:
        for i, bot in enumerate(self.bots):          # what BleDirectLink does on connect
            bot.handle(protocol.encode_assign(i), self.now_ms())
        self._watchdog = asyncio.create_task(self._tick_forever())

    async def stop(self) -> None:
        if self._watchdog:
            self._watchdog.cancel()

    async def _tick_forever(self) -> None:
        while True:                                  # firmware runs this every 20 ms
            for b in self.bots:
                b.tick(self.now_ms())
            await asyncio.sleep(0.02)

    async def send_line(self, line: str) -> None:
        if len(line) > protocol.MAX_LINE:
            raise ValueError(f"line too long: {line!r}")
        loop = asyncio.get_running_loop()
        for bot in self.bots:
            if self._rng.random() < self.loss:
                continue
            loop.call_later(self._delay(), self._deliver, bot, line)

    def _deliver(self, bot: protocol.BotReceiver, line: str) -> None:
        reply = bot.handle(line, self.now_ms())
        if reply and self._rng.random() >= self.loss:
            asyncio.get_running_loop().call_later(self._delay(), self._emit, reply)


# --------------------------------------------------------------------------
# 2. Direct BLE to each micro:bit (firmware/cutebot_ble.ts), via laptop_hub.py
# --------------------------------------------------------------------------

class BleDirectLink(Link):
    """One BLE connection per micro:bit through laptop_hub.Hub.

    `names` are the micro:bits' 5-letter names IN ROBOT ID ORDER (robot 0
    first). That list is the single place the camera's robot IDs are tied to
    physical cars: after every connect or reconnect the car is sent "I<id>"
    for its name, so a rebooted car can't drive with a stale ID.

    The same Hub also serves the phone camera link: pass on_camera here rather
    than creating a second Hub (one process must own both Bluetooth roles).
    Dropped cars are reconnected automatically. A car that was not found at
    start() is not retried - restart with all three powered on."""

    def __init__(self, names: list[str], on_camera=None, scan_seconds: float = 8.0,
                 retry_s: float = 1.0, hub_cls=None) -> None:
        super().__init__()
        self.names = [n.lower() for n in names or []]
        if len(self.names) != protocol.N_BOTS or len(set(self.names)) != protocol.N_BOTS:
            raise ValueError(f"give {protocol.N_BOTS} distinct micro:bit names, robot 0 first")
        if hub_cls is None:
            try:
                from laptop_hub import Hub as hub_cls
            except ImportError as exc:
                raise ImportError("BleDirectLink needs `pip install bleak bless`") from exc
        self.hub = hub_cls(on_camera=on_camera,
                           on_bot_message=lambda _bot, line: self._emit(line),
                           on_bot_disconnect=self._on_drop,
                           names=list(names), max_bots=protocol.N_BOTS,
                           scan_seconds=scan_seconds)
        self.retry_s = retry_s
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._running = False
        self._reconnecting: set[str] = set()

    def robot_id(self, bot) -> Optional[int]:
        """Robot ID for a hub BotLink, from its advertised name."""
        hits = [i for i, n in enumerate(self.names) if n in bot.name.lower()]
        return hits[0] if len(hits) == 1 else None

    async def start(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._running = True
        await self.hub.start()
        for bot in self.hub.bots:
            await self._assign(bot)
        have = {self.robot_id(b) for b in self.hub.bots}
        missing = [n for i, n in enumerate(self.names) if i not in have]
        if missing:
            log.warning("NOT connected: %s (robot ids %s)", missing,
                        [self.names.index(n) for n in missing])

    async def _assign(self, bot) -> bool:
        rid = self.robot_id(bot)
        if rid is None:
            log.error("cannot tell which robot %r is; it will stay parked", bot.name)
            return False
        ok = await bot.send_line(protocol.encode_assign(rid))
        if ok:
            log.info("%s is robot %d", bot.name, rid)
        return ok

    def _on_drop(self, index: int) -> None:
        bot = next((b for b in self.hub.bots if b.index == index), None)
        if bot is not None and self._running and self._loop is not None:
            # bleak may call this off the event-loop thread
            self._loop.call_soon_threadsafe(self._start_reconnect, bot)

    def _start_reconnect(self, bot) -> None:
        if bot.name not in self._reconnecting:
            self._reconnecting.add(bot.name)
            self._loop.create_task(self._reconnect(bot))

    async def _reconnect(self, bot) -> None:
        try:
            while self._running and not bot.connected:
                try:
                    await bot.connect()
                    await self._assign(bot)
                    log.info("reconnected %s", bot.name)
                except Exception as exc:              # noqa: BLE001
                    log.warning("reconnect %s failed: %s", bot.name, exc)
                    await asyncio.sleep(self.retry_s)
        finally:
            self._reconnecting.discard(bot.name)

    async def send_line(self, line: str) -> None:
        delivered = await self.hub.send("all", line)
        if delivered < protocol.N_BOTS:
            log.debug("line %r reached %d bot(s)", line, delivered)

    async def stop(self) -> None:
        self._running = False
        try:
            await self.hub.send("all", protocol.STOP_ALL)
        finally:
            await self.hub.stop()


# --------------------------------------------------------------------------
# Round-trip measurement (works on every link)
# --------------------------------------------------------------------------

class Pinger:
    def __init__(self, link: Link) -> None:
        self.link = link
        self._waiting: dict[tuple[int, int], asyncio.Future] = {}
        self._counter = itertools.count(1)
        link.add_listener(self._on_line)

    def _on_line(self, line: str) -> None:
        got = protocol.parse_id_num(line, "R")
        fut = self._waiting.pop(got, None) if got else None
        if fut and not fut.done():
            fut.set_result(time.perf_counter())

    async def ping(self, robot_id: int, timeout: float = 0.5) -> Optional[float]:
        """Round-trip time in seconds, or None if no reply."""
        n = next(self._counter) % protocol.SEQ_MOD
        fut = asyncio.get_running_loop().create_future()
        self._waiting[(robot_id, n)] = fut
        t0 = time.perf_counter()
        await self.link.send_line(protocol.encode_ping(robot_id, n))
        try:
            return await asyncio.wait_for(fut, timeout) - t0
        except asyncio.TimeoutError:
            self._waiting.pop((robot_id, n), None)
            return None
