"""
protocol.py - wire format between the laptop and the Cutebots (BLE UART).

The laptop holds one BLE connection per micro:bit (laptop_hub.py). Every line
is <= 19 characters, so it fits one 20-byte BLE write with its newline.

Laptop -> one bot
    I<id>                 assign robot ID 0..2. Sent right after every (re)connect;
                          the laptop decides which micro:bit is which from its
                          name. A bot with no ID ignores commands and pings, so a
                          rebooted car can never drive with the wrong ID.

Laptop -> all bots
    C<seq>:<a0><a1><a2>   one command line, the same for all three bots.
                          seq  = 0..65535, wraps around.
                          a<i> = action digit 0..6 for bot i, or 'x' when the
                                 camera host said tracking is invalid (stop).
                          e.g. "C1042:15x"  bot0 forward, bot1 fwd-right, bot2 stop
    S                     emergency stop: every bot stops at once, no seq check.
    P<id>:<n>             ping bot <id> (n = 0..65535).

Bots -> laptop
    R<id>:<n>             reply to ping n.

Bot rules (implemented in firmware/cutebot_ble.ts, mirrored by BotReceiver below):
  * stop and forget its ID when the BLE link drops;
  * read only its own slot a<BOT_ID>;
  * accept a command only if seq is newer than the last one (mod 65536),
    except after SESSION_RESET_MS of silence, when any seq starts a new session
    (this handles a laptop restart);
  * stop if no accepted command arrives within CMD_TIMEOUT_MS.

Action IDs match arena.py / camera_host.py:
  0 stop, 1 forward, 2 rotate left, 3 rotate right, 4 forward-left,
  5 forward-right, 6 reverse.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Sequence

N_BOTS = 3
SEQ_MOD = 65536
MAX_LINE = 19
INVALID = "x"
ACTION_NAMES = ("stop", "forward", "rotate-left", "rotate-right",
                "forward-left", "forward-right", "reverse")

# Firmware timing (keep in sync with firmware/cutebot_ble.ts).
CMD_TIMEOUT_MS = 300      # ~2 control periods (140 ms) of slack before stopping
SESSION_RESET_MS = 1000   # silence after which any seq starts a new session


# --------------------------------------------------------------------------
# Encoding (laptop side)
# --------------------------------------------------------------------------

def encode_actions(seq: int, actions: Sequence[Optional[int]]) -> str:
    """actions[i] = 0..6, or None for 'tracking invalid' -> 'x'."""
    if len(actions) != N_BOTS:
        raise ValueError(f"need {N_BOTS} actions, got {len(actions)}")
    slots = []
    for a in actions:
        if a is None:
            slots.append(INVALID)
        elif isinstance(a, int) and 0 <= a <= 6:
            slots.append(str(a))
        else:
            raise ValueError(f"bad action {a!r}")
    return f"C{seq % SEQ_MOD}:{''.join(slots)}"


def encode_host_packets(packets: Sequence[dict]) -> str:
    """Turn the 3 dicts returned by CameraHost.dispatch() into one line."""
    by_id = {p["robot_id"]: p for p in packets}
    if sorted(by_id) != list(range(N_BOTS)):
        raise ValueError("expected one packet per robot id 0..2")
    seqs = {p["sequence"] for p in packets}
    if len(seqs) != 1:
        raise ValueError("packets from one dispatch() share a sequence number")
    actions = [by_id[i]["action"] if by_id[i].get("tracking_valid") else None
               for i in range(N_BOTS)]
    return encode_actions(seqs.pop(), actions)


def encode_ping(bot_id: int, n: int) -> str:
    return f"P{bot_id}:{n % SEQ_MOD}"


def encode_assign(bot_id: int) -> str:
    if not 0 <= bot_id < N_BOTS:
        raise ValueError(f"bad robot id {bot_id}")
    return f"I{bot_id}"


STOP_ALL = "S"


# --------------------------------------------------------------------------
# Decoding
# --------------------------------------------------------------------------

@dataclass
class Command:
    seq: int
    actions: list[Optional[int]]   # None = invalid tracking


def parse_command(line: str) -> Optional[Command]:
    if not line.startswith("C"):
        return None
    head, sep, slots = line[1:].partition(":")
    if not sep or len(slots) != N_BOTS or not head.isdigit() or int(head) >= SEQ_MOD:
        return None
    actions: list[Optional[int]] = []
    for ch in slots:
        if ch == INVALID:
            actions.append(None)
        elif ch in "0123456":
            actions.append(int(ch))
        else:
            return None
    return Command(int(head), actions)


def parse_id_num(line: str, prefix: str) -> Optional[tuple[int, int]]:
    """'P1:77' / 'R1:77' -> (1, 77)."""
    if not line.startswith(prefix):
        return None
    a, sep, b = line[1:].partition(":")
    if not sep or not a.isdigit() or not b.isdigit():
        return None
    return int(a), int(b)


def seq_is_newer(seq: int, last: int) -> bool:
    d = (seq - last) % SEQ_MOD
    return 0 < d < SEQ_MOD // 2


# --------------------------------------------------------------------------
# Reference receiver (exactly what each bot's firmware does)
# --------------------------------------------------------------------------

@dataclass
class BotReceiver:
    """Python twin of the firmware's message handler. Used by the simulated
    link and the unit tests so protocol behaviour can be verified without a
    Cutebot. `now_ms` is supplied by the caller (firmware uses control.millis())."""
    bot_id: Optional[int] = None          # None until the laptop sends I<id>
    action: int = 0                       # what the motors are doing now
    last_seq: int = -1
    last_cmd_ms: float = -1e12
    log: list = field(default_factory=list)   # (now_ms, action) on every change

    def _set(self, now_ms: float, action: int) -> None:
        if action != self.action:
            self.log.append((now_ms, action))
        self.action = action

    def handle(self, line: str, now_ms: float) -> Optional[str]:
        """Process one line; returns a reply line or None."""
        if line == STOP_ALL:
            self._set(now_ms, 0)
            return None
        if len(line) == 2 and line[0] == "I" and line[1] in "012":
            self._set(now_ms, 0)
            self.bot_id = int(line[1])
            return None
        if self.bot_id is None:
            return None                   # unassigned: never move
        if (ping := parse_id_num(line, "P")) is not None:
            return f"R{self.bot_id}:{ping[1]}" if ping[0] == self.bot_id else None
        cmd = parse_command(line)
        if cmd is None:
            return None
        new_session = now_ms - self.last_cmd_ms > SESSION_RESET_MS
        if not new_session and not seq_is_newer(cmd.seq, self.last_seq):
            return None                   # duplicate or out of order
        self.last_seq = cmd.seq
        self.last_cmd_ms = now_ms
        a = cmd.actions[self.bot_id]
        self._set(now_ms, 0 if a is None else a)
        return None

    def disconnected(self, now_ms: float) -> None:
        """BLE link dropped: stop and wait to be re-assigned."""
        self._set(now_ms, 0)
        self.bot_id = None

    def tick(self, now_ms: float) -> None:
        """Watchdog: call periodically (firmware does every 20 ms)."""
        if self.action != 0 and now_ms - self.last_cmd_ms > CMD_TIMEOUT_MS:
            self._set(now_ms, 0)
