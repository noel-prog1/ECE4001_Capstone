#!/usr/bin/env python3
"""
link_test.py - measure and exercise the laptop -> Cutebot link.

    python link_test.py --link sim                            # no hardware
    python link_test.py --link ble --names tuzov gatep zavog  # robot 0, 1, 2

Use placeholder names (e.g. none1 none2) for cars you are not testing yet.

  ping         always: round-trip time per car (one-way ~ RTT/2) and loss.
  --drive      each car rotates left ~1 s in turn (checks the ID -> car
               mapping), then one forward command and silence (checks that
               every car's watchdog stops it). Lift the cars off the floor.
  --calibrate N  car N drives forward 2.0 s, then rotates left 2.1 s, so you
               can measure it and tune FWD / SPIN in the firmware.
  --phone      (ble) wait for the phone camera page to connect first, then
               ping under real load and report the camera packet rate.
"""
import argparse
import asyncio
import logging
import statistics
import time

import links
import protocol

PERIOD_S = 0.14   # camera_host.DT: the controller's command cadence


class Sender:
    """Streams command lines at the controller's cadence."""

    def __init__(self, link: links.Link) -> None:
        self.link = link
        self.seq = int(time.time()) % protocol.SEQ_MOD

    async def send(self, actions) -> str:
        self.seq += 1
        line = protocol.encode_actions(self.seq, actions)
        await self.link.send_line(line)
        return line

    async def hold(self, actions, seconds: float) -> None:
        """Repeat `actions` every PERIOD_S for `seconds`, then stop everyone."""
        t0 = time.perf_counter()
        next_t = t0
        while time.perf_counter() - t0 < seconds:
            await self.send(actions)
            next_t += PERIOD_S
            await asyncio.sleep(max(0.0, next_t - time.perf_counter()))
        await self.send([0, 0, 0])


def make_link(args, on_camera=None) -> links.Link:
    if args.link == "sim":
        return links.SimLink(latency_ms=args.sim_latency, jitter_ms=args.sim_latency / 2,
                             loss=args.sim_loss, seed=1)
    return links.BleDirectLink(names=args.names, scan_seconds=args.scan, on_camera=on_camera)


async def ping_all(link: links.Link, count: int) -> dict:
    """Prints and returns {robot: {median_ms, p95_ms, min_ms, max_ms, lost, sent}}."""
    pinger = links.Pinger(link)
    results = {}
    print(f"\nping: {count} per car")
    for bot in range(protocol.N_BOTS):
        rtts, sent = [], 0
        for _ in range(count):
            rtt = await pinger.ping(bot)
            sent += 1
            if rtt is not None:
                rtts.append(rtt * 1000)
            elif not rtts and sent >= 5:
                break                              # never answered: absent / placeholder
            await asyncio.sleep(0.02)
        lost = sent - len(rtts)
        if not rtts:
            print(f"  robot {bot}: no replies (connected? showing '{bot}' on its LEDs?)")
            results[bot] = {"median_ms": None, "p95_ms": None, "min_ms": None,
                            "max_ms": None, "lost": lost, "sent": sent}
            continue
        rtts.sort()
        p95 = rtts[min(len(rtts) - 1, int(0.95 * len(rtts)))]
        med = statistics.median(rtts)
        print(f"  robot {bot}: rtt min {rtts[0]:.1f}  median {med:.1f}  "
              f"p95 {p95:.1f}  max {rtts[-1]:.1f} ms   lost {lost}/{sent}")
        results[bot] = {"median_ms": round(med, 1), "p95_ms": round(p95, 1),
                        "min_ms": round(rtts[0], 1), "max_ms": round(rtts[-1], 1),
                        "lost": lost, "sent": sent}
    return results


async def drive_demo(link: links.Link) -> None:
    s = Sender(link)
    print("\ndrive: each car rotates left ~1 s in turn, robot 0 first")
    for bot in range(protocol.N_BOTS):
        acts = [0, 0, 0]
        acts[bot] = 2
        print(f"  robot {bot} rotating left")
        await s.hold(acts, 1.0)
        await asyncio.sleep(0.5)
    await s.send([None, None, None])               # "tracking invalid" -> stop

    print("watchdog: one forward command, then silence - every car should stop by itself "
          f"within {protocol.CMD_TIMEOUT_MS} ms")
    await s.send([1, 1, 1])
    await asyncio.sleep(1.0)

    if isinstance(link, links.SimLink):
        for b in link.bots:
            t0 = b.log[0][0] if b.log else 0
            changes = ", ".join(f"{(t - t0) / 1000:.2f}s:{protocol.ACTION_NAMES[a]}" for t, a in b.log)
            print(f"  sim robot {b.bot_id} action changes: {changes}")


async def calibrate(link: links.Link, robot: int) -> None:
    s = Sender(link)
    acts = [0, 0, 0]
    ask = lambda msg: asyncio.to_thread(input, msg)

    await ask(f"\ncalibrate robot {robot}: put it ON THE FLOOR with ~1 m clear ahead, press Enter ")
    acts[robot] = 1
    await s.hold(acts, 2.0)
    dist = await ask("forward 2.0 s done. Distance driven in cm (target 40): ")

    await ask("put it back, mark its heading, press Enter ")
    acts[robot] = 2
    await s.hold(acts, 2.1)
    deg = await ask("rotate-left 2.1 s done. Degrees turned (target 360, one full turn): ")

    print("\nsuggested firmware values (motors aren't perfectly linear: re-flash and repeat once):")
    try:
        print(f"  const FWD  = round(FWD  x 40 / {float(dist):g})   e.g. 40 -> {round(40 * 40 / float(dist))}")
        print(f"  const SPIN = round(SPIN x 360 / {float(deg):g})   e.g. 30 -> {round(30 * 360 / float(deg))}")
        print("  const REV  = -round(0.65 x FWD)")
    except (ValueError, ZeroDivisionError):
        print("  (numbers not recognised - compute FWD x 40/cm and SPIN x 360/deg by hand)")
    print("  If it curves instead of driving straight, note which way: the wheels need a trim.")


async def wait_for_phone(counter: list, timeout_s: float = 180.0) -> bool:
    print("\nconnect the phone now: camera page -> 'Connect to laptop' -> pick this PC")
    t_end = time.monotonic() + timeout_s
    while time.monotonic() < t_end:
        if counter[0] > 0:
            await asyncio.sleep(2.0)                # let it settle
            print("phone connected, camera packets arriving")
            return True
        await asyncio.sleep(0.25)
    print("no phone packets - continuing without phone load")
    return False


async def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--link", choices=["sim", "ble"], default="sim")
    ap.add_argument("--names", nargs=3, metavar="NAME",
                    help="the 3 micro:bit names (button B on each car), robot 0 first")
    ap.add_argument("--scan", type=float, default=8.0, help="BLE scan seconds")
    ap.add_argument("--count", type=int, default=30, help="pings per car")
    ap.add_argument("--drive", action="store_true", help="motion + watchdog test")
    ap.add_argument("--calibrate", type=int, choices=[0, 1, 2], metavar="ROBOT",
                    help="speed calibration run for one car")
    ap.add_argument("--phone", action="store_true", help="ping with the phone streaming")
    ap.add_argument("--sim-latency", type=float, default=8.0, help="sim one-way latency, ms")
    ap.add_argument("--sim-loss", type=float, default=0.0, help="sim packet loss 0..1")
    args = ap.parse_args()
    if args.link == "ble" and not args.names:
        ap.error("--link ble needs --names NAME0 NAME1 NAME2")
    if args.phone and args.link != "ble":
        ap.error("--phone needs --link ble")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s",
                        datefmt="%H:%M:%S")

    camera = [0]
    link = make_link(args, on_camera=lambda _pkt: camera.__setitem__(0, camera[0] + 1))
    await link.start()
    try:
        if args.phone:
            await wait_for_phone(camera)
        c0, t0 = camera[0], time.monotonic()
        await ping_all(link, args.count)
        if args.phone:
            rate = (camera[0] - c0) / (time.monotonic() - t0)
            print(f"  camera packets during ping: {rate:.1f}/s (page sends ~12.5/s)")
        if args.drive:
            await drive_demo(link)
        if args.calibrate is not None:
            await calibrate(link, args.calibrate)
    finally:
        await link.stop()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
