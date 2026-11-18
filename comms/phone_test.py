#!/usr/bin/env python3
"""
phone_test.py - check the phone camera page -> laptop Bluetooth link alone.

    python phone_test.py            # run, then press "Connect to laptop" on the phone
    python phone_test.py --seconds 120

Prints once a second: packets received, packet rate and the decoded markers.
No micro:bits needed.
"""
import argparse
import asyncio
import logging
import time

from laptop_hub import Hub


async def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = until Ctrl+C)")
    args = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")

    count = 0

    def on_camera(_pkt) -> None:
        nonlocal count
        count += 1

    hub = Hub(on_camera=on_camera, max_bots=0)
    await hub.start()
    print("Laptop is advertising. On the phone: open the camera page, tap "
          "'Connect to laptop', pick this PC in the list. Ctrl+C to stop.")
    t_end = time.monotonic() + args.seconds if args.seconds else float("inf")
    last = 0
    try:
        while time.monotonic() < t_end:
            await asyncio.sleep(1.0)
            p = hub.latest_camera
            if count != last and p is not None:
                print(f"{count:6d} packets  {count - last:3d}/s  {len(p.raw)} B  {p.scene}")
            last = count
    finally:
        await hub.stop()
        print(f"done: {count} packets")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
