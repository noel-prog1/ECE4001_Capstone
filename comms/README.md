# comms: phone → laptop → Cutebots

```
 phone (index.html) ──BLE──▶ LAPTOP (laptop_hub.py + CameraHost) ──BLE UART──▶ Cutebot 0
   colour tracking            peripheral role: phone                 one line, ─▶ Cutebot 1
   on the phone itself        central role: 3 micro:bits             same for all ─▶ Cutebot 2
```

The laptop holds **four Bluetooth connections at once**: the phone, plus one per car. This laptop's Intel adapter supports both roles at the same time. Every car runs the same firmware.

## Files

| File | What it is |
|---|---|
| `protocol.py` | Wire format, plus `BotReceiver`, a Python copy of the firmware logic |
| `links.py` | `BleDirectLink` (real cars), `SimLink` (fake cars), `Pinger` (latency) |
| `laptop_hub.py` | Team's BLE hub (copied out of `files.zip`), used by `BleDirectLink` |
| `firmware/cutebot_ble.ts` | MakeCode program for every car |
| `link_test.py` | Latency, motion and watchdog test |
| `test_comms.py` | Hardware-free tests |

## Protocol

```
I<id>                 laptop → one car after every (re)connect: "you are robot <id>"
C<seq>:<a0><a1><a2>   laptop → all cars, e.g. C1042:15x
                      a = action 0..6 (camera_host IDs), x = tracking invalid → stop
S                     emergency stop
P<id>:<n> / R<id>:<n> ping / reply, for latency measurement
```

* **Which car is which** is decided in one place: the `names` list on the laptop, robot 0 first. Each micro:bit has a fixed 5-letter name (press **B** on the car to scroll it). On every connect the laptop sends that car its ID. A car without an ID ignores all commands, so a rebooted car can't drive as the wrong robot.
* Robot 0 must be the car wearing the "Bot 1 rear" colour on the phone page, robot 1 "Bot 2 rear", and so on.
* Each car reads only its own slot and ignores duplicate or out-of-order sequence numbers. It **stops by itself if no command arrives for 300 ms**, and immediately on disconnect.
* If a car drops, the laptop reconnects it and re-sends its ID automatically.

## Try it without hardware

```
cd comms
python -m unittest -v test_comms
python link_test.py --link sim --drive
```

Set `CUTEBOT_CONTROLLER_DIR` to the extracted `cutebot_variable_grid_controller` folder to also run the CameraHost → wire → cars test.

## In class with the cars

`pip install bleak` (on macOS/Linux also `bless`).

**Easiest:** open `class_day_link_test.ipynb` and run it top to bottom. It covers the steps below and saves the results to `results/`. To rehearse at home, set `LINK = "sim"`.

1. **Flash** `firmware/cutebot_ble.ts` to all three cars. Use MakeCode with micro:bit V2, add the `bluetooth` and `Cutebot` extensions, remove `radio`, and set Project Settings → Bluetooth Pairing to *No Pairing Required*.
2. **Name the cars.** Press B on each, write its name on tape, and decide which marker colour it wears.
3. **Test one car first.** Use placeholder names for the others: `python link_test.py --link ble --names <car> none1 none2`
   * The car should show `?` then `0`.
   * Ping prints RTT and loss per car. One-way latency ≈ RTT/2. This fills in "measured latency" in §3.2 of the report.
4. **Wheels off the floor:** add `--drive`. Robot 0 rotates left (left wheel back, right forward) for about 1 s and stops. Then comes one forward command, and the car must stop *on its own* within 0.3 s.
5. **Test a drop.** Run with `--count 400`, and switch the car off and on mid-run. It should reconnect and show `0` again.
6. **All three cars** (`--names <red> <green> <blue> --drive`): the cars must spin one at a time in robot order. If the wrong car spins, the names are in the wrong order.
7. **Calibrate** on the floor: `--calibrate 0`, then follow the prompts. Put the suggested `FWD`/`SPIN` into the firmware, re-flash all three, and repeat once. The targets, forward ≈ 0.20 m/s and spin ≈ 3 rad/s, are the simulator speeds the policy was trained on.
8. **Real load:** `--phone --count 100`. Connect the phone when prompted: four links on one adapter. Close `phone_test.py` first, because only one program can host the phone link.

Turn off other Bluetooth devices on the laptop (headphones, controllers) during runs. They share the same radio.

Known limit: a car that isn't found during the startup scan isn't retried. Restart with all three switched on.

## Open items for the vision group (phone packet)

The controller needs three object centres, and the current phone packet has only one target. Proposed v2, still one 19-byte BLE write:

```
byte 0       0x80 | frame_seq (0..127)   ≥ 0x80, so the hub can tell it from v1 (byte 0 = N ≤ 4)
bytes 1-6    rear  x,y for bots 0,1,2
bytes 7-12   front x,y for bots 0,1,2
bytes 13-18  object x,y for objects 0,1,2    (255 = not seen, as now)
```

The goal moves to laptop config (`goal_center_m`), as `camera_host` expects. `frame_seq` lets the laptop drop duplicate frames. Seven distinct colours is tight for hue-only tracking, so printed tags may be worth considering.
