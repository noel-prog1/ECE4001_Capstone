"""Hardware-free tests for the comms layer.   python -m unittest -v test_comms

Set CUTEBOT_CONTROLLER_DIR to the extracted cutebot_variable_grid_controller
folder to also run the CameraHost -> wire -> bots integration test."""
import asyncio
import os
import sys
import unittest

import links
import protocol as P


class Encoding(unittest.TestCase):
    def test_roundtrip_and_length(self):
        line = P.encode_actions(65535, [6, None, 0])
        self.assertEqual(line, "C65535:6x0")
        self.assertLessEqual(len(line), P.MAX_LINE)
        cmd = P.parse_command(line)
        self.assertEqual((cmd.seq, cmd.actions), (65535, [6, None, 0]))

    def test_seq_wraps(self):
        self.assertEqual(P.encode_actions(65536 + 7, [0, 0, 0]), "C7:000")

    def test_host_packets(self):
        pk = [{"robot_id": 2, "sequence": 9, "tracking_valid": True, "action": 5},
              {"robot_id": 0, "sequence": 9, "tracking_valid": True, "action": 1},
              {"robot_id": 1, "sequence": 9, "tracking_valid": False, "action": 0}]
        self.assertEqual(P.encode_host_packets(pk), "C9:1x5")

    def test_rejects_malformed(self):
        for bad in ["C12", "C1:99", "C1:0000", "Cabc:000", "C1:7xx", "C:000", "C70000:000", "X1:000"]:
            self.assertIsNone(P.parse_command(bad), bad)
        with self.assertRaises(ValueError):
            P.encode_actions(1, [7, 0, 0])


class Receiver(unittest.TestCase):
    def setUp(self):
        self.bot = P.BotReceiver(bot_id=1)

    def test_reads_own_slot(self):
        self.bot.handle("C1:052", 0)
        self.assertEqual(self.bot.action, 5)

    def test_invalid_tracking_stops(self):
        self.bot.handle("C1:010", 0)
        self.bot.handle("C2:0x0", 100)
        self.assertEqual(self.bot.action, 0)

    def test_duplicate_and_old_rejected(self):
        self.bot.handle("C10:010", 0)
        self.bot.handle("C10:020", 10)          # duplicate (bridge repeat)
        self.bot.handle("C9:030", 20)           # late, out of order
        self.assertEqual(self.bot.action, 1)

    def test_wraparound_accepted(self):
        self.bot.handle("C65535:010", 0)
        self.bot.handle("C0:020", 100)
        self.assertEqual(self.bot.action, 2)

    def test_new_session_after_silence(self):
        self.bot.handle("C5000:010", 0)
        self.bot.handle("C3:020", P.SESSION_RESET_MS + 50)   # laptop restarted
        self.assertEqual(self.bot.action, 2)

    def test_watchdog(self):
        self.bot.handle("C1:010", 0)
        self.bot.tick(P.CMD_TIMEOUT_MS - 10)
        self.assertEqual(self.bot.action, 1)
        self.bot.tick(P.CMD_TIMEOUT_MS + 10)
        self.assertEqual(self.bot.action, 0)

    def test_unassigned_never_moves(self):
        bot = P.BotReceiver()
        bot.handle("C1:111", 0)
        self.assertEqual(bot.action, 0)
        self.assertIsNone(bot.handle("P0:1", 0))
        bot.handle("I2", 10)
        bot.handle("C2:004", 20)
        self.assertEqual(bot.action, 4)

    def test_disconnect_forgets_id(self):
        self.bot.handle("C1:010", 0)
        self.bot.disconnected(50)
        self.assertEqual((self.bot.action, self.bot.bot_id), (0, None))
        self.bot.handle("C2:010", 60)
        self.assertEqual(self.bot.action, 0)

    def test_stop_all_and_ping(self):
        self.bot.handle("C1:010", 0)
        self.bot.handle("S", 5)
        self.assertEqual(self.bot.action, 0)
        self.assertEqual(self.bot.handle("P1:42", 5), "R1:42")
        self.assertIsNone(self.bot.handle("P0:42", 5))


class SimulatedLink(unittest.TestCase):
    def test_ping_and_broadcast(self):
        async def run():
            link = links.SimLink(latency_ms=10, jitter_ms=0, seed=0)
            await link.start()
            rtts = [await links.Pinger(link).ping(b) for b in range(3)]
            await link.send_line("C1:156")
            await asyncio.sleep(0.05)
            actions = [b.action for b in link.bots]
            await asyncio.sleep(P.CMD_TIMEOUT_MS / 1000 + 0.1)   # no more commands
            after = [b.action for b in link.bots]
            await link.stop()
            return rtts, actions, after
        rtts, actions, after = asyncio.run(run())
        for r in rtts:          # 2 x 10 ms, plus up to ~16 ms per hop of Windows timer granularity
            self.assertIsNotNone(r)
            self.assertTrue(0.019 <= r <= 0.060, r)
        self.assertEqual(actions, [1, 5, 6])
        self.assertEqual(after, [0, 0, 0])


class FakeBotLink:
    """Stands in for laptop_hub.BotLink."""
    def __init__(self, index, name, connect_failures=0):
        self.index, self.name, self.connected = index, name, True
        self.sent, self.connect_failures = [], connect_failures

    async def connect(self):
        if self.connect_failures:
            self.connect_failures -= 1
            raise OSError("simulated connect failure")
        self.connected = True

    async def send_line(self, text):
        if self.connected:
            self.sent.append(text)
        return self.connected


class FakeHub:
    """Stands in for laptop_hub.Hub; connects bots in a scrambled order."""
    def __init__(self, on_bot_disconnect, **_kw):
        self.on_bot_disconnect = on_bot_disconnect
        self.bots = [FakeBotLink(1, "BBC micro:bit [zavog]", connect_failures=2),
                     FakeBotLink(2, "BBC micro:bit [tuzov]"),
                     FakeBotLink(3, "BBC micro:bit [gatep]")]

    async def start(self): ...
    async def stop(self): ...

    async def send(self, _target, text):
        return sum([await b.send_line(text) for b in self.bots])


class BleDirect(unittest.TestCase):
    NAMES = ["tuzov", "gatep", "zavog"]   # robot 0, 1, 2

    def test_ids_follow_names_not_connection_order(self):
        async def run():
            link = links.BleDirectLink(self.NAMES, hub_cls=FakeHub)
            await link.start()
            return {b.name[-6:-1]: b.sent for b in link.hub.bots}
        sent = asyncio.run(run())
        self.assertEqual(sent, {"zavog": ["I2"], "tuzov": ["I0"], "gatep": ["I1"]})

    def test_reconnect_reassigns_id(self):
        async def run():
            link = links.BleDirectLink(self.NAMES, hub_cls=FakeHub, retry_s=0.01)
            await link.start()
            zavog = link.hub.bots[0]
            zavog.connected = False                       # link drops
            link.hub.on_bot_disconnect(zavog.index)
            await asyncio.sleep(0.2)                      # 2 failed tries, then success
            await link.stop()
            return zavog
        zavog = asyncio.run(run())
        self.assertTrue(zavog.connected)
        self.assertEqual(zavog.sent, ["I2", "I2", "S"])

    def test_bad_names_rejected(self):
        for names in (["a", "b"], ["a", "a", "b"], None):
            with self.assertRaises(ValueError):
                links.BleDirectLink(names, hub_cls=FakeHub)


CONTROLLER = os.environ.get("CUTEBOT_CONTROLLER_DIR")


@unittest.skipUnless(CONTROLLER, "set CUTEBOT_CONTROLLER_DIR to run")
class CameraHostIntegration(unittest.TestCase):
    def test_dispatch_reaches_bots(self):
        sys.path.insert(0, CONTROLLER)
        from time import monotonic
        from camera_host import CameraHost        # README example layout
        host = CameraHost(grid_ft=(8, 6), goal_center_m=(0.70, 0.90), goal_radius_m=0.18,
                          camera_min_m=(-0.65, -0.65), camera_max_m=(3.10, 2.50),
                          assignments=(2, 0, 1))
        t = monotonic()
        packets = host.dispatch(robot_xy_m=[(0.30, 0.40), (1.60, 0.35), (2.00, 1.30)],
                                heading_rad=[0.0, 1.57, 3.14],
                                object_xy_m=[(0.95, 0.45), (1.45, 0.85), (1.85, 1.20)],
                                captured_at=t, host_now=monotonic())
        line = P.encode_host_packets(packets)
        bots = [P.BotReceiver(bot_id=i) for i in range(3)]
        for b in bots:
            b.handle(line, 0)
        self.assertEqual([b.action for b in bots], [p["action"] for p in sorted(packets, key=lambda p: p["robot_id"])])
        stale = host.dispatch(robot_xy_m=[(0.3, 0.4)] * 3, heading_rad=[0, 0, 0],
                              object_xy_m=[(1, 1)] * 3, captured_at=t, host_now=monotonic())
        self.assertTrue(P.encode_host_packets(stale).endswith(":xxx"))


if __name__ == "__main__":
    unittest.main()
