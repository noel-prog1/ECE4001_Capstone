/**
 * cutebot_ble.ts - Cutebot firmware, direct BLE to the laptop (laptop_hub.py)
 *
 * Extends the team's microbit_comms.ts skeleton with motor control, the
 * shared protocol (comms/protocol.py) and a stop-on-timeout watchdog.
 * All three cars run the SAME hex.
 *
 * MakeCode (https://makecode.microbit.org):
 *   1. micro:bit V2 required (V1 runs out of memory with Bluetooth).
 *   2. Extensions -> add "bluetooth" and "Cutebot" (ELECFREAKS); REMOVE "radio".
 *   3. Project Settings (gear) -> Bluetooth Pairing -> "No Pairing Required".
 *   4. JavaScript view -> paste this file -> Download.
 *
 * Robot ID: the laptop sends "I0".."I2" after connecting, based on this
 * micro:bit's 5-letter name (button B scrolls it - label each car with it).
 * Until it has an ID the car ignores every command, so it can never drive
 * with the wrong ID after a reboot or reconnect.
 *
 * LEDs: "?" = connected, waiting for ID; digit = assigned ID;
 *       X = disconnected; top-right pixel blinks on every accepted command.
 */

// ---------------- settings ----------------
const CMD_TIMEOUT_MS = 300     // stop if no fresh command for this long
const SESSION_RESET_MS = 1000  // after this much silence any seq starts a new session

// Motor calibration, percent (-100..100). Tune on the real grid so that
//   FWD  -> ~0.20 m/s  (drives ~40 cm in 2 s)
//   SPIN -> ~3 rad/s   (one full turn in ~2.1 s)
// The simulator the policy was trained in uses exactly those speeds.
const FWD = 40
const SPIN = 30
const REV = -26                // simulator reverses at 0.65 x forward speed

// ---------------- state ----------------
let BOT_ID = -1                // -1 = not assigned yet
let action = 0
let lastSeq = -1
let lastCmdMs = -100000

// ---------------- motors ----------------
function drive(a: number) {
    action = a
    if (a == 1) cuteBot.motors(FWD, FWD)
    else if (a == 2) cuteBot.motors(-SPIN, SPIN)               // rotate left (CCW)
    else if (a == 3) cuteBot.motors(SPIN, -SPIN)               // rotate right
    else if (a == 4) cuteBot.motors(FWD - SPIN, FWD + SPIN)    // forward-left arc
    else if (a == 5) cuteBot.motors(FWD + SPIN, FWD - SPIN)    // forward-right arc
    else if (a == 6) cuteBot.motors(REV, REV)
    else { action = 0; cuteBot.stopcar() }
}

// ---------------- protocol (mirrors protocol.BotReceiver) ----------------
function isDigits(s: string): boolean {
    if (s.length == 0 || s.length > 5) return false
    for (let i = 0; i < s.length; i++) {
        const c = s.charCodeAt(i)
        if (c < 48 || c > 57) return false
    }
    return true
}

// Returns a reply line ("" for none).
function handleLine(line: string): string {
    if (line == "S") { drive(0); return "" }
    if (line.length == 2 && line.charAt(0) == "I") {          // ID assignment
        const id = line.charCodeAt(1) - 48
        if (id >= 0 && id <= 2) {
            drive(0)
            BOT_ID = id
            basic.showNumber(id)
        }
        return ""
    }
    if (BOT_ID < 0) return ""                                  // unassigned: never move

    const kind = line.charAt(0)
    const colon = line.indexOf(":")
    if (colon < 2) return ""
    const head = line.substr(1, colon - 1)
    const tail = line.substr(colon + 1, line.length - colon - 1)
    if (!isDigits(head)) return ""
    const num = parseInt(head)

    if (kind == "P") {                                          // ping
        if (num == BOT_ID && isDigits(tail)) return "R" + BOT_ID + ":" + tail
        return ""
    }
    if (kind != "C" || tail.length != 3 || num > 65535) return ""
    for (let i = 0; i < 3; i++) {                               // validate all slots
        const c = tail.charCodeAt(i)
        if (!((c >= 48 && c <= 54) || c == 120)) return ""      // '0'..'6' or 'x'
    }
    const now = control.millis()
    if (now - lastCmdMs <= SESSION_RESET_MS) {                  // same session: newer only
        const d = (num - lastSeq + 65536) % 65536
        if (d == 0 || d >= 32768) return ""
    }
    lastSeq = num
    lastCmdMs = now
    const c = tail.charCodeAt(BOT_ID)
    const a = c == 120 ? 0 : c - 48                             // 'x' = tracking invalid -> stop
    if (a != action) drive(a)
    led.toggle(4, 0)
    return ""
}

function stripEnd(s: string): string {
    while (s.length > 0 && (s.charAt(s.length - 1) == "\n" || s.charAt(s.length - 1) == "\r")) {
        s = s.substr(0, s.length - 1)
    }
    return s
}

// ---------------- bluetooth ----------------
bluetooth.onBluetoothConnected(function () {
    basic.showString("?")                      // waiting for the laptop to send I<id>
})

bluetooth.onBluetoothDisconnected(function () {
    drive(0)                                   // link lost -> stop immediately
    BOT_ID = -1                                // must be re-assigned on reconnect
    basic.showIcon(IconNames.No)
})

bluetooth.onUartDataReceived(serial.delimiters(Delimiters.NewLine), function () {
    const reply = handleLine(stripEnd(bluetooth.uartReadUntil(serial.delimiters(Delimiters.NewLine))))
    if (reply.length > 0) bluetooth.uartWriteString(reply + "\n")
})

// ---------------- watchdog ----------------
basic.forever(function () {
    if (action != 0 && control.millis() - lastCmdMs > CMD_TIMEOUT_MS) drive(0)
    basic.pause(20)
})

// Button B: scroll this micro:bit's name (what to pass to --names on the laptop).
input.onButtonPressed(Button.B, function () {
    basic.showString(control.deviceName())
})

// ---------------- startup ----------------
bluetooth.startUartService()
drive(0)
basic.showIcon(IconNames.SmallDiamond)         // advertising, not connected
