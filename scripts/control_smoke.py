#!/usr/bin/env python3
"""Coordinator-only neutral-output Pico control smoke test.

Requires pyserial on the host. Never flashes, uploads ESC firmware, or sends
nonneutral raw motor commands. The default temporary configuration has ZERO
allocation, ZERO power, and no nullspace vectors. Opt-in nullspace stress adds
eight dense vectors and CAN produce nonneutral calculated outputs; it requires
--escs-disconnected. Settings are RAM-only and replaced by the Pi's full
configuration at its next negotiated session. --backpressure pauses host reads
for four seconds while sending fresh commands and retains the enclosing device
statistics window; it requires an increased device USB-drop count as evidence.
"""

import argparse
import json
import math
import re
import secrets
import struct
import time
from collections import Counter, deque


def crc32c(data):
    value = 0xFFFFFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0x82F63B78 if value & 1 else 0)
    return value ^ 0xFFFFFFFF


def frame(kind, session, sequence, payload=b""):
    header = struct.pack("<BBBBHII", 0xF0, 1, kind, 0, len(payload), session, sequence)
    body = header + payload
    return body + struct.pack("<I", crc32c(body))


def legacy(kind, payload):
    result = bytes([kind]) + payload
    checksum = 0
    for byte in result:
        checksum ^= byte
    return result + bytes([checksum])


def neutral_settings(stress_nullspace=False):
    data = bytearray(628)
    for offset in (0, 16, 32):
        struct.pack_into("<4f", data, offset, 6, 2, 0.6, 120)
    struct.pack_into("<4f", data, 48, 2, 0, 0.5, 0.5)
    struct.pack_into("<3f", data, 80, 1, 1, 1)
    data[348:356] = bytes(range(8))
    data[356:364] = bytes([1] * 8)
    if stress_nullspace:
        struct.pack_into("<I", data, 364, 8)
        for row in range(8):
            for column in range(8):
                sign = -1.0 if (row & column).bit_count() % 2 else 1.0
                struct.pack_into("<f", data, 368 + (row * 8 + column) * 4, sign)
    struct.pack_into("<HH", data, 624, 1, 300)
    return bytes(data)


class Link:
    def __init__(self, port, stress_nullspace=False):
        self.port = port
        self.stress_nullspace = stress_nullspace
        self.buffer = bytearray()
        self.messages = deque(maxlen=256)
        self.session = secrets.randbits(32) or 1
        self.sequence = 0
        self.counts = Counter()
        self.last_attitude = None
        self.stats = []
        self.usb_drops = None

    def pump(self):
        self.buffer.extend(self.port.read(max(1, self.port.in_waiting)))
        while self.buffer:
            kind = self.buffer[0]
            if kind == 0xF0:
                if len(self.buffer) < 14:
                    return
                _, version, message, flags, size, session, sequence = struct.unpack_from(
                    "<BBBBHII", self.buffer
                )
                if version != 1 or flags or size > 768:
                    del self.buffer[0]
                    continue
                total = size + 18
                if len(self.buffer) < total:
                    return
                packet = bytes(self.buffer[:total])
                del self.buffer[:total]
                if crc32c(packet[:-4]) != struct.unpack_from("<I", packet, total - 4)[0]:
                    self.counts["bad_crc"] += 1
                    continue
                payload = packet[14:-4]
                self.counts[hex(message)] += 1
                self.messages.append((message, session, sequence, payload))
                self.observe(message, payload)
            elif kind in (0xA5, 0xD5, 0xE9):
                size = {0xA5: 8, 0xD5: 8, 0xE9: 12}[kind]
                if len(self.buffer) < size:
                    return
                del self.buffer[:size]
            elif kind in (0xB5, 0xD6, 0xA6):
                if len(self.buffer) < 3:
                    return
                size = self.buffer[1] * 6 + 3 if kind == 0xA6 else self.buffer[2] + 4
                if len(self.buffer) < size:
                    return
                packet = bytes(self.buffer[:size])
                del self.buffer[:size]
                checksum = 0
                for byte in packet:
                    checksum ^= byte
                if checksum:
                    self.counts["bad_legacy_checksum"] += 1
                    continue
                if kind == 0xB5:
                    message = packet[3:-1].decode("utf-8", errors="replace")
                    drops = re.search(r"USB_drops=(\d+)", message)
                    if drops is not None:
                        self.usb_drops = int(drops.group(1))
                    print("MCU", message, flush=True)
            else:
                del self.buffer[0]
        while len(self.messages) > 256:
            self.messages.popleft()

    def observe(self, kind, payload):
        if kind == 0x90:
            if len(payload) != 84:
                raise RuntimeError(f"ATTITUDE size {len(payload)} != 84")
            current = struct.unpack_from("<4f", payload, 8)
            desired = struct.unpack_from("<4f", payload, 24)
            for quaternion in (current, desired):
                if not all(math.isfinite(v) for v in quaternion):
                    raise RuntimeError("nonfinite quaternion")
                if abs(sum(v * v for v in quaternion) - 1) > 0.001:
                    raise RuntimeError(f"unnormalized quaternion {quaternion}")
            motors = struct.unpack_from("<8H", payload, 64)
            if any(value > 2000 for value in motors):
                raise RuntimeError(f"OUT OF RANGE MOTOR COMMANDS: {motors}")
            if not self.stress_nullspace and motors != (1000,) * 8:
                raise RuntimeError(f"UNEXPECTED NONNEUTRAL MOTOR COMMANDS: {motors}")
            self.last_attitude = {
                "current": current,
                "desired": desired,
                "desired_depth": struct.unpack_from("<f", payload, 40)[0],
                "generation": struct.unpack_from("<I", payload, 44)[0],
                "health": struct.unpack_from("<I", payload, 52)[0],
                "host_age_us": struct.unpack_from("<I", payload, 56)[0],
                "output_age_us": struct.unpack_from("<I", payload, 60)[0],
            }
        elif kind == 0x92:
            if len(payload) != 28 or not all(math.isfinite(v) for v in struct.unpack("<7f", payload)):
                raise RuntimeError("invalid raw IMU telemetry")
        elif kind == 0x91:
            values = struct.unpack("<Q10I", payload)
            elapsed = values[0] / 1_000_000
            report = {"received_host_monotonic_s": time.monotonic(),
                      "elapsed_s": elapsed, "ahrs_hz": values[1] / elapsed,
                      "pid_hz": values[2] / elapsed, "depth_hz": values[3] / elapsed,
                      "misses": values[4], "avg_us": values[5], "max_us": values[6],
                      "sensor_errors": values[7], "queue_overflows": values[8],
                      "max_host_age_us": values[9], "max_output_age_us": values[10]}
            self.stats.append(report)
            print("STATS", json.dumps(report), flush=True)

    def send(self, kind, payload=b"", reliable=False, expected=0):
        self.sequence += 1
        sequence = self.sequence
        packet = frame(kind, self.session, sequence, payload)
        self.port.write(packet)
        if not reliable:
            return None
        deadline = time.monotonic() + 8
        next_retry = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            self.pump()
            for message in tuple(self.messages):
                msg_kind, session, seq, reply = message
                if msg_kind == 0x80 and session == self.session and seq == sequence:
                    self.messages.remove(message)
                    request, result, reserved, generation, digest = struct.unpack("<BBHII", reply)
                    if request != kind or result != expected or reserved:
                        raise RuntimeError(f"request {kind:#x}: result {result}, expected {expected}")
                    return generation, digest
            if time.monotonic() >= next_retry:
                self.port.write(packet)
                next_retry += 0.5
        raise TimeoutError(f"request {kind:#x} outcome unknown")

    def settings(self, generation, data, result=0):
        digest = crc32c(data)
        self.send(0x20, struct.pack("<III", generation, len(data), digest), True, 1)
        for offset in range(0, len(data), 128):
            self.send(0x21, struct.pack("<I", offset) + data[offset:offset + 128], True, 1)
        return self.send(0x22, struct.pack("<II", generation, digest), True, result)

    def drain(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.pump()


def stream_window(link, seconds, read=True, stop=None):
    """Keep real commands fresh, optionally without draining any host serial input."""
    next_control = next_pressure = time.monotonic()
    deadline = next_control + seconds
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_pressure:
            link.send(0x11, struct.pack("<ffI", 0.2, 0, 1))
            next_pressure += 1 / 15
        if now >= next_control:
            direction = [0, 0, 0, 0, 0.03, 0, 0, 0]
            link.send(0x10, struct.pack("<9fI", *direction, 1 / 60, 7))
            next_control += 1 / 60
        if read:
            link.pump()
        else:
            time.sleep(0.001)
        if stop is not None and stop():
            return


def test_backpressure(link):
    """Retain a complete five-second device window containing four seconds blocked."""
    previous = len(link.stats)
    stream_window(link, 7, stop=lambda: len(link.stats) > previous and link.usb_drops is not None)
    if len(link.stats) == previous or link.usb_drops is None:
        raise RuntimeError("Cannot align backpressure to measured device statistics")
    boundary = len(link.stats)
    before = link.usb_drops
    started = time.monotonic()
    print("BACKPRESSURE_BEGIN", json.dumps({"host_monotonic_s": started,
                                           "usb_drops_before": before}), flush=True)
    # A synchronous pyserial port has no userspace background reader. Kernel and
    # device buffers must actually fill; an increased device drop count proves it.
    stream_window(link, 4, read=False)
    blocked_seconds = time.monotonic() - started
    stream_window(link, 7, stop=lambda: len(link.stats) > boundary and link.usb_drops > before)
    if len(link.stats) <= boundary or link.usb_drops <= before:
        raise RuntimeError("No retained device timing window or no proven USB backpressure")
    measured = link.stats[boundary]
    if measured["received_host_monotonic_s"] - started > 6:
        raise RuntimeError("Blocked statistics window was lost; cannot claim its timing")
    if not (495 <= measured["ahrs_hz"] <= 505 and 495 <= measured["pid_hz"] <= 505):
        raise RuntimeError(f"500 Hz control failed under USB backpressure: {measured}")
    if measured["misses"] or measured["queue_overflows"] or measured["max_us"] >= 2000:
        raise RuntimeError(f"Control deadline or queue failed under backpressure: {measured}")
    result = {"blocked_host_read_seconds": blocked_seconds,
              "usb_drops_before": before, "usb_drops_after": link.usb_drops,
              "device_window": measured}
    print("BACKPRESSURE_RESULT", json.dumps(result), flush=True)
    return result


def run(args):
    import serial  # Host runtime already supplies pyserial; not a build dependency.

    with serial.Serial(args.port, 115200, timeout=0.005, write_timeout=0.5, exclusive=True) as port:
        link = Link(port, args.stress_nullspace)
        if args.stress_nullspace:
            print("WARNING: dense nullspace stress can produce nonneutral calculated outputs; ESCs MUST be disconnected.", flush=True)
        port.write(legacy(0x5A, struct.pack("<8H", *([1000] * 8))))
        request = legacy(0xC5, struct.pack("<BBBH", 3, 1, 0, 0))
        port.write(request)
        deadline = time.monotonic() + 3
        capabilities = None
        while time.monotonic() < deadline and capabilities is None:
            link.pump()
            for message in tuple(link.messages):
                if message[:3] == (0x81, 0, 1):
                    capabilities = message[3]
                    break
        if capabilities is None:
            raise RuntimeError("No control capability. No extended messages sent to old firmware.")
        schema, maximum, size, vectors, features = struct.unpack_from("<4HI", capabilities)
        identity = capabilities[12:].decode("ascii")
        if schema != 1 or maximum < 628 or size != 628 or vectors != 8:
            raise RuntimeError("Unsupported capability layout")
        print("CAPABILITIES", schema, maximum, size, vectors, hex(features), identity, flush=True)
        link.send(0x01, reliable=True)
        data = neutral_settings(args.stress_nullspace)
        assert link.settings(1, data) == (1, crc32c(data))
        invalid = bytearray(data)
        struct.pack_into("<f", invalid, 0, float("nan"))
        link.settings(2, invalid, result=2)
        assert link.send(0x24, reliable=True) == (1, crc32c(data))
        link.send(0x23, reliable=True)
        link.send(0x12, struct.pack("<4f", 0, 0, math.sin(0.1), math.cos(0.1)), True)
        link.send(0x13, struct.pack("<f", 0.3), True)
        next_control = next_pressure = time.monotonic()
        end = next_control + args.seconds
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_pressure:
                # Synthetic fixed depth validates transport/PID cadence only.
                link.send(0x11, struct.pack("<ffI", 0.2, 0, 1))
                next_pressure += 1 / 15
            if now >= next_control:
                direction = [0, 0, 0, 0, 0.03, 0, 0, 0]
                link.send(0x10, struct.pack("<9fI", *direction, 1 / 60, 7))
                next_control += 1 / 60
            link.pump()
        backpressure = test_backpressure(link) if args.backpressure else None
        # Prove host expiry even though sample/telemetry continue.
        link.drain(0.35)
        if not link.last_attitude or link.last_attitude["health"] & 4:
            raise RuntimeError("Output authority did not expire after host silence")
        if not link.last_attitude["health"] & 1:
            raise RuntimeError("IMU is not healthy; inspect MCU logs and wiring")
        if not link.stats:
            raise RuntimeError("No measured execution statistics received")
        print("RESULT", json.dumps({"frames": link.counts, "last": link.last_attitude,
                                    "stats": link.stats, "backpressure": backpressure}), flush=True)
        link.send(0x14, struct.pack("<8H", *([1000] * 8)))
        link.drain(0.1)
        link.send(0x25, reliable=True)
        print("Left in sticky neutral maintenance. Pi must HELLO + full settings + fresh CONTROL.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--seconds", type=float, default=12)
    parser.add_argument("--execute", action="store_true", help="acknowledge coordinator-only serial I/O")
    parser.add_argument("--backpressure", action="store_true", help="prove control timing while host serial reads are paused for four seconds")
    parser.add_argument("--stress-nullspace", action="store_true", help="exercise eight dense nullspace vectors; computed outputs may be nonneutral")
    parser.add_argument("--escs-disconnected", action="store_true", help="explicit acknowledgement required for nullspace stress")
    options = parser.parse_args()
    if not options.execute:
        parser.error("No hardware access without --execute; stop Pi service and disconnect ESCs first")
    if options.stress_nullspace and not options.escs_disconnected:
        parser.error("--stress-nullspace requires --escs-disconnected; computed motor outputs will not remain neutral")
    if options.seconds < 6:
        parser.error("--seconds must be >=6 for one measured five-second statistics window")
    run(options)
