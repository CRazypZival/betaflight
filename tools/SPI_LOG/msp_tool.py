#!/usr/bin/env python3
"""Read ASCENT diagnostics or export dataflash through standard Betaflight MSP."""
import argparse
import functools
import json
import operator
import pathlib
import struct
import time

import serial


class MSP:
    def __init__(self, port):
        self.serial = serial.Serial(port, 115200, timeout=0.1)

    def request(self, command, payload=b"", attempts=4):
        body = bytes((len(payload), command)) + payload
        request = b"$M<" + body + bytes((functools.reduce(operator.xor, body),))
        for attempt in range(attempts):
            self.serial.reset_input_buffer()
            self.serial.write(request)
            deadline = time.monotonic() + 3
            data = bytearray()
            while time.monotonic() < deadline:
                data.extend(self.serial.read(self.serial.in_waiting or 1))
                start = data.find(b"$M")
                if start < 0 or len(data) < start + 6:
                    continue
                length, cmd = data[start + 3:start + 5]
                end = start + 6 + length
                if len(data) < end:
                    continue
                frame = data[start:end]
                del data[:end]
                if functools.reduce(operator.xor, frame[3:]) != 0:
                    break  # corrupted frame, retry
                if cmd != command:
                    continue
                if frame[2] != ord(">"):
                    raise RuntimeError(f"MSP command {command} rejected")
                return bytes(frame[5:-1])
        raise RuntimeError(f"MSP command {command} failed after {attempts} attempts")

    def summary(self):
        flags, sectors, capacity, used = struct.unpack("<BIII", self.request(70))
        return dict(ready=bool(flags & 1), supported=bool(flags & 2), sectors=sectors,
                    capacity=capacity, used=used)

    def read(self, address, length):
        reply = self.request(71, struct.pack("<IHB", address, length, 0))
        offset, count, compression = struct.unpack_from("<IHB", reply)
        if offset != address or compression or count != len(reply) - 7 or count != length:
            raise RuntimeError(f"Short/invalid read at {address}: {reply[:7].hex()}")
        return reply[7:]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/cu.usbmodem0x80000001")
    parser.add_argument("--export", type=pathlib.Path)
    parser.add_argument("--length", type=int)
    parser.add_argument("--address", type=int, default=0)
    args = parser.parse_args()
    resume = 0
    if args.export:
        args.export.parent.mkdir(parents=True, exist_ok=True)
    if args.export and args.export.exists():
        # resume after the last complete 128-byte block
        resume = args.address + (args.export.stat().st_size // 128) * 128
        with args.export.open("r+b") as stream:
            stream.truncate(resume - args.address)
        print(f"resuming at {resume}", flush=True)

    length = None
    while True:
        try:
            msp = MSP(args.port)
            summary = msp.summary()
            print(json.dumps(summary), flush=True)
            print("debug:", struct.unpack("<8h", msp.request(254)), flush=True)
            if length is None:
                length = args.length if args.length is not None else min(summary["used"], summary["capacity"]) - args.address
                if not summary["ready"] or length <= 0 or args.address < 0 or args.address + length > summary["capacity"]:
                    raise RuntimeError("Invalid export range or flash not ready")
            with args.export.open("ab") as stream:
                for position in range(resume, args.address + length, 128):
                    stream.write(msp.read(position, min(128, args.address + length - position)))
                    stream.flush()
                    if position % 6400 < 128:
                        print(f"{position}/{length}", flush=True)
            print(f"Exported {length} bytes: {args.export}", flush=True)
            msp.serial.close()
            break
        except (serial.SerialException, RuntimeError, TimeoutError, OSError) as error:
            print(f"interrupted: {error}", flush=True)
            try:
                msp.serial.close()
            except Exception:
                pass
            resume = args.address + (args.export.stat().st_size // 128) * 128
            with args.export.open("r+b") as stream:
                stream.truncate(resume - args.address)
            print(f"will resume at {resume}; waiting for port...", flush=True)
            while True:
                try:
                    probe = serial.Serial(args.port, 115200, timeout=0.2)
                    probe.close()
                    break
                except serial.SerialException:
                    time.sleep(5)
            time.sleep(2)


if __name__ == "__main__":
    main()
