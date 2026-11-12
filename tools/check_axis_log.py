#!/usr/bin/env python3
"""Inspect a fake-robot JSONL log and assert the raw wire axis mapping.

The SDK packs API axes [forward, strafe, yaw, head] as
wire [-strafe, -forward, -yaw, -head]. Therefore:
  forward command -> wire index 1 negative
  left strafe     -> wire index 0 negative
  left turn       -> wire index 2 negative
"""

from __future__ import annotations

import argparse
import json
import sys


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", default="/tmp/agent_fake.jsonl")
    args = parser.parse_args()

    vectors: set[tuple[float, float, float, float]] = set()
    with open(args.log, encoding="utf-8") as handle:
        for line in handle:
            try:
                packet = json.loads(line).get("packet", {})
            except ValueError:
                continue
            if packet.get("type") != "remote":
                continue
            joystick = packet.get("joystick") or []
            if len(joystick) >= 4 and any(abs(float(v)) > 0.01 for v in joystick):
                vectors.add(tuple(round(float(v), 3) for v in joystick[:4]))

    checks = {
        "forward (wire[1] nonzero)": any(
            abs(v[1]) > 0.01 and abs(v[0]) < 0.01 and abs(v[2]) < 0.01
            for v in vectors
        ),
        "left strafe (wire[0] negative)": any(
            v[0] < -0.01 and abs(v[1]) < 0.01 and abs(v[2]) < 0.01
            for v in vectors
        ),
        "left turn (wire[2] negative)": any(
            v[2] < -0.01 and abs(v[0]) < 0.01 and abs(v[1]) < 0.01
            for v in vectors
        ),
    }
    failed = False
    for name, ok in checks.items():
        print(("PASS  " if ok else "FAIL  ") + name)
        failed = failed or not ok
    print("nonzero wire vectors:", sorted(vectors)[:12])
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
