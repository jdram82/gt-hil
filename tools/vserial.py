#!/usr/bin/env python3
"""
vserial.py - a virtual null-modem cable made of two pseudo-terminals.

    python3 tools/vserial.py                 # prints two device paths, e.g.
        /dev/pts/3 /dev/pts/4
    plant_node --port /dev/pts/3 ...         # one end
    ctrl_node  --port /dev/pts/4 ...         # the other end

Everything written to one end appears at the other, so plant_node and
ctrl_node (or the STM32 code, via ctrl_node) can be tested without wiring.
With --state FILE the two device paths are also written to FILE.
"""
import argparse
import os
import select
import sys
import tty


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--state", help="write the two slave paths to this file")
    args = ap.parse_args()

    ends = []
    for _ in range(2):
        master, slave = os.openpty()
        tty.setraw(slave)
        ends.append((master, os.ttyname(slave), slave))   # keep the slave open so the pty stays alive
    line = f"{ends[0][1]} {ends[1][1]}"
    print(line, flush=True)
    if args.state:
        with open(args.state, "w") as f:
            f.write(line + "\n")

    a, b = ends[0][0], ends[1][0]
    while True:
        r, _, _ = select.select([a, b], [], [])
        for fd in r:
            try:
                data = os.read(fd, 4096)
            except OSError:
                continue
            if data:
                os.write(b if fd == a else a, data)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
