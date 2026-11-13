#!/usr/bin/env python3
"""Run one owned process group; bound shutdown when its signal handler hangs.

This supervises console exit, not robot motion. Stop gait normally before ending E.
It never searches for, signals, or kills other sessions' Gazebo / ROS processes.
"""
import argparse
import os
import signal
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command
    if command[:1] == ['--']:
        command = command[1:]
    if not command:
        parser.error('Provide a command after --')
    pending = []

    def requested(number, frame):
        if not pending:
            pending.append(number)

    signal.signal(signal.SIGINT, requested)
    signal.signal(signal.SIGTERM, requested)
    child = subprocess.Popen(command, start_new_session=True)
    print(f'[guard] started PID={child.pid}', flush=True)
    stop = None
    stage = 0

    def send(number):
        try:
            os.killpg(child.pid, number)
        except ProcessLookupError:
            pass

    while child.poll() is None:
        now = time.monotonic()
        if pending and stop is None:
            stop = now
            send(pending[0])
            print('[guard] exit requested; waiting up to 5 seconds', flush=True)
        if stop is not None:
            elapsed = now-stop
            if elapsed >= 5 and stage < 1:
                stage = 1
                send(signal.SIGTERM)
                print('[guard] still running; sent SIGTERM to owned group', flush=True)
            if elapsed >= 8 and stage < 2:
                stage = 2
                send(signal.SIGKILL)
                print('[guard] still running; sent SIGKILL to owned group', flush=True)
            if elapsed >= 10:
                print(f'[guard] PID {child.pid} has not exited; inspect kernel wait state',
                      file=sys.stderr, flush=True)
                return 137
        time.sleep(0.05)
    print(f'[guard] child exited: {child.returncode}', flush=True)
    if pending:
        return 128+pending[0]
    return child.returncode if child.returncode >= 0 else 128-child.returncode


if __name__ == '__main__':
    raise SystemExit(main())
