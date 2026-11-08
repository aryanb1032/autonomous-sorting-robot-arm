#!/usr/bin/env python3
"""Explicit one-channel 90→92→90 trial; all outputs disabled on completion."""
import argparse
import time

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--channel', type=int, required=True, choices=[0, 1, 2, 5, 6, 7])
p.add_argument('--port')
p.add_argument('--driver', choices=['direct','adafruit'], default='direct')
p.add_argument('--execute', action='store_true')
p.add_argument('--confirm-supported-home', action='store_true')
a = p.parse_args()
print(f'Only channel {a.channel}: 90 → 92 → 90; all others disabled.')
print('All outputs disable afterward. Support the entire arm throughout.')
if not a.execute:
    raise SystemExit('PREVIEW ONLY; no connection or motion.')
if not a.port or not a.confirm_supported_home:
    raise SystemExit('Provide --port and --confirm-supported-home.')
import serial
with serial.Serial(port=None, baudrate=115200, timeout=.1, write_timeout=1, exclusive=True) as s:
    s.dtr = False
    s.rts = False
    s.port = a.port
    s.open()
    time.sleep(2)
    s.reset_input_buffer()
    def send(text):
        s.write((text+'\n').encode())
    send('INFO')
    deadline = time.monotonic()+3
    while time.monotonic() < deadline:
        if s.readline().strip() == (b'CONFIG SINGLE_CHANNEL_ADAFRUIT_V1' if a.driver=='adafruit' else b'CONFIG SINGLE_CHANNEL_V2'):
            break
    else:
        raise SystemExit('Wrong/unresponsive firmware; RUN not sent.')
    try:
        send(f'RUN {a.channel}')
        deadline = time.monotonic()+9
        last = 0
        acknowledged = False
        while time.monotonic() < deadline:
            if time.monotonic()-last > .3:
                send('KEEPALIVE')
                last = time.monotonic()
            line = s.readline().decode(errors='replace').strip()
            if not line:
                continue
            print(line, flush=True)
            if line.startswith('ERR'):
                raise RuntimeError(line)
            if line == f'ACK CHANNEL {a.channel}':
                acknowledged = True
            if line == 'DONE ESTIMATED DISABLED' and acknowledged:
                break
        else:
            raise TimeoutError('Trial completion not confirmed.')
    finally:
        send('STOP')
