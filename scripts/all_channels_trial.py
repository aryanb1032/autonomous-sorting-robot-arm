#!/usr/bin/env python3
"""Sequential 5-degree test; support all links because unselected motors are disabled."""
import argparse
import time

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--port')
p.add_argument('--execute',action='store_true')
p.add_argument('--confirm-supported-home',action='store_true')
a=p.parse_args()
channels=[(0,'base'),(1,'shoulder'),(2,'elbow'),(5,'wrist pitch'),(6,'wrist rotation'),(7,'gripper')]
for ch,name in channels:
    lo,hi=(1000,2300) if ch==7 else (500,2500)
    print(f'{name}, channel {ch}: 90 → 95 → 90; {lo+(hi-lo)*90/180:.1f} → {lo+(hi-lo)*95/180:.1f} → {lo+(hi-lo)*90/180:.1f} us',flush=True)
if not a.execute:
    raise SystemExit('PREVIEW ONLY; no serial connection or movement.')
if not a.port or not a.confirm_supported_home:
    raise SystemExit('Actual port and supported home confirmation required.')
import serial
s=serial.Serial(port=None,baudrate=115200,timeout=.1,write_timeout=1,exclusive=True)
s.dtr=False;s.rts=False;s.port=a.port
with s:
    time.sleep(2);s.reset_input_buffer()
    def send(cmd):
        s.write((cmd+'\n').encode());s.flush()
    send('INFO')
    deadline=time.monotonic()+3
    while time.monotonic()<deadline:
        if s.readline().strip()==b'CONFIG CHANNEL_SWEEP_5DEG_V1':break
    else:raise SystemExit('Wrong/unresponsive firmware: no RUN sent.')
    try:
        for ch,name in channels:
            print(f'START {name} channel {ch}',flush=True)
            send(f'RUN {ch}')
            deadline=time.monotonic()+15;last=0;ack=False
            while time.monotonic()<deadline:
                if time.monotonic()-last>.3:
                    send('KEEPALIVE');last=time.monotonic()
                line=s.readline().decode(errors='replace').strip()
                if not line:continue
                print(line,flush=True)
                if line.startswith('ERR'):raise RuntimeError(line)
                if line==f'ACK CHANNEL {ch}':ack=True
                if line=='DONE ESTIMATED DISABLED' and ack:break
            else:raise TimeoutError(f'No completion for channel {ch}')
            time.sleep(1)
    finally:
        send('STOP')
