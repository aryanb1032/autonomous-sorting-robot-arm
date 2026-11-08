#!/usr/bin/env python3
"""Preview by default; execute one 90→92→90-degree motor trial only after verified preparation."""
import argparse,json,math,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--channel',type=int,required=True,choices=[0,1,2,5,6,7]);p.add_argument('--port');p.add_argument('--execute',action='store_true');p.add_argument('--confirm-supported-home',action='store_true');a=p.parse_args()
cfg=json.loads((ROOT/'hardware/motor_trial.json').read_text());i=cfg['channels'].index(a.channel)
lo,hi=cfg['pulse_ranges_us'][i]
print(f"Channel {a.channel}: {cfg['roles'][i]}; servo degrees, NOT robot joint angles")
for deg in [90,92,90]:print(f'{deg} degrees -> {lo+(hi-lo)*deg/180:.1f} microseconds (nominal)')
if not a.execute:raise SystemExit('PREVIEW ONLY: no serial port opened, no motor commands sent.')
if not a.port or not a.confirm_supported_home:raise SystemExit('Provide actual port and confirm the supported 90-degree starting pose')
if not cfg['power_verified'] or not cfg['oe_wired_verified'] or not 4.8<=cfg['servo_rail_voltage']<=6.6:raise SystemExit('Power/OE prerequisites not satisfied; no port opened')
manifest=json.loads((ROOT/'hardware/motor_trial/trial_manifest.json').read_text())
if not manifest['enabled']:raise SystemExit('Trial firmware was generated locked')
import hashlib
expected='TRIAL_'+hashlib.sha256(json.dumps(cfg,sort_keys=True).encode()).hexdigest()[:16]
if manifest['config_id']!=expected:raise SystemExit('Profile changed; regenerate and upload matching trial firmware')
import serial
with serial.Serial(a.port,115200,timeout=.1,write_timeout=1,exclusive=True) as port:
    port.dtr=False;port.rts=False
    time.sleep(2);port.reset_input_buffer()
    def send(s):port.write((s+'\n').encode())
    def expect(prefix,timeout=4,heartbeat=False):
        deadline=time.monotonic()+timeout;last=0
        while time.monotonic()<deadline:
            if heartbeat and time.monotonic()-last>.3:send('KEEPALIVE');last=time.monotonic()
            line=port.readline().decode(errors='replace').strip()
            if line.startswith('ERR') or line.startswith('LOCKED'):raise RuntimeError(line)
            if line.startswith(prefix):return line
        raise TimeoutError('No expected reply: '+prefix)
    send('INFO')
    if expect('CONFIG ')!='CONFIG '+expected:raise SystemExit('Wrong firmware configuration; motor initialization not sent')
    try:
        send('INIT CONFIRM_INITIAL_POSE');expect('READY')
        # INIT energizes all six at 90 degrees. They must already be supported there.
        for deg in [92,90]:
            q=[math.pi/2]*6;q[i]=math.radians(deg)
            send('MOVE 2000 '+' '.join(f'{v:.8f}' for v in q));expect('ACK');expect('DONE ESTIMATED',heartbeat=True)
            time.sleep(.3)
        print('Commands completed; actual position has no encoder verification.')
    finally:send('STOP') # Hold last command; support the arm before removing power.
