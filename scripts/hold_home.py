#!/usr/bin/env python3
"""Hold nominal 90 degrees for a bounded photo interval. Support arm throughout."""
import argparse,time
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--port',required=True)
p.add_argument('--seconds',type=int,default=120)
p.add_argument('--execute',action='store_true')
p.add_argument('--confirm-supported-home',action='store_true')
a=p.parse_args()
if not 1<=a.seconds<=300:raise SystemExit('Duration must be 1–300 seconds.')
print('All six: nominal 90 degrees. Channels 0/1/2/5/6: 1500 us; channel 7: 1650 us.')
print(f'Outputs will disable after {a.seconds} seconds or communication loss. Support arm throughout.')
if not a.execute:raise SystemExit('PREVIEW ONLY')
if not a.confirm_supported_home:raise SystemExit('Confirm supported home pose first.')
import serial
s=serial.Serial(port=None,baudrate=115200,timeout=.1,write_timeout=1,exclusive=True)
s.dtr=False;s.rts=False;s.port=a.port
with s:
 time.sleep(2);s.reset_input_buffer()
 def send(c):s.write((c+'\n').encode());s.flush()
 send('INFO');deadline=time.monotonic()+3
 while time.monotonic()<deadline:
  if s.readline(256).strip()==b'CONFIG HOME_HOLD_V1':break
 else:raise SystemExit('Firmware not confirmed; HOME90 not sent.')
 try:
  send('HOME90');deadline=time.monotonic()+3
  while time.monotonic()<deadline:
   line=s.readline(256).decode(errors='replace').strip()
   if line.startswith('ERR'):raise RuntimeError(line)
   if line=='HOLDING NOMINAL_90 NO_POSITION_FEEDBACK':
    print(line,flush=True);break
  else:raise TimeoutError('Hold not acknowledged.')
  end=time.monotonic()+a.seconds
  while time.monotonic()<end:
   send('KEEPALIVE')
   line=s.readline(256).decode(errors='replace').strip()
   if line.startswith('ERR'):raise RuntimeError(line)
   time.sleep(.15)
 finally:
  send('STOP')
  deadline=time.monotonic()+2
  while time.monotonic()<deadline:
   line=s.readline(256).decode(errors='replace').strip()
   if line=='DISABLED':print(line,flush=True);break
  else:print('STOP sent; disable acknowledgment missing. Switch servo power off.',flush=True)
