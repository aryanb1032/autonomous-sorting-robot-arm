#!/usr/bin/env python3
"""Run the explicitly requested single direction-observation sequence."""
import json,time
from pathlib import Path
import serial
ROOT=Path(__file__).resolve().parents[1]
status=ROOT/'control/active_run.json';stop=ROOT/'control/stop.request'
p='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
def state(value):
 tmp=status.with_suffix('.tmp');tmp.write_text(json.dumps({'state':value,'time':time.time()}));tmp.replace(status)
def request(s,text):s.write((text+'\n').encode())
def line(s):return s.readline(256).decode(errors='replace').strip()
stop.unlink(missing_ok=True)
s=serial.Serial(p,115200,timeout=.05,write_timeout=.5,exclusive=True)
s.dtr=False;s.rts=False
started=False;finished=False;stopped=False
try:
 time.sleep(6);s.reset_input_buffer()
 request(s,'INFO');deadline=time.monotonic()+3
 while time.monotonic()<deadline:
  reply=line(s)
  if reply=='CONFIG ONE_PASS_DIRECTION_V1':break
 else:raise RuntimeError('Firmware identity not confirmed; no RUN sent')
 state('active');request(s,'RUN_ALL');started=True
 deadline=time.monotonic()+125;beat=0;ack=False
 while time.monotonic()<deadline:
  now=time.monotonic()
  if stop.exists():
   request(s,'STOP');limit=now+3
   while time.monotonic()<limit:
    reply=line(s)
    if reply=='DISABLED':
     print('DISABLED by O-key stop',flush=True);state('disabled');stopped=True;break
   if not stopped:raise RuntimeError('STOP acknowledgment missing; cut servo power')
   break
  if now-beat>.2:
   request(s,'KEEPALIVE');state('active');beat=now
  reply=line(s)
  if not reply:continue
  print(reply,flush=True)
  if reply.startswith('ERR'):raise RuntimeError(reply)
  if reply=='ACK RUN_ALL HOME_HELD':ack=True
  if reply=='DONE ALL HOME_HELD ESTIMATED' and ack:
   state('home_held');finished=True;break
 else:raise TimeoutError('Sequence completion missing')
finally:
 if started and not finished and not stopped:
  try:request(s,'STOP')
  finally:state('failed_check_power')
 elif not started:state('not_started')
 s.close()
