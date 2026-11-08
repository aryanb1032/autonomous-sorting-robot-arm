"""Read-only protocol test; no movement commands."""
import argparse
import time
import serial
p=argparse.ArgumentParser();p.add_argument('port');args=p.parse_args()
with serial.Serial(args.port,115200,timeout=1) as connection:
    time.sleep(2);connection.reset_input_buffer();connection.write(b'PING\n')
    reply=connection.readline().decode().strip()
    if not reply.startswith('PONG '):raise SystemExit('No valid PONG: '+repr(reply))
    print('Communication OK:',reply)
