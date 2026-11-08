#!/usr/bin/env python3
"""Focused-window O key: disable physical outputs and pause discovered Gazebo worlds."""
import concurrent.futures
import os
import queue
import subprocess
import threading
import time
import tkinter as tk
from pathlib import Path
import json

PORT='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
events=queue.Queue()

def physical_stop():
    folder=Path(__file__).resolve().parent
    status_file=folder/'active_run.json'
    try:
        status=json.loads(status_file.read_text())
    except (OSError,ValueError):
        status={}
    if status.get('state')=='active' and time.time()-status.get('time',0)<3:
        (folder/'stop.request').touch()
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            try:
                if json.loads(status_file.read_text()).get('state')=='disabled':
                    return 'Physical: disable acknowledged by active run. Support the arm.'
            except (OSError,ValueError):pass
            time.sleep(.1)
        return 'Physical: disable NOT confirmed — cut servo power.'
    import serial
    s=serial.Serial(port=None,baudrate=115200,timeout=.05,write_timeout=.3,exclusive=True)
    s.dtr=False;s.rts=False;s.port=PORT
    try:
        s.open()
        # Repeat through possible serial-open boot noise; bounded, never HOME/MOVE.
        deadline=time.monotonic()+4
        confirmed=False
        while time.monotonic()<deadline:
            s.write(b'STOP\n')
            limit=time.monotonic()+.2
            while time.monotonic()<limit:
                line=s.readline(256).decode(errors='replace').strip()
                if line.startswith('DISABLED'):confirmed=True
        return 'Physical: disable acknowledged. Support the arm.' if confirmed else 'Physical: NO ACKNOWLEDGMENT — cut servo power.'
    except Exception as e:
        return f'Physical: STOP FAILED — cut servo power. ({e})'
    finally:
        s.close()

def simulation_stop():
    env=dict(os.environ,GZ_PARTITION='robot_arm_provisional_sim')
    try:
        result=subprocess.run(['gz','service','-l'],env=env,capture_output=True,text=True,timeout=3)
        services=[x.strip() for x in result.stdout.splitlines() if x.strip().startswith('/world/') and x.strip().endswith('/control')]
        if not services:return 'Gazebo: no running world found; pause not confirmed.'
        outcomes=[]
        for service in services:
            r=subprocess.run(['gz','service','-s',service,'--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true'],env=env,capture_output=True,text=True,timeout=3)
            outcomes.append(r.returncode==0 and 'data: true' in r.stdout)
        return 'Gazebo: world paused. Do not resume while motion goals remain.' if all(outcomes) else 'Gazebo: pause not confirmed; stop simulation manually.'
    except Exception as e:return f'Gazebo: pause failed ({e}).'

def stop_all(event=None):
    if stop_all.busy:return
    stop_all.busy=True
    status.set('STOP requested — waiting for acknowledgments…')
    def worker():
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            jobs=[pool.submit(physical_stop),pool.submit(simulation_stop)]
            events.put('\n'.join(job.result() for job in jobs))
    threading.Thread(target=worker,daemon=True).start()
stop_all.busy=False

root=tk.Tk();root.title('Robot STOP — press O in this window');root.geometry('760x360')
root.attributes('-topmost',True)
tk.Label(root,text='STOP BOTH — O',font=('Sans',26,'bold')).pack(pady=12)
tk.Label(root,text='O works while THIS WINDOW has keyboard focus.\nPhysical STOP removes holding torque. Keep the arm supported.\nSoftware/USB can fail: keep the servo power switch accessible.',font=('Sans',12)).pack()
tk.Button(root,text='STOP PHYSICAL + PAUSE GAZEBO',bg='#b00020',fg='white',font=('Sans',16,'bold'),command=stop_all).pack(pady=14)
status=tk.StringVar(value='Ready. No serial connection or motion command sent.')
tk.Label(root,textvariable=status,wraplength=730,justify='left').pack(padx=12)
root.bind('<KeyPress-o>',stop_all);root.bind('<KeyPress-O>',stop_all)
def poll():
    try:status.set(events.get_nowait());stop_all.busy=False
    except queue.Empty:pass
    root.after(100,poll)
root.protocol('WM_DELETE_WINDOW',lambda: status.set('Stop panel stays open. Closing it would remove the O-key control.'))
poll();root.mainloop()
