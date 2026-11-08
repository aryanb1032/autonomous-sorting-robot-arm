"""Keep one simulation launch, and clean up its child processes on exit."""
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import sys
import psutil
root=Path(__file__).resolve().parents[1]
lock=(root/'.simulation.lock').open('w')
try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
except BlockingIOError:raise SystemExit('Simulation is already running. Stop it before starting another.')
proc=subprocess.Popen(['ros2','launch','arm_simulation','simulation.launch.py',f'backend:={sys.argv[1] if len(sys.argv)>1 else "gazebo"}','gui:=true','rviz:=true',*sys.argv[2:]])
children=[]
def stop(signum,frame):
    global children
    try:children=psutil.Process(proc.pid).children(recursive=True)
    except psutil.NoSuchProcess:pass
    proc.send_signal(signal.SIGINT)
    try:proc.wait(timeout=7)
    except subprocess.TimeoutExpired:proc.terminate()
    for child in reversed(children):
        try:child.terminate()
        except psutil.NoSuchProcess:pass
    _,alive=psutil.wait_procs(children,timeout=3)
    for child in alive:
        try:child.kill()
        except psutil.NoSuchProcess:pass
    raise SystemExit(0)
signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
while proc.poll() is None:
    try:children=psutil.Process(proc.pid).children(recursive=True)
    except psutil.NoSuchProcess:pass
    try:proc.wait(timeout=1)
    except subprocess.TimeoutExpired:pass
for child in reversed(children):
    try:child.terminate()
    except psutil.NoSuchProcess:pass
sys.exit(proc.returncode)
