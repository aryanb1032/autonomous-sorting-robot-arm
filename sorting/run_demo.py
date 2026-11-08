#!/usr/bin/env python3
"""Start a fresh sorting simulation, run one cycle, keep windows open for inspection."""
from pathlib import Path
import fcntl,subprocess,signal,time,argparse
p=argparse.ArgumentParser()
p.add_argument("--seed",type=int)
p.add_argument("--per-color",type=int,choices=[1,2,3],default=3)
a=p.parse_args()
import rclpy
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
ROOT=Path(__file__).resolve().parents[1]
lock=(ROOT/'.simulation.lock').open('w')
try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
except BlockingIOError:
    raise SystemExit('A simulation is already running. Stop it with Ctrl+C in its terminal before starting a fresh sorting demo.')
fcntl.flock(lock,fcntl.LOCK_UN);lock.close()
generate=['python',str(ROOT/'sorting/build_workcell.py'),'--per-color',str(a.per_color)]
if a.seed is not None:generate+=['--seed',str(a.seed)]
subprocess.run(generate,check=True)
subprocess.run(['bash',str(ROOT/'scripts/build.sh')],check=True)
sim=subprocess.Popen(['bash',str(ROOT/'scripts/start_simulation.sh'),'gazebo','sorting:=true'])
sorter=None
try:
    rclpy.init();node=rclpy.create_node('wait_for_sorting_simulation')
    try:
        client=ActionClient(node,MoveGroup,'/move_action');deadline=time.monotonic()+60
        while not client.wait_for_server(timeout_sec=.5):
            if sim.poll() is not None:raise RuntimeError('Simulation startup failed')
            if time.monotonic()>deadline:raise RuntimeError('MoveIt did not become ready')
    finally:node.destroy_node();rclpy.shutdown()
    sorter=subprocess.Popen(['python',str(ROOT/'sorting/sort_boxes.py')])
    if sorter.wait()!=0:raise RuntimeError('Sorting failed; inspect the preceding error')
    print('\nSorting complete. Gazebo and RViz remain open. Press Ctrl+C here to close them.',flush=True)
    sim.wait()
except KeyboardInterrupt:
    pass
finally:
    for proc in [sorter,sim]:
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                proc.terminate();proc.wait(timeout=5)
