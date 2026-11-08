#!/usr/bin/env python3
"""One command owns simulation startup, controller, bounded episodes, logs and shutdown."""
import argparse,fcntl,json,os,signal,subprocess,sys,time,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def finish(proc):
 if proc is not None and proc.poll() is None:
  proc.send_signal(signal.SIGINT)
  try:proc.wait(timeout=15)
  except subprocess.TimeoutExpired:
   os.killpg(proc.pid,signal.SIGTERM)
   try:proc.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
def main():
 p=argparse.ArgumentParser();p.add_argument('--seed',type=int,default=42);p.add_argument('--per-color',type=int,choices=[1,2,3],default=3);p.add_argument('--episodes',type=int,default=1);p.add_argument('--learn',action='store_true');p.add_argument('--exit-after',action='store_true');a=p.parse_args()
 if a.episodes<1:p.error('episodes must be positive')
 own=(ROOT/'autonomous/.run.lock').open('w')
 try:fcntl.flock(own,fcntl.LOCK_EX|fcntl.LOCK_NB)
 except BlockingIOError:raise SystemExit('Autonomous launcher is already running')
 probe=(ROOT/'.simulation.lock').open('a')
 try:fcntl.flock(probe,fcntl.LOCK_EX|fcntl.LOCK_NB)
 except BlockingIOError:raise SystemExit('Close the existing simulation first; no changes made')
 probe.close()
 stop=ROOT/'autonomous/STOP';stop.unlink(missing_ok=True)
 run=ROOT/'autonomous/runs'/time.strftime('%Y%m%d-%H%M%S');run.mkdir(parents=True)
 sim=controller=panel=None
 try:
  for episode in range(a.episodes):
   if stop.exists():raise InterruptedError('STOP requested')
   folder=run/f'episode-{episode+1:03d}';folder.mkdir()
   subprocess.run([sys.executable,str(ROOT/'sorting/build_workcell.py'),'--seed',str(a.seed+episode),'--per-color',str(a.per_color)],check=True)
   for source in [ROOT/'sorting/layout.json',ROOT/'ros2_ws/src/arm_simulation/config/geometry.yaml']:
    if source.exists():shutil.copy2(source,folder/source.name)
   (folder/'run_config.json').write_text(json.dumps({'seed':a.seed+episode,'per_color':a.per_color,'learn':a.learn,'physical_control':False},indent=2))
   # Workspace uses symlink-install: changed SDF/layout/plugin files are immediately visible.
   simlog=(folder/'gazebo.log').open('w')
   sim=subprocess.Popen([sys.executable,str(ROOT/'scripts/run_simulation.py'),'gazebo','sorting:=true'],stdout=simlog,stderr=subprocess.STDOUT,start_new_session=True)
   panel=subprocess.Popen([sys.executable,str(ROOT/'autonomous/dashboard.py'),str(folder)],start_new_session=True)
   import rclpy
   from rclpy.action import ActionClient
   from moveit_msgs.action import MoveGroup
   rclpy.init();node=rclpy.create_node('autonomous_startup_check')
   try:
    client=ActionClient(node,MoveGroup,'/move_action');deadline=time.monotonic()+90
    while not client.wait_for_server(timeout_sec=.5):
     if stop.exists():raise InterruptedError('Startup stopped')
     if sim.poll() is not None:raise RuntimeError('Gazebo exited; see gazebo.log')
     if time.monotonic()>deadline:raise TimeoutError('MoveIt startup timeout')
   finally:node.destroy_node();rclpy.shutdown()
   cmd=[sys.executable,str(ROOT/'autonomous/run_sorter.py'),'--run-dir',str(folder),'--seed',str(a.seed+episode)]
   if a.learn:cmd.append('--learn')
   with (folder/'controller.log').open('w') as log:
    controller=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    print(f'Autonomous episode {episode+1}; logs: {folder}',flush=True)
    while controller.poll() is None:
     if sim.poll() is not None:raise RuntimeError('Simulation exited during sorting')
     time.sleep(.2)
    if controller.returncode:raise RuntimeError(f'Sorting stopped; see {folder}/controller.log')
   print((folder/'result.json').read_text(),flush=True)
   if episode==a.episodes-1 and not a.exit_after:
    print('Completed. Gazebo/RViz remain open. Ctrl+C closes this run.',flush=True)
    while sim.poll() is None and not stop.exists():time.sleep(.2)
   finish(panel);panel=None;finish(sim);sim=None;simlog.close()
 except KeyboardInterrupt:stop.touch()
 except Exception as error:
  print(str(error),file=sys.stderr,flush=True);raise SystemExit(1)
 finally:
  finish(controller);finish(panel);finish(sim)
if __name__=='__main__':main()
