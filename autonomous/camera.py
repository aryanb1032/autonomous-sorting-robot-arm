#!/usr/bin/env python3
"""Start the configured phone detector. Vision preparation only, no motor interface."""
import argparse,json,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--config',type=Path,default=Path(__file__).with_name('camera_config.json'))
 p.add_argument('--headless',action='store_true');p.add_argument('--frames',type=int,default=0)
 a=p.parse_args();c=json.loads(a.config.read_text())
 cmd=[sys.executable,str(ROOT/'opencv_boxes/box_vision.py'),'--stream',c['stream_url'],
      '--settings',str(ROOT/'opencv_boxes/settings.json'),'--output',str(ROOT/'../../work/phone-camera-live'),
      '--frames',str(a.frames)]
 if a.headless:cmd.append('--headless')
 if c.get('calibration_file'):cmd+=['--calibration',c['calibration_file']]
 print('Phone vision only. No motor commands. Calibration is required for robot coordinates.',flush=True)
 try:raise SystemExit(subprocess.call(cmd))
 except KeyboardInterrupt:pass
if __name__=='__main__':main()
