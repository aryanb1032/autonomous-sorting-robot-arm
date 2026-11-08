"""Click measured reference points in an unmodified snapshot; enter their XY in mm."""
import argparse
import json
from pathlib import Path
import cv2
p=argparse.ArgumentParser();p.add_argument('image');p.add_argument('output');args=p.parse_args()
image=cv2.imread(args.image)
if image is None:raise SystemExit('Cannot read snapshot')
points=[]
def click(event,x,y,flags,param):
    if event==cv2.EVENT_LBUTTONDOWN:points.append([x,y])
cv2.namedWindow('Click references; u undo; Enter finish; q cancel')
cv2.setMouseCallback('Click references; u undo; Enter finish; q cancel',click)
while True:
    shown=image.copy()
    for i,(x,y) in enumerate(points):
        cv2.circle(shown,(x,y),4,(0,0,255),-1)
        cv2.putText(shown,str(i+1),(x+6,y-6),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,255),2)
    cv2.imshow('Click references; u undo; Enter finish; q cancel',shown)
    key=cv2.waitKey(30)&255
    if key==ord('q'):cv2.destroyAllWindows();raise SystemExit('Cancelled')
    if key==ord('u') and points:points.pop()
    if key in (10,13) and len(points)>=6:break
cv2.destroyAllWindows()
coordinates=[]
for i,point in enumerate(points):
    while True:
        try:
            pair=[float(v) for v in input(f'Reference {i+1}, pixel {point}: measured X Y in mm: ').split()]
            if len(pair)!=2:raise ValueError()
            coordinates.append(pair);break
        except ValueError:print('Enter two numbers separated by a space.')
Path(args.output).write_text(json.dumps({'pixels':points,'plane_xy_mm':coordinates},indent=2)+'\n')
print('Saved',args.output)
