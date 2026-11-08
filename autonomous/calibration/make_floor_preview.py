"""Single-frame diagnostic mapping, not a robot calibration.
Corners are manually selected on the saved image. No lens model is estimated.
"""
from pathlib import Path
import json,cv2,numpy as np
HERE=Path(__file__).resolve().parent
image=cv2.imread(str(HERE/'floor_reference.jpg'))
src=np.float32([[262,134],[566,165],[521,595],[222,562]])
# Local image-oriented floor coordinates only. Rectangle dimensions 150 x 220 mm.
dst=np.float32([[0,0],[150,0],[150,220],[0,220]])
h=cv2.getPerspectiveTransform(src,dst)
preview=cv2.warpPerspective(image,np.diag([3.,3.,1.])@h,(450,660))
cv2.imwrite(str(HERE/'floor_preview.jpg'),preview)
annotated=image.copy()
for i,p in enumerate(src):
 cv2.circle(annotated,tuple(p.astype(int)),7,(0,255,255),2)
 cv2.putText(annotated,str(i+1),tuple(p.astype(int)+[8,-8]),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,255),2)
cv2.imwrite(str(HERE/'selected_floor_corners.jpg'),annotated)
(HERE/'provisional_floor_mapping.json').write_text(json.dumps({'validated':False,'physical_control_enabled':False,'coordinate_frame':'pickup_rectangle_local_image_axes','plane_height_mm':0,'image_size':[1280,720],'corners_raw_px':src.tolist(),'corners_local_mm':dst.tolist(),'raw_pixel_to_floor_mm':h.tolist(),'limitations':['Single image, four manually selected points; no independent validation','150 mm across short edge and 220 mm across long edge assumed from supplied dimensions','Lens distortion not corrected','Floor mapping is not the cube-top plane at 20 mm','No robot-base origin or heading registered','Bins cropped; no bin coordinates','Changes in mount, lens or resolution invalidate this preview']},indent=2)+'\n')
