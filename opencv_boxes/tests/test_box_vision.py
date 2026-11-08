import cv2
import numpy as np
import pytest
from box_vision import detect

def scene():
    image=np.full((480,640,3),125,np.uint8)
    for center,angle,value in [((120,140),0,20),((340,140),25,240),((120,340),-15,20),((340,340),0,240)]:
        points=cv2.boxPoints((center,(80,120),angle)).astype(np.int32)
        cv2.fillPoly(image,[points],(value,value,value))
    return image

def calibration():
    return dict(image_size=[640,480],camera_matrix=[[500,0,320],[0,500,240],[0,0,1]],dist_coeffs=[0]*5,pixel_to_plane_mm=[[.25,0,0],[0,.25,0],[0,0,1]])

def test_detect_multiple_colours_and_metric_sides():
    report,_,_=detect(scene(),calibration=calibration())
    assert report['counts']=={'black':2,'white':2}
    for box in report['objects']:
        assert box['width_mm']==pytest.approx(20,abs=.5)
        assert box['length_mm']==pytest.approx(30,abs=.5)
        assert box['matches_20mm_side']

def test_uncalibrated_never_fabricates_mm():
    report,_,_=detect(scene())
    assert report['counts']=={'black':2,'white':2}
    assert not report['calibrated']
    assert all(b['width_mm'] is None and b['matches_20mm_side'] is None for b in report['objects'])

def test_clipped_and_round_objects_rejected():
    image=np.full((480,640,3),125,np.uint8)
    cv2.rectangle(image,(0,30),(100,130),(10,10,10),-1)
    cv2.circle(image,(300,200),40,(245,245,245),-1)
    assert detect(image)[0]['objects']==[]

def test_wrong_resolution_rejected():
    with pytest.raises(ValueError):detect(scene()[:300],calibration=calibration())

def test_midgrey_no_boxes_and_bad_thresholds():
    assert detect(np.full((480,640,3),125,np.uint8))[0]['objects']==[]
    with pytest.raises(ValueError):detect(scene(),{'black_max':200,'white_min':150})

def test_bad_roi_rejected():
    with pytest.raises(ValueError):detect(scene(),{'roi':[600,0,100,100]})
