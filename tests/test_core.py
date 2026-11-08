import math
import cv2
import numpy as np
import pytest
from arm_sorter.geometry import measure_polygon, classify_brightness
from arm_sorter.validation import validate_points

def test_rotated_box_metric_width():
    polygon=cv2.boxPoints(((200.,150.),(80.,40.),37.))
    result=measure_polygon(polygon,[[.001,0,0],[0,.001,0],[0,0,1]])
    assert result['width_m']==pytest.approx(.04,abs=1e-6)
    assert result['length_m']==pytest.approx(.08,abs=1e-6)
    assert result['x']==pytest.approx(.2,abs=1e-6)

def test_perspective_measurement():
    h=np.array([[.001,0,0],[0,.001,0],[.001,0,1.]])
    corners=np.array([[[.1,.1],[.15,.1],[.15,.13],[.1,.13]]],np.float32)
    pixels=cv2.perspectiveTransform(corners,np.linalg.inv(h))[0]
    assert measure_polygon(pixels,h)['width_m']==pytest.approx(.03,abs=1e-5)

def test_projection_rejects_bad_calibration():
    with pytest.raises(ValueError):measure_polygon([[0,0],[1,0],[1,1],[0,1]],np.zeros((3,3)))

@pytest.mark.parametrize('brightness,expected',[(20,'black'),(230,'white'),(120,'uncertain')])
def test_colour(brightness,expected):
    assert classify_brightness(np.full((20,20,3),brightness,np.uint8),np.ones((20,20),np.uint8)*255)==expected

@pytest.mark.parametrize('names,points',[(['bad'],[(1,[0])]),(['j','j'],[(1,[0,0])]),(['j'],[(1,[float('nan')])]),(['j'],[(1,[2])]),(['j'],[(2,[0]),(1,[0])]),(['j'],[])])
def test_invalid_trajectory(names,points):
    with pytest.raises(ValueError):validate_points(names,points,{'joint_names':['j'],'limits':{'j':[-1,1]}})

def test_valid_trajectory():
    validate_points(['j'],[(0,[0]),(1,[.5])],{'joint_names':['j'],'limits':{'j':[-1,1]}})
