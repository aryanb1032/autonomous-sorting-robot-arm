"""Measured servo-angle ↔ physical jaw opening mapping; never extrapolate."""
import numpy as np
import math

class JawMapping:
    def __init__(self,table,finger_length=None):
        self.finger_length=finger_length
        if finger_length is not None and (not math.isfinite(finger_length) or finger_length<=0):
            raise ValueError("Positive finger length required")
        a=np.asarray(table,dtype=float)
        if a.ndim!=2 or a.shape[1]!=2 or len(a)<2 or not np.isfinite(a).all():
            raise ValueError('Jaw table needs finite [servo_rad, opening_m] measurements')
        if np.any(np.diff(a[:,0])<=0) or np.any(a[:,1]<0):
            raise ValueError('Servo angles must increase and openings cannot be negative')
        d=np.diff(a[:,1])
        if not (np.all(d>0) or np.all(d<0)):
            raise ValueError('Jaw opening must be strictly monotonic')
        self.table=a;self.minimum=float(a[:,1].min());self.maximum=float(a[:,1].max())
    def to_slide(self,servo):
        if not np.isfinite(servo) or not self.table[0,0]-1e-9<=servo<=self.table[-1,0]+1e-9:
            raise ValueError('Servo angle outside jaw calibration')
        width=float(np.interp(servo,self.table[:,0],self.table[:,1]))
        return (width-self.minimum)/2
    def to_servo(self,slide):
        width=2*slide+self.minimum
        if not np.isfinite(width) or not self.minimum-1e-9<=width<=self.maximum+1e-9:
            raise ValueError('Jaw request outside calibrated travel')
        order=np.argsort(self.table[:,1])
        return float(np.interp(width,self.table[order,1],self.table[order,0]))

    def to_joint(self,servo):
        slide=self.to_slide(servo)
        if self.finger_length is None:return slide
        if slide/self.finger_length>math.sin(.4)+1e-9:raise ValueError('Jaw opening exceeds hinge proxy range')
        return math.asin(slide/self.finger_length)
    def from_joint(self,value):
        if self.finger_length is None:return self.to_servo(value)
        if not math.isfinite(value) or not 0<=value<=.4:raise ValueError('Hinge angle outside proxy range')
        return self.to_servo(self.finger_length*math.sin(value))
