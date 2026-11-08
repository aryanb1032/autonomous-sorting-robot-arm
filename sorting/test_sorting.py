import math,unittest
import numpy as np
from sort_boxes import joints,inside_bin,LAYOUT,L1,L2,SHOULDER_HEIGHT,TOOL_OFFSET
from build_workcell import generate

class SortingTests(unittest.TestCase):
    def test_ik_reaches_requested_position(self):
        for angle in [-.85,-.23,.23,.85]:
            for z in [.01,.07,.12]:
                _,a,b,c,_=joints(angle,z)
                radius=L1*math.sin(a)+L2*math.sin(a+b)+TOOL_OFFSET*math.sin(a+b+c)
                height=SHOULDER_HEIGHT+L1*math.cos(a)+L2*math.cos(a+b)+TOOL_OFFSET*math.cos(a+b+c)
                self.assertAlmostEqual(radius,.3,places=10)
                self.assertAlmostEqual(height,z,places=10)
    def test_correct_and_wrong_bin(self):
        target=LAYOUT['bins']['black'];p=np.array(target['centre'])
        self.assertTrue(inside_bin(p,[0,0,0,1],target))
        self.assertFalse(inside_bin(p,[0,0,0,1],LAYOUT['bins']['white']))
    def test_edge_and_air_rejected(self):
        target=LAYOUT['bins']['black'];p=np.array(target['centre'])
        self.assertFalse(inside_bin(p+[.04,0,0],[0,0,0,1],target))
        self.assertFalse(inside_bin(p+[0,0,.05],[0,0,0,1],target))

class LayoutTests(unittest.TestCase):
    def test_random_layouts_spaced_and_balanced(self):
        for seed in range(100):
            layout=generate(seed,3,write=False)
            objects=list(layout['objects'].values())
            self.assertEqual(sum(o['color']=='black' for o in objects),3)
            self.assertEqual(sum(o['color']=='white' for o in objects),3)
            self.assertEqual(len(set(o['color'] for o in objects[:3])),2)
            for i,a in enumerate(objects):
                for b in objects[i+1:]:
                    self.assertGreater(np.linalg.norm(np.array(a['xyz'])-b['xyz']),.065)
                self.assertEqual(a['xyz'][2],.01)
                self.assertEqual(a['support'],'sorting_ground')
                joints(a['source_angle'],.01,a['source_radius'])
                joints(a['source_angle'],.12,a['source_radius'])
    def test_seed_repeatability_and_fixed_bins(self):
        a=generate(42,write=False);b=generate(43,write=False)
        self.assertEqual(a,generate(42,write=False))
        self.assertNotEqual(a['objects'],b['objects'])
        self.assertEqual(a['bins'],b['bins'])
    def test_drop_slots_fit_and_do_not_overlap(self):
        for target in LAYOUT['bins'].values():
            for i,p in enumerate(target['slots']):
                self.assertTrue(inside_bin(np.array(p),[0,0,0,1],target))
                for other in target['slots'][i+1:]:
                    self.assertGreater(np.linalg.norm(np.array(p)-other),.035)

if __name__=='__main__':unittest.main()
