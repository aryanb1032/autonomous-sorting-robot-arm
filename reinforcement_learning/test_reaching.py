import unittest
import numpy as np
from reach_env import candidate, HOME, LOW, HIGH

class ActionTests(unittest.TestCase):
    def test_bounds_and_wrist_compensation(self):
        q = HOME.copy()
        for _ in range(100):
            q = candidate(q, [100., 100., -100.])
        np.testing.assert_allclose(q[:3], [HIGH[0], HIGH[1], LOW[2]])
        self.assertAlmostEqual(q[1] + q[2] + q[4], 0.)
        self.assertEqual(q[3], 0.)
    def test_no_action_holds(self):
        np.testing.assert_allclose(candidate(HOME, [0., 0., 0.]), HOME)
    def test_bad_action_rejected(self):
        for a in ([np.nan, 0., 0.], [np.inf, 0., 0.], [0., 0.]):
            with self.assertRaises(ValueError):
                candidate(HOME, a)

if __name__ == '__main__':
    unittest.main()
