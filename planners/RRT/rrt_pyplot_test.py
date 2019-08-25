import time
import numpy as np
import unittest

from rrt_pyplot import Config, RRT

class TestRRT(unittest.TestCase):

    def test_add_config(self):
        rrt = RRT(Config([0, 0]))

        rrt.add_config(Config([0, 0]), Config([10, 10]))

        self.assertEqual(rrt.graph[Config([0, 0]).id()][0], Config([10, 10]).id())
        self.assertEqual(rrt.graph[Config([10, 10]).id()], [])

    def test_build_graph_components(self):
        rrt = RRT(Config([0.0, 0.0]))

        qrand = rrt.get_rand_config()
        for i, val in enumerate(qrand.values):
            self.assertTrue(val >= rrt.limits[i][0] and val <= rrt.limits[i][1])

        qnear = rrt.get_nearest_vertex(Config([10.0, 10.0]))
        self.assertEqual(qnear.id(), Config([0.0, 0.0]).id())

        qnew = rrt.get_new_vertex(qnear, Config([10.0, 10.0]), 0.1)
        self.assertTrue(abs(qnew.distance(qnear) - 0.1 < 0.01))

        rrt.add_config(qnear, qnew)
        self.assertTrue(qnew.id() in rrt.graph[qnear.id()])


if __name__ == '__main__':
    unittest.main()

