#!/usr/bin/env python3
from geometry_msgs.msg import PolygonStamped
import numpy as np

class FootprintCost():
	def __init__(self, ros_object):
		pass

	def getFootprintCost(self, footprint):
		footprint_cost = 0.0

		for i in range(0, len(footprint.points)):
			x1, y1 = getWorldToMap(footprint[i+1].x, footprint[i+1].y)
			footprint_cost = np.max(getCost(x1, y1), footprint_cost)

			if (footprint_cost == 100.0):
				return 1.0

		return footprint_cost
