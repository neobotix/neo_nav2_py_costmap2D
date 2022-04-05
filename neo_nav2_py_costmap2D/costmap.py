#!/usr/bin/env python3
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np

class Costmap2d():
	def __init__(self, ros_object):
		self.subscription = ros_object.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.costmap_callback,
            10)
		self.subscription  # prevent unused variable warning.
		self.map_resolution = 0.0

	def costmap_callback(self, msg):
		self.costmap = msg
		self.map_resolution = msg.info.resolution
		self.grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
		self.size_x_ = msg.info.width

	def getCost(self, x, y):
		mx, my = self.getWorldToMap(x, y)
		if(abs(mx) > self.costmap.info.height - 1 or abs(my) > self.costmap.info.width - 1):
			return 1.0

		return self.grid[int(mx)][int(my)] / 100.

	def getWorldToMap(self, x, y):
		mx = round((x - self.costmap.info.origin.position.x) / self.costmap.info.resolution)
		my = round((y - self.costmap.info.origin.position.y) / self.costmap.info.resolution)

		return mx, my
