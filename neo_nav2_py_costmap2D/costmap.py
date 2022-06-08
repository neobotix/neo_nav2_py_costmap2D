#!/usr/bin/env python3
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PolygonStamped, Polygon, Point
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

	def footprint_callback(self, msg):
		self.footprint = msg.polygon

	def getFootprintCost(self, footprint):
		footprint_cost = 0.0

		for i in range(0, len(footprint.points)-1):
			x1, y1 = self.getWorldToMap(footprint.points[i].x, footprint.points[i].y)
			footprint_cost = np.fmax(self.getCost(x1, y1), footprint_cost)

			if (footprint_cost == 1.0):
				return 1.0

		return footprint_cost

	def getFootprintCostAtPose(self, x, y, theta, footprint):
		cos_th = np.cos(theta)
		sin_th = np.sin(theta)
		oriented_footprint = Polygon()

		for i in range(0, len(footprint.points)):
			new_pt = Point()
			new_pt.x = x + (footprint.points[i].x * cos_th - footprint.points[i].y * sin_th)
			new_pt.y = y + (footprint.points[i].x * sin_th + footprint.points[i].y * cos_th)
			oriented_footprint.points.append(new_pt)

		return (self.getFootprintCost(oriented_footprint))

	def costmap_callback(self, msg):
		self.costmap = msg
		self.map_resolution = msg.info.resolution
		self.grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
		self.size_x_ = msg.info.width

	def getCost(self, mx, my):
		
		if(abs(mx) > self.costmap.info.height - 1 or abs(my) > self.costmap.info.width - 1):
			return 1.0

		return self.grid[int(my)][int(mx)] / 100.

	def getWorldToMap(self, x, y):
		mx = round((x - self.costmap.info.origin.position.x) / self.costmap.info.resolution)
		my = round((y - self.costmap.info.origin.position.y) / self.costmap.info.resolution)

		return mx, my
