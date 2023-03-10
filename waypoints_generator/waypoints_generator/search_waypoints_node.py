#!/bin/env/python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from anafi_uav_interfaces.srv import GetSearchPositions#, GetSearchPositionsRequest, GetSearchPositionsResponse

from . import search_pattern

class GenerateSearchWaypointsNode(Node):

	def __init__(self):
		super().__init__('generate_search_waypoints_node')

		self.declare_parameter('search.altitude')
		self.declare_parameter('search.overlap')
		self.declare_parameter('search.area')

		self.search_altitude = self.get_parameter('search.altitude').get_parameter_value().double_value
		self.search_overlap = self.get_parameter('search.overlap').get_parameter_value().double_value
		self.search_area = self.get_parameter('search.area').get_parameter_value().double_array_value

		self.srv = self.create_service(GetSearchPositions, "/waypoint_generator/generate_search_waypoints", self.generate_waypoints)
		# self.get_logger().info("Node for generating search positions activated")

	def generate_waypoints(
				self, 
				request,#		: GetSearchPositionsRequest, 
				response# 	: GetSearchPositionsResponse
			):# -> GetSearchPositionsResponse:
		preferred_search_technique = request.preferred_search_technique

		camera_fov = search_pattern.get_fov_from_hfov(1280, 720, 69)

		if preferred_search_technique == request.EXPANDING_SQUARE_SEARCH:
			positions = search_pattern.expanding_square_search_ned(self.search_altitude, camera_fov, self.search_overlap, self.search_area)
		elif preferred_search_technique == request.LINE_SEARCH:
			self.get_logger().warn("Beware that the positions are in longitude, latidude and altidue")
			positions = search_pattern.line_search_lla(self.search_altitude, camera_fov, self.search_overlap, self.search_area)

		msg = Float64MultiArray()
		msg.data = [p for pos in positions for p in pos] # Flattening data

		dim0 = MultiArrayDimension()
		dim0.size = len(positions) # First dimension size is the number of positions
		dim0.stride = len(msg.data)

		dim1 = MultiArrayDimension()
		dim1.size = len(positions[0]) 
		dim1.stride = dim1.size # Last stride equal to the size when there is no padding

		msg.layout.dim = [dim0, dim1]

		response.positions = msg
		response.success = True

		return response


def main(args=None):
	rclpy.init(args=args)
	search_waypoints_node = GenerateSearchWaypointsNode()
	rclpy.spin(search_waypoints_node)
	rclpy.shutdown()


if __name__ == '__main__':
	main()