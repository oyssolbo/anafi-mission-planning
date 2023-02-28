#!/bin/env/python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from anafi_uav_interfaces.srv import GetSearchPositions#, GetSearchPositionsRequest, GetSearchPositionsResponse

from . import search_pattern

class GenerateSearchWaypointsNode(Node):

	def __init__(self):
		super().__init__('generate_search_waypoints_node')
		self.srv = self.create_service(GetSearchPositions, '/generate_search_waypoints', self.generate_waypoints)
		self.get_logger().info("Node for generating search positions activated")

	def generate_waypoints(
				self, 
				request,#		: GetSearchPositionsRequest, 
				response# 	: GetSearchPositionsResponse
			):# -> GetSearchPositionsResponse:
		preferred_search_technique = request.preferred_search_technique

		# Add these parameters into a config file
		search_altitude = 5
		search_overlap = 0.25
		search_area = (20, 20) # Not have too much area to cover - not much battery
		camera_fov = search_pattern.get_fov_from_hfov(1280, 720, 69)

		if preferred_search_technique == request.EXPANDING_SQUARE_SEARCH:
			positions = search_pattern.expanding_square_search_ned(search_altitude, camera_fov, search_overlap, search_area)
		elif preferred_search_technique == request.LINE_SEARCH:
			self.get_logger().warn("Beware that the positions are in longitude, latidude and altidue")
			positions = search_pattern.line_search_lla(search_altitude, camera_fov, search_overlap, search_area)

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