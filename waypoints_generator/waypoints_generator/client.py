# This is just for testing service-calls to generate waypoints

import sys

import rclpy
from rclpy.node import Node

from anafi_uav_interfaces.srv import GetSearchPositions


class MinimalClientAsync(Node):

    def __init__(self):
      super().__init__('minimal_client_async')
      self.cli = self.create_client(GetSearchPositions, "generate_search_waypoints")
      while not self.cli.wait_for_service(timeout_sec=1.0):
        self.get_logger().info('service not available, waiting again...')
      self.req = GetSearchPositions.Request()

    def send_request(self):
      self.req.preferred_search_technique = self.req.EXPANDING_SQUARE_SEARCH
      self.future = self.cli.call_async(self.req)
      rclpy.spin_until_future_complete(self, self.future)
      return self.future.result()


def main(args=None):
  rclpy.init(args=args)

  minimal_client = MinimalClientAsync()
  response = minimal_client.send_request()
  print(response)

  minimal_client.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()