import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState, SetEntityState

SERVICE_NAME = "/demo/get_entity_state"

class UAV(Node):
    def __init__(self):
        node_name="uav"
        super().__init__(node_name)
        self.client = self.create_client(GetEntityState, SERVICE_NAME)
        self.run()

    def run(self):
        while not self.client.wait_for_service(1.0):
            self.get_logger().warning("Waiting for service...")
        request = GetEntityState.Request()
        request.name = "uav"
        future = self.client.call_async(request)
        future.add_done_callback(self.get_entity_state_handler)

    def get_entity_state_handler(self, future):
        try:
            response = future.result()
            print(response)
            self.get_logger().info("shutdown")
            # self.executor.shutdown()
            self.context.shutdown()
            self.destroy_node()
            exit()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = UAV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("User exit")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()