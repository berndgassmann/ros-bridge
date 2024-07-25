import rclpy

from object_visualizer.object_visualizer import ObjectVisualizer


def main(args=None):
    rclpy.init(args=args)

    # create nodes
    object_visualizer = ObjectVisualizer()

    # create executor and add nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    if(not executor.add_node(object_visualizer)):
        object_visualizer.get_logger().error(
            "Can't add object_visualizer to executor")

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Keyboard interrupt. Shutting down.")

    object_visualizer.before_shutdown()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
