import rclpy
from std_msgs.msg import Float32MultiArray

def get_topic_data_once(topic_name, message_type, timeout=5.0):

    rclpy.init()
    node = rclpy.create_node('temporary_subscriber')

    # Create a future to store the received message
    future = rclpy.Future()

    def callback(msg):
        future.set_result(msg.data)

    subscription = node.create_subscription(
        message_type,
        topic_name,
        callback,
        10
    )

    try:
        # Wait for the message with a timeout
        current_time = node.get_clock().now()
        start_time = current_time.nanoseconds / 1e9
        while not future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            current_time = node.get_clock().now()
            current_time_ = current_time.nanoseconds / 1e9
            if current_time_ - start_time > timeout:
                node.get_logger().warn(f"Timeout waiting for topic: {topic_name}")
                return None
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return future.result()

# Example usage
if __name__ == "__main__":
    topic_name = 'load_cells_data'
    message_type = Float32MultiArray 

    data = get_topic_data_once(topic_name, message_type)
    if data is not None:
        print(f"Latest data from topic '{topic_name}': {data}")
    else:
        print(f"Failed to get data from topic '{topic_name}'.")
