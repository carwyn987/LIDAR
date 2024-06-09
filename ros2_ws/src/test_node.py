import rclpy
from geometry_msgs.msg import Quaternion

def callback(msg):
    print('Received: ', msg.x, msg.y, msg.z, msg.w)

rclpy.init()
node = rclpy.create_node('my_first_node')
publisher = node.create_publisher(Quaternion, 'my_first_publisher', 10)
subscriber = node.create_subscription(Quaternion, 'my_first_subscription', callback, 10)

D = Quaternion()
D.x = 1.0
D.y = 2.0
D.z = 3.0
D.w = 1.0
publisher.publish(D)

# print(D)

rclpy.spin(node)