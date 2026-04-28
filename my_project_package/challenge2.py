import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import cv2

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.subscription = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.listener_callback, 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10) # publisher : pour faire bouger le robot
        self.obstacle_detecte = False

    def scan_callback(self, msg):
        """ Fonction en boucle qui surveille les obstacles """
        points_devant = [r for r in msg.ranges[0:20] + msg.ranges[340:359] if r > 0.05]

        if points_devant:
            distance_min = min(points_devant)
            if distance_min < 0.5:
                self.obstacle_detecte = True
            else:
                self.obstacle_detecte = False
        else:
            self.obstacle_detecte = False

    def listener_callback(self, msg):
        """ Fonction qui gère la caméra et le mouvement """
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if image is not None:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            lower_blue = np.array([60, 100, 100])
            upper_blue = np.array([130, 255, 255])
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
            mask = cv2.inRange(hsv, lower_blue, upper_blue)

            mask_red1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
            mask_red2 = cv2.inRange(hsv, np.array([170, 100, 100]), np.array([180, 255, 255]))
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)

            mask_total = cv2.bitwise_or(mask_blue, mask_red)
            M = cv2.moments(mask_total)
            msg_twist = Twist()

            if self.obstacle_detecte:
                msg_twist.linear.x = 0.0
                msg_twist.angular.z = 0.0
                self.get_logger().warn("ARRET : obstacle devant !")
            else:
                if M['m00'] > 0:
                    cx = int(M['m10']/M['m00'])
                    image_center = image.shape[1] / 2
                    error = cx - image_center

                    msg_twist.linear.x = 0.08
                    msg_twist.angular.z = -float(error) / 100.0
                else:
                    msg_twist.linear.x = 0.0
                    msg_twist.angular.z = 0.2

            self.publisher.publish(msg_twist)

            cv2.imshow("Detection Ligne", image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

