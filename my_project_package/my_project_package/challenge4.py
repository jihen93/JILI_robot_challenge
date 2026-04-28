import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from cv_bridge import CvBridge

class GoalNode(Node):
    def __init__(self):
        super().__init__('goal_node') 

        self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
        self.image_publisher = self.create_publisher(Image, '/processed_image', 10)
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.cv_bridge = CvBridge()
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.obstacle_detecte = False
        self.distance = 10.0 # Initialisé loin

        self.kp = 0.004
        self.ki = 0.0
        self.kd = 0.0025
        self.last_error = 0.0
        self.integral = 0.0
        self.max_angular_vel = 1.0
        self.objet_touche = False

    def scan_callback(self, msg):
        points_devant = [r for r in msg.ranges[0:20] + msg.ranges[340:359] if r > 0.05]
        if points_devant:
            self.distance = min(points_devant)
            self.obstacle_detecte = self.distance <= 0.8

    def listener_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Erreur conversion: {e}")
            return

        if cv_image is not None:
            height, width = cv_image.shape[:2]
            image_center = width / 2
            cv_image_roi = cv_image[int(height * 0.1):height, :]
            hsv = cv2.cvtColor(cv_image_roi, cv2.COLOR_BGR2HSV)

            # Masques
            mask_ball = cv2.inRange(hsv, np.array([25, 80, 60]), np.array([50, 255, 255]))
            mask_red1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
            mask_red2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)
            mask_total = cv2.bitwise_or(mask_red, mask_ball)

            M_r = cv2.moments(mask_red)
            M_balle = cv2.moments(mask_ball)
            msg_twist = Twist()


            self.get_logger().info(f"Distance: {self.distance:.2f}m | Touche: {self.objet_touche}")

            # --- LOGIQUE DE DECISION (Maintenant bien indentée dans le callback) ---
            if not self.objet_touche:
                if M_balle['m00'] > 50:
                    cx = int(M_balle['m10'] / M_balle['m00'])
                    error = image_center - cx
                    if self.distance <= 0.60:
                        msg_twist.linear.x = 0.0
                        self.objet_touche = True
                    else:
                        msg_twist.linear.x = 0.15
                        msg_twist.angular.z = self.kp * error
                else:
                    msg_twist.angular.z = 0.25 # Cherche la balle
            else:
                if M_r['m00'] > 100:
                    cx = int(M_r['m10'] / M_r['m00'])
                    error = image_center - cx
                    msg_twist.linear.x = 0.15 # Réduit de 0.7 à 0.5 pour plus de stabilité
                    msg_twist.angular.z = self.kp * error
                else:
                    # Orbite autour de la balle
                    msg_twist.linear.x = 0.15 
                    msg_twist.angular.z = 0.6 
                    if self.distance < 0.15: msg_twist.angular.z += 0.1
                    if self.distance > 0.25: msg_twist.angular.z -= 0.1

            # Publication (Sortie de la logique if/else mais dans le if cv_image)
            self.vel_publisher.publish(msg_twist)
            ros_img = self.cv_bridge.cv2_to_imgmsg(cv_image_roi, encoding="bgr8")
            self.image_publisher.publish(ros_img)

            try:
                cv2.imshow("Masque Total", mask_total)
                cv2.waitKey(1)
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)  
    node = GoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.vel_publisher.publish(Twist())
            node.destroy_node()
            rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()