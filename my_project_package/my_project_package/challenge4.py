import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, CompressedImage
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from cv_bridge import CvBridge

class GoalNode(Node):
    def __init__(self):
        super().__init__('goal_node') 

        self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
        #self.subscription = self.create_subscription(CompressedImage, 'camera/image_raw/compressed', self.listener_callback, 10) # pour le robot reel

        self.image_publisher = self.create_publisher(Image, '/processed_image', 10)
        
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.cv_bridge = CvBridge()
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.obstacle_detecte = False
        self.distance = 10.0 # Initialisé loin

        self.kp = 0.00025
        self.ki = 0.0
        self.kd = 0.0005
        self.last_error = 0.0
        self.integral = 0.0
        self.max_angular_vel = 1.0
        self.objet_touche = False

        self.poteau_gauche = None  
        self.poteau_droit = None  

        self.depassement_en_cours = False


    def scan_callback(self, msg):
        points_devant = [r for r in msg.ranges[0:20] + msg.ranges[340:359] if r > 0.05]
        if points_devant:
            self.distance = min(points_devant)
            self.obstacle_detecte = self.distance <= 0.8

        # AJOUT : poteaux visibles sur les côtés pendant l'orbite
        gauche = [r for r in msg.ranges[300:345] if 0.3 < r < 3.0]
        droite = [r for r in msg.ranges[15:60]   if 0.3 < r < 3.0]
        self.poteau_gauche = len(gauche) > 0
        self.poteau_droit  = len(droite) > 0

    def listener_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            #cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8') #pour le robot reel 

        except Exception as e:
            self.get_logger().error(f"Erreur conversion: {e}")
            return

        if cv_image is not None:
            height, width = cv_image.shape[:2]
            image_center = width / 2
            cv_image_roi = cv_image[int(height * 0.1):height, :]
            hsv = cv2.cvtColor(cv_image_roi, cv2.COLOR_BGR2HSV)

            # CORRECTION 1 : plage HSV élargie [5-30] au lieu de [25-50]
            # pour couvrir l'orange réel selon l'éclairage
            mask_ball = cv2.inRange(hsv, np.array([25, 80, 60]),    np.array([50, 255, 255]))
            mask_red1 = cv2.inRange(hsv, np.array([0, 100, 100]),   np.array([10, 255, 255]))
            mask_red2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
            mask_red  = cv2.bitwise_or(mask_red1, mask_red2)
            mask_total = cv2.bitwise_or(mask_red, mask_ball)

            M_r     = cv2.moments(mask_red)
            M_balle = cv2.moments(mask_ball)
            msg_twist = Twist()
            msg_twist.linear.x = 0.0
            msg_twist.angular.z = 0.0

            self.get_logger().info(f"Distance: {self.distance:.2f}m | Touche: {self.objet_touche}")

            if not self.objet_touche:
                # CORRECTION 2 : seuil relevé à 500 pour éviter les faux positifs
                if M_balle['m00'] > 500:
                    cx = int(M_balle['m10'] / M_balle['m00'])
                    error = image_center - cx
                    if self.distance <= 0.850 or self.obstacle_detecte:
                        msg_twist.linear.x = 0.0
                        msg_twist.angular.z = 0.0
                        self.objet_touche = True
                    else:
                        
                        msg_twist.linear.x = 0.07
                        msg_twist.angular.z = self.kp * error
                else:
                    msg_twist.angular.z = 0.2

            else:
                if self.poteau_gauche and self.poteau_droit and M_r['m00'] > 500 and M_balle['m00'] > 500:
                    cx_r = int(M_r['m10'] / M_r['m00'])
                    error = image_center - cx_r
                    msg_twist.linear.x = 0.1
                    msg_twist.angular.z = self.kp * error

                
                elif M_balle['m00'] > 500 and not self.poteau_droit and not self.poteau_gauche:
                    cx_b = int(M_balle['m10'] / M_balle['m00'])
                    error_b = image_center - cx_b

                    if self.distance > 1.2:
                        # Encore loin : approche centré sur la balle
                        msg_twist.linear.x = 0.08
                        msg_twist.angular.z = self.kp * error_b

                    elif not self.depassement_en_cours:
                        # À portée : démarre le dépassement par la gauche (fixe)
                        self.depassement_en_cours = True
                        msg_twist.linear.x = 0.07
                        msg_twist.angular.z = 0.5

                    elif self.depassement_en_cours and self.distance > 1.0:
                        # La balle est derrière : stop
                        msg_twist.linear.x = 0.0
                        msg_twist.angular.z = 0.0
                        self.depassement_en_cours = False

                    else:
                        # Continue l'arc
                        msg_twist.linear.x = 0.07
                        msg_twist.angular.z = 0.5
                    
                else:
                    msg_twist.linear.x = 0.0
                    msg_twist.angular.z = 0.3

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