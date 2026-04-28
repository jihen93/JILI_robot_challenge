import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from cv_bridge import CvBridge

class GoalNode(Node):
    def __init__(self): # Correction : Ajout de la définition de la fonction
        super().__init__('goal_node') 

        self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
        self.image_publisher = self.create_publisher(Image, '/processed_image', 10)
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.cv_bridge = CvBridge()
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.obstacle_detecte = False
        self.distance = 0.0

         # Parametres PID
        self.kp = 0.004   # Proportionnel : force de correction
        self.ki = 0   # Integral : correction d'erreur statique
        self.kd = 0.0025   # Derivatif : amortisseur d'oscillations
        
        # Memoire du PID
        self.last_error = 0.0
        self.integral = 0.0
        self.max_angular_vel = 1.0 # Limite de rotation pour eviter les tete-a-queue
        self.objet_touche = False

    def scan_callback(self, msg):
        points_devant = [r for r in msg.ranges[0:20] + msg.ranges[340:359] if r > 0.05]
        if points_devant:
            distance_min = min(points_devant)
            if distance_min <= 0.8:
                self.obstacle_detecte = True
                self.distance = distance_min
            else:
                self.obstacle_detecte = False

    def listener_callback(self, msg):


        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Erreur conversion: {e}")
            return
        
        if cv_image is not None:
            self.get_logger().info("Image ok")
            height = cv_image.shape[0]
            width = cv_image.shape[1]
            image_center = width / 2

            cv_image_roi = cv_image[int(height * 0.1):height, :]
            hsv = cv2.cvtColor(cv_image_roi, cv2.COLOR_BGR2HSV)

            # --- DETECTION BALLE (JAUNE) ---
            lower_ball = np.array([25, 80, 60])
            upper_ball = np.array([50, 255, 255])
            mask_ball = cv2.inRange(hsv, lower_ball, upper_ball)
            
            # --- DETECTION BUT (ROUGE) ---
            mask_red1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
            mask_red2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)

            msg_twist = Twist()
            M_r = cv2.moments(mask_red)
            mask_total = cv2.bitwise_or(mask_red, mask_ball)

            M_r = cv2.moments(mask_red)
            M_balle = cv2.moments(mask_ball)

            target_x = None


            if self.obstacle_detecte and M_balle['m00'] > 50:
                target_x = int(M_balle['m10'] / M_balle['m00'])
                msg_twist.linear.x = 0.5
                #msg_twist.angular.z = 0
            
                error = image_center - target_x # Erreur de trajectoire
                    
                # Calculs PID
                P = self.kp * error
                self.integral += error
                I = self.ki * self.integral
                D = self.kd * (error - self.last_error)
                    
                output_z = P + I + D
                    
                # Limitation de la vitesse de rotation
                msg_twist.angular.z = max(min(output_z, self.max_angular_vel), -self.max_angular_vel)
                self.last_error = error
                
            elif self.obstacle_detecte and M_balle['m00']< 0:
                msg_twist.linear.x = 0.0
                msg_twist.angular.z = 0.5 # On tourne pour l'éviter
                    
                # Dessin du centre pour le debug
                cv2.circle(cv_image_roi, (int(target_x), int(cv_image_roi.shape[0]/2)), 10, (0,255,0), -1)
            else:
                # Si on ne voit rien du tout : on cherche en tournant
                msg_twist.linear.x = 0.0
                msg_twist.angular.z = 0.3
                self.integral = 0.0 # Reset l'integrale pour eviter l'accumulation hors ligne   
                
                

            # Publication
            self.vel_publisher.publish(msg_twist)
            ros_img = self.cv_bridge.cv2_to_imgmsg(cv_image_roi, encoding="bgr8")
            self.image_publisher.publish(ros_img)

            # Fenetres de visualisation
            try : 
                ##cv2.imshow("Detection but", cv_image_roi)
                cv2.imshow("Masque Total", mask_total)
                cv2.waitKey(1)
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)  
    node = GoalNode() # instancie le noeud
    try:
        rclpy.spin(node) # run le node en boucle 
    except KeyboardInterrupt :
        pass
    finally : 
        if rclpy.ok():
            node.vel_publisher.publish(Twist()) # Arrete le robot avant de couper
            node.destroy_node() # destruction du noeud
            rclpy.shutdown() # eteindre
        cv2.destroyAllWindows() # fermer la fenetre 

if __name__ == '__main__':
    main()