import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage,LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from cv_bridge import CvBridge

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower') # initialisation du node line follower

        self.subscription = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.listener_callback, 10) # pour le robot reel
        #self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 10) # pour la simu

        self.image_publisher = self.create_publisher(
            Image,
            '/processed_image',
            10)
        
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10) # publisher pour faire bouger le robot
        
        self.cv_bridge = CvBridge()

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.obstacle_detecte = False
        self.distance = 0

        # Parametres PID
        self.kp = 0.004 # Proportionnel : force de correction
        self.ki = 0   # Integral : correction d'erreur statique
        self.kd = 0.0025  # Derivatif : amortisseur d'oscillations
        
        # Memoire du PID
        self.last_error = 0.0
        self.integral = 0.0
        self.max_angular_vel = 1.0 # Limite de rotation pour eviter les tete-a-queue

        self.rondpoint_gauche = False # Changer par True si gauche ou par False si droite
        self.rondpoint_detecte = False

    def scan_callback(self, msg):
        points_devant = [r for r in msg.ranges[0:20] + msg.ranges[340:359] if r > 0.05]
        
        if points_devant:
            distance_min = min(points_devant)
            if distance_min <= 0.2:
                self.obstacle_detecte = True
                self.distance = distance_min
            else:
                self.obstacle_detecte = False


    def listener_callback(self, msg):
        self.get_logger().info("Image ok") # Optionnel : sature les logs sinon

        try:
            # Conversion de l'image ROS en OpenCV
            cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8') #pour le robot reel 
            #cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # pour la simu
        except Exception as e:
            self.get_logger().error(f"Erreur a la conversion de l'image: {e}")
            return
        
        if cv_image is not None:

            # Pretraitement
            self.get_logger().info("Image ok")
            height = cv_image.shape[0]
            width = cv_image.shape[1]
            image_center = width / 2
            
            # ROI : On ne garde que le bas de l'image
            cv_image_roi = cv_image[int(height * 0.28):height, :] 
            cv_image_roi2 = cv_image[0:int(height * 0.40), :]

            hsv = cv2.cvtColor(cv_image_roi, cv2.COLOR_BGR2HSV) # conversion de BGR a HSV

            # Masque Vert
            mask_green = cv2.inRange(hsv, np.array([40, 70, 70]), np.array([100, 255, 255]))

            # Masque Rouge (avec les deux plages HSV)
            mask_red1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
            mask_red2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)

            msg_twist = Twist() # instance de type Twist pour la vitesse du robot

            # Calcul des moments pour chaque couleur
            M_v = cv2.moments(mask_green)
            M_r = cv2.moments(mask_red)
            mask_total = cv2.bitwise_or(mask_red, mask_green)
            

            ###############################

            hsv2 = cv2.cvtColor(cv_image_roi2, cv2.COLOR_BGR2HSV) # conversion de BGR a HSV

            # Masque Vert
            mask_green_2 = cv2.inRange(hsv2, np.array([40, 70, 70]), np.array([100, 255, 255]))

            # Masque Rouge (avec les deux plages HSV)
            mask_red11 = cv2.inRange(hsv2, np.array([0, 100, 100]), np.array([10, 255, 255]))
            mask_red21 = cv2.inRange(hsv2, np.array([160, 100, 100]), np.array([180, 255, 255]))
            mask_red_2 = cv2.bitwise_or(mask_red11, mask_red21)

            msg_twist = Twist() # instance de type Twist pour la vitesse du robot

            # Calcul des moments pour chaque couleur
            M_v2 = cv2.moments(mask_green_2)
            M_r2 = cv2.moments(mask_red_2)
            mask_total2 = cv2.bitwise_or(mask_red_2, mask_green_2)



            target_x = None
            self.get_logger().info(f"Pixels blancs vus : {M_v['m00']}")
                        
            if M_v2['m00']>500 and M_r2['m00']>500:
                if int(M_v2['m10'] / M_v2['m00'])> int(M_r2['m10'] / M_r2['m00']): 
                    rondpoint_detecte = True
                    self.get_logger().warn("Rond point detecté !")

            if self.obstacle_detecte:
                msg_twist.linear.x = 0.0
                msg_twist.angular.z = 0.0
                self.get_logger().warn("ARRET : obstacle devant !")
            
            else : 

                if self.rondpoint_detecte:
                    msg_twist.linear.x = 0.05 
                    
                    if M_v['m00'] > 200 and M_r['m00'] > 200:
                        cx_v = M_v['m10'] / M_v['m00']
                        cx_r = M_r['m10'] / M_r['m00']
                        
                        target_x = (cx_v + cx_r) / 2
                        error = image_center - target_x

                        if self.rondpoint_gauche:
                            msg_twist.angular.z = (self.kp * error) + 0.3
                        else:
                            msg_twist.angular.z = (self.kp * error) - 0.3


                else : 

                    # LOGIQUE DE DECISION DU POINT CIBLE (target_x)
                    if M_v['m00'] > 0 and M_r['m00'] > 0:
                        # On voit les deux : cible = milieu
                        cx_v = M_v['m10'] / M_v['m00']
                        cx_r = M_r['m10'] / M_r['m00']
                        target_x = (cx_v + cx_r) / 2

                        msg_twist.linear.x = 0.1 # Plus rapide quand on est bien entre les deux
                    
                    elif M_v['m00'] > 0:
                        # On ne voit que la gauche : on vise 150 pixels a droite de la ligne verte
                        target_x = (M_v['m10'] / M_v['m00']) + 80
                        msg_twist.linear.x = 0.05
                    
                    elif M_r['m00'] > 0:
                        # On ne voit que la droite : on vise 150 pixels a gauche de la ligne rouge
                        target_x = (M_r['m10'] / M_r['m00']) - 80
                        msg_twist.linear.x = 0.05

                    # CALCUL DU PID SI UNE CIBLE EST TROUVeE
                    if target_x is not None:
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
                        
                        # Dessin du centre pour le debug
                        cv2.circle(cv_image_roi, (int(target_x), int(cv_image_roi.shape[0]/2)), 10, (0,255,0), -1)
                    else:
                        # Si on ne voit rien du tout : on cherche en tournant
                        msg_twist.linear.x = 0.0
                        msg_twist.angular.z = 0.3
                        self.integral = 0.0 # Reset l'integrale pour eviter l'accumulation hors ligne

            # Publication des commandes et de l'image traitee (Désindenté pour être hors du else)
            self.vel_publisher.publish(msg_twist)
            ros_img = self.cv_bridge.cv2_to_imgmsg(cv_image_roi, encoding="bgr8")
            self.image_publisher.publish(ros_img)

            # Fenetres de visualisation
            try : 
                #cv2.imshow("Detection Ligne", cv_image_roi)
                cv2.imshow("Masque Total", mask_total)
                cv2.waitKey(1)
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)  
    node = LineFollower() # instancie le noeud
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