import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp

import mediapipe.python.solutions.hands as mp_hands
import mediapipe.python.solutions.drawing_utils as mp_drawing

class HumanControlNode(Node):
    def __init__(self):
        super().__init__('human_control_node')
        self.publisher = self.create_subscription
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.5)
        self.mp_draw = mp_drawing

        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        success, image = self.cap.read()
        if not success: return

        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image_rgb)

        msg = Twist()

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:

                index_finger = hand_landmarks.landmark[8]

                if index_finger.y < 0.4:
                    msg.linear.x = 0.2  # si index en haut de l'image > Avancer
                elif index_finger.y > 0.6:
                    msg.linear.x = -0.2 # si index en bas de l'image > Reculer
                if index_finger.x < 0.4:
                    msg.angular.z = 0.5 # si index a gauche de l'image > Gauche
                elif index_finger.x > 0.6:
                    msg.angular.z = -0.5 # si index a droite de l'image > Droite

                self.mp_draw.draw_landmarks(image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

        self.cmd_publisher.publish(msg)
        cv2.imshow("Controle Gestuel", image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = HumanControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
