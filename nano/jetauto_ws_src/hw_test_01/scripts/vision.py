#!/usr/bin/env python3
# encoding: utf-8
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class ObjectFollower:
    def __init__(self):
        rospy.init_node('object_follower', anonymous=True)
        self.bridge = CvBridge()
        
        # Suscriptor para la imagen de la cámara del brazo del JetAuto Pro
        self.image_sub = rospy.Subscriber('/usb_camera/image_raw', Image, self.image_callback)
        
        # Publicador para la imagen procesada
        self.image_pub = rospy.Publisher('/object_follower/image_processed', Image, queue_size=1)
        
        # Publicador para comandos de movimiento (tópico estándar en ROS)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Parámetros para la detección de color (ajusta estos valores según el objeto)
        self.lower_color = np.array([30, 50, 50])  # Valor mínimo de color (H, S, V)
        self.upper_color = np.array([90, 255, 255])  # Valor máximo de color (H, S, V)
        
        # Tamaño mínimo del objeto para considerarlo válido
        self.min_area = 500

    def image_callback(self, data):
        try:
            # Convertir la imagen de ROS a OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Convertir la imagen a espacio de color HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Crear una máscara para el rango de color especificado
        mask = cv2.inRange(hsv_image, self.lower_color, self.upper_color)

        # Encontrar contornos en la máscara
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Si se detectan contornos
        if contours:
            # Encontrar el contorno más grande
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            # Si el área es mayor que el mínimo especificado
            if area > self.min_area:
                # Obtener el rectángulo delimitador del contorno
                x, y, w, h = cv2.boundingRect(largest_contour)

                # Dibujar un rectángulo alrededor del objeto
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Calcular el centro del objeto
                center_x = x + w // 2
                center_y = y + h // 2

                # Dibujar un círculo en el centro del objeto
                cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)

                # Mostrar la posición del objeto en la pantalla
                cv2.putText(cv_image, f"X: {center_x}, Y: {center_y}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                # Mover el robot hacia el objeto
                self.move_towards_object(center_x, cv_image.shape[1])

        # Publicar la imagen procesada
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        except CvBridgeError as e:
            rospy.logerr(e)

    def move_towards_object(self, object_x, image_width):
        twist = Twist()

        # Calcular la diferencia entre el centro del objeto y el centro de la imagen
        center_image = image_width // 2
        error = object_x - center_image

        # Control proporcional para mover el robot
        if abs(error) > 20:  # Solo mover si el error es significativo
            twist.angular.z = -float(error) / 100  # Ajusta la velocidad angular
            twist.linear.x = 0.2  # Velocidad lineal constante
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Publicar el comando de velocidad
        self.cmd_vel_pub.publish(twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        follower = ObjectFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
