#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class RedColorDetector:
    def __init__(self):
        rospy.init_node('red_color_detector', anonymous=True)
        
        # Inicializar el bridge para convertir entre ROS y OpenCV
        self.bridge = CvBridge()
        
        # Publicar en el tópico '/color_detected'
        self.color_pub = rospy.Publisher('/red_detected', Bool, queue_size=10)
        # Suscribirse al tópico de la cámara
        self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)
    
    def image_callback(self, msg):
            # Convertir la imagen ROS a OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Convertir la imagen de BGR a HSV
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Definir el rango de color rojo en HSV
            # Ajustar los rangos de color rojo en HSV
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])
            
            # Crear la máscara para detectar el rojo
            red_mask = cv2.inRange(hsv_image, lower_red, upper_red)
            
            # Detectar el rojo en el rango superior (H > 160)
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])
            red_mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
            
            # Combinar ambas máscaras
            red_mask = red_mask | red_mask2
            
            # Calcular cuántos píxeles rojos hay
            red_pixels = cv2.countNonZero(red_mask)
            
            # Publicar True si se detecta rojo, False si no
            if red_pixels > 1200:
                self.color_pub.publish(True)
                rospy.loginfo("¡Iker encontrado!")


if __name__ == '__main__':
    # Crear y ejecutar el objeto detector
    detector = RedColorDetector()
    
    # Mantener el nodo funcionando
    rospy.spin()

