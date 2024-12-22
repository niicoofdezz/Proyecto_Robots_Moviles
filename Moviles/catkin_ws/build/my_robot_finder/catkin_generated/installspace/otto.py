#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class YellowColorDetector:
    def __init__(self):
        rospy.init_node('yellow_color_detector', anonymous=True)
        
        # Inicializar el bridge para convertir entre ROS y OpenCV
        self.bridge = CvBridge()
        
        # Publicar en el tópico '/color_detected'
        self.color_pub = rospy.Publisher('/yellow_detected', Bool, queue_size=10)
        # Suscribirse al tópico de la cámara
        self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)
    
    def image_callback(self, msg):
            # Convertir la imagen ROS a OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Convertir la imagen de BGR a HSV
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Definir el rango de color amarillo en HSV
            lower_yellow = np.array([20, 100, 100])  # Rango inferior ajustado
            upper_yellow = np.array([30, 255, 255])  # Rango superior ajustado
            
            # Crear la máscara para detectar el amarillo
            yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
            
            # Aplicar operaciones morfológicas para eliminar ruido
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
            yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
            
            # Calcular cuántos píxeles amarillos hay
            yellow_pixels = cv2.countNonZero(yellow_mask)
            
            # Publicar True si se detecta amarillo, False si no
            if yellow_pixels > 1200:
                self.color_pub.publish(True)
                rospy.loginfo(f"¡Otto encontrado!")

if __name__ == '__main__':
    # Crear y ejecutar el objeto detector
    detector = YellowColorDetector()
    
    # Mantener el nodo funcionando
    rospy.spin()

