#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class GreenColorDetector:
    def __init__(self):
        rospy.init_node('green_color_detector', anonymous=True)
        
        # Inicializar el bridge para convertir entre ROS y OpenCV
        self.bridge = CvBridge()
        
        # Publicar en los tópicos
        self.color_pub = rospy.Publisher('/green_detected', Bool, queue_size=10)        
        # Suscribirse al tópico de la cámara simulada
        self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)
    
    def image_callback(self, msg):
            # Convertir la imagen ROS a OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Convertir la imagen de BGR a HSV
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Definir el rango de color verde en HSV
            lower_green = np.array([35, 100, 100])  # Rango inferior para verde
            upper_green = np.array([85, 255, 255])  # Rango superior para verde
            
            # Crear la máscara para detectar el verde
            green_mask = cv2.inRange(hsv_image, lower_green, upper_green)
            
            # Calcular cuántos píxeles verdes hay
            green_pixels = cv2.countNonZero(green_mask)
            
            # Determinar si el color verde está presente
            if green_pixels > 1200:
                self.color_pub.publish(True)
                rospy.loginfo("¡Nico detectado!")
                

if __name__ == '__main__':
    # Crear y ejecutar el objeto detector
    detector = GreenColorDetector()
    
    # Mantener el nodo funcionando
    rospy.spin()

