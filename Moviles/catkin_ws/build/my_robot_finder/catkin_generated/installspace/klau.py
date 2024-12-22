#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class BlueColorDetector:
    def __init__(self):
        rospy.init_node('blue_color_detector', anonymous=True)
        
        # Inicializar el bridge para convertir entre ROS y OpenCV
        self.bridge = CvBridge()
        
        # Publicar en los tópicos
        self.color_pub = rospy.Publisher('/blue_detected', Bool, queue_size=10)
        # Suscribirse al tópico de la cámara simulada
        self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)
        
    def image_callback(self, msg):

            # Convertir la imagen ROS a OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Convertir la imagen de BGR a HSV
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Definir el rango de color azul en HSV
            lower_blue = np.array([100, 100, 100])  # Rango inferior para azul
            upper_blue = np.array([130, 255, 255])  # Rango superior para azul
            
            # Crear la máscara para detectar el azul
            blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
        
            # Calcular cuántos píxeles azules hay
            blue_pixels = cv2.countNonZero(blue_mask)
            
            # Determinar si el color azul está presente
            if blue_pixels > 1200:
                rospy.loginfo("¡Klau detectada!")
                self.color_pub.publish(True)               
                
    
if __name__ == '__main__':
    # Crear y ejecutar el objeto detector
    detector = BlueColorDetector()
    
    # Mantener el nodo funcionando
    rospy.spin()
