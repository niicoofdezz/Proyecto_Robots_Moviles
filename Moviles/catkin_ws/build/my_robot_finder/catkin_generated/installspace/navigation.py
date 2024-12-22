import rospy
import smach_ros
import math
from smach import State, StateMachine
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, Int32
import random
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import speech_recognition as sr  # Librería para reconocimiento de voz
import pyttsx3  # Librería para síntesis de voz

TOPIC_VEL = "/cmd_vel"              
TOPIC_SCAN = '/scan'            

# Definición de ángulos de escaneo (radianes)
ANG_IZQ = 30 * math.pi / 180.0
ANG_DER = -ANG_IZQ

# Inicializar el motor de síntesis de voz
engine = pyttsx3.init()
engine.setProperty('rate', 150)  # Velocidad de la voz
engine.setProperty('volume', 1)  # Volumen (de 0 a 1)

# Función para hacer hablar al robot
def speak(message):
    engine.say(message)
    engine.runAndWait()


class GoPoint(State):
    def __init__(self, x: float, y: float, w: float, color_topic):
        super().__init__(outcomes=['point_reached', 'failed', 'color_detected'])
        self.x = x
        self.y = y
        self.w = w
        self.color_detected = False
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.Subscriber(color_topic, Bool, self.color_callback)

    def color_callback(self, msg):
        self.color_detected = msg.data

    def execute(self, userdata):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.x
        goal.target_pose.pose.position.y = self.y
        goal.target_pose.pose.orientation.w = self.w

        rospy.loginfo("Movimiento a la habitación específica...")
        self.client.send_goal(goal)

        while not rospy.is_shutdown():
            if self.color_detected:
                rospy.loginfo("Color detectado durante el movimiento. Retornando.")
                self.client.cancel_goal()
                return 'color_detected'

            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                return 'point_reached'
            elif state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
                return 'failed'

            rospy.sleep(0.1)

        self.client.cancel_goal()
        return 'failed'


# estado que realiza un giro sobre el robot
class Girar(State):
    def __init__(self):
        State.__init__(self, outcomes=['spun']) # define el estado y una posible salida
        self.pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=5) # publicador de velocidades
        
        self.angular_speed = 1.0                           # velocidad angular en rad/s
        self.spin_time = 2 * math.pi / self.angular_speed  # tiempo para 2 vueltas completas

    def execute(self, userdata):
        rospy.loginfo("Iniciando una vuelta sobre sí mismo...")
        cmd = Twist()
        cmd.angular.z = self.angular_speed # define la velocidad angular

        # tiempo inicial
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10 Hz

        # mientras el tiempo sea menor al calculado en la ecuación
        while rospy.Time.now() - start_time < rospy.Duration(self.spin_time):
            self.pub.publish(cmd)   # el robot girará
            rate.sleep()

        # detener el movimiento al finalizar
        cmd.angular.z = 0
        self.pub.publish(cmd)
        rospy.loginfo("Dos vueltas completas realizadas.")
        return 'spun' # transición al siguiente estado

class ApproachColor(State):
    def __init__(self, color_topic, scan_topic=TOPIC_SCAN, camera_topic='/object_position', distance_threshold=0.5, image_width=640):
        super().__init__(outcomes=['object_reached', 'failed'])
        self.color_detected = False
        self.scan_data = None
        self.left_pixels = 0
        self.right_pixels = 0
        self.distance_threshold = distance_threshold
        self.image_width = image_width
        self.pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=5)
        
        rospy.Subscriber(color_topic, Bool, self.color_callback)
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)
        rospy.Subscriber(camera_topic, Int32, self.camera_callback)

    def color_callback(self, msg):
        self.color_detected = msg.data

    def scan_callback(self, msg):
        self.scan_data = msg.ranges

    def camera_callback(self, msg):
        # Suponiendo que msg.data contiene la posición del centroide del objeto detectado
        self.left_pixels = msg.data['left']  # Píxeles a la izquierda
        self.right_pixels = msg.data['right']  # Píxeles a la derecha

    def execute(self, userdata):
        rate = rospy.Rate(10)
        cmd = Twist()

        rospy.loginfo("Aproximándose al objeto detectado...")
        while not rospy.is_shutdown():
            if not self.color_detected:
                rospy.loginfo("El color ya no está presente. Deteniendo.")
                cmd.linear.x = 0
                cmd.angular.z = 0
                self.pub.publish(cmd)
                return 'failed'

            if self.scan_data:
                # Calcular la distancia frontal
                front_distance = min(self.scan_data[len(self.scan_data)//3:2*len(self.scan_data)//3])
                rospy.loginfo(f"Distancia frontal: {front_distance}")

                if front_distance < self.distance_threshold:
                    rospy.loginfo("Distancia al objeto alcanzada. Deteniendo.")
                    cmd.linear.x = 0
                    cmd.angular.z = 0
                    self.pub.publish(cmd)
                    return 'object_reached'

            # Control para centrar el objeto usando la diferencia de píxeles
            pixel_difference = self.left_pixels - self.right_pixels
            error_angular = pixel_difference / self.image_width  # Error proporcional a la diferencia de píxeles
            cmd.angular.z = -0.1 * error_angular  # Factor de ajuste para corregir el giro

            # Mantén una velocidad constante hacia adelante
            cmd.linear.x = 1.0

            self.pub.publish(cmd)
            rate.sleep()

        cmd.linear.x = 0
        cmd.angular.z = 0
        self.pub.publish(cmd)
        return 'failed'




class GoHome(State):
    def __init__(self):
        State.__init__(self, outcomes=['home_reached', 'failed'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = -1.0
        goal.target_pose.pose.position.y = -11.0
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Regresando a la posición de inicio...")
        self.client.send_goal(goal)

        while not rospy.is_shutdown():
            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                return 'home_reached'
            elif state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
                return 'failed'
            rospy.sleep(0.1)

        self.client.cancel_goal()
        return 'failed'
def listen_for_color():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("¿Necesitas a alguien?")
        speak("¿Necesitas a alguien?")
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

    try:
        command = recognizer.recognize_google(audio, language="es-ES").lower()
        print(f"Comando recibido: {command}")
        return command
    except sr.UnknownValueError:
        print("No pude entender lo que dijo.")
        return None
    except sr.RequestError:
        print("Error al conectar con el servicio de reconocimiento de voz.")
        return None


if __name__ == '__main__':
    rospy.init_node('navigation_with_states')

    engine = pyttsx3.init()

    # Configurar la velocidad y volumen de la voz
    engine.setProperty('rate', 150)  # Velocidad de la voz
    engine.setProperty('volume', 1)  # Volumen (de 0 a 1)

    # Obtener todas las voces disponibles
    voices = engine.getProperty('voices')

    # Establecer la voz en español (puede variar según el sistema)
    for voice in voices:
        if "spanish" in voice.name.lower():
            engine.setProperty('voice', voice.id)
            break

    # Función para hablar en español
    def speak(message):
        engine.say(message)
        engine.runAndWait()
    # Escuchar por voz para obtener el nombre
    color_choice = listen_for_color()

    if not color_choice:
        rospy.logerr("No se pudo recibir un nombre válido por comando de voz.")
        exit()

    # Decir "Buscando..." antes de comenzar a buscar
    speak("Buscando a {command}")

    # Definir el tópico basado en la elección
    color_topic_mapping = {
        "nico": "/green_detected",
        "iker": "/red_detected",
        "klau": "/blue_detected",
        "otto": "/yellow_detected"
    }

    if color_choice not in color_topic_mapping:
        rospy.logerr("Color no válido. Por favor, elija entre klau, nico, otto o iker.")
        exit()

    selected_topic = color_topic_mapping[color_choice]
    rospy.loginfo(f"Buscando a la persona: {color_choice} en el tópico: {selected_topic}")

    sm = StateMachine(outcomes=['end'])

    with sm:
        StateMachine.add('GoPoint1', GoPoint(9.0, -4.0, 1.0, selected_topic),
                         transitions={
                             'point_reached': 'Girar1',
                             'color_detected': 'ApproachColor1',
                             'failed': 'GoHome'
                         })
        StateMachine.add('ApproachColor1', ApproachColor(selected_topic, TOPIC_SCAN),
                         transitions={
                             'object_reached': 'GoHome',
                             'failed': 'GoHome',
                         })
        StateMachine.add('Girar1', Girar(),
                         transitions={'spun': 'GoPoint2'})

        StateMachine.add('GoPoint2', GoPoint(13.0, -9.0, 1.0, selected_topic),
                         transitions={
                             'point_reached': 'Girar2',
                             'color_detected': 'ApproachColor2',
                             'failed': 'GoHome'
                         })
        StateMachine.add('ApproachColor2', ApproachColor(selected_topic, TOPIC_SCAN),
                         transitions={
                             'object_reached': 'GoHome',
                             'failed': 'GoHome',
                         })
        StateMachine.add('Girar2', Girar(),
                         transitions={'spun': 'GoPoint3'})

        StateMachine.add('GoPoint3', GoPoint(5.0, -2.0, 1.0, selected_topic),
                         transitions={
                             'point_reached': 'Girar3',
                             'color_detected': 'ApproachColor3',
                             'failed': 'GoHome'
                         })
        StateMachine.add('ApproachColor3', ApproachColor(selected_topic, TOPIC_SCAN),
                         transitions={
                             'object_reached': 'GoHome',
                             'failed': 'GoHome',
                         })
        StateMachine.add('Girar3', Girar(),
                         transitions={'spun': 'GoPoint4'})

        StateMachine.add('GoPoint4', GoPoint(6.0, 3.0, 1.0, selected_topic),
                         transitions={
                             'point_reached': 'Girar4',
                             'color_detected': 'ApproachColor4',
                             'failed': 'GoHome'
                         })
        StateMachine.add('ApproachColor4', ApproachColor(selected_topic, TOPIC_SCAN),
                         transitions={
                             'object_reached': 'GoHome',
                             'failed': 'GoHome',
                         })
        StateMachine.add('Girar4', Girar(),
                         transitions={'spun': 'GoPoint5'})

        StateMachine.add('GoPoint5', GoPoint(-1.0, 4.0, 1.0, selected_topic),
                         transitions={
                             'point_reached': 'Girar5',
                             'color_detected': 'ApproachColor5',
                             'failed': 'GoHome'
                         })
        StateMachine.add('ApproachColor5', ApproachColor(selected_topic, TOPIC_SCAN),
                         transitions={
                             'object_reached': 'GoHome',
                             'failed': 'GoHome',
                         })
        StateMachine.add('Girar5', Girar(),
                         transitions={'spun': 'GoPoint6'})

        StateMachine.add('GoPoint6', GoPoint(-4.0, 0.0, 1.0, selected_topic),
                         transitions={
                             'point_reached': 'Girar6',
                             'color_detected': 'ApproachColor6',
                             'failed': 'GoHome'
                         })
        StateMachine.add('ApproachColor6', ApproachColor(selected_topic, TOPIC_SCAN),
                         transitions={
                             'object_reached': 'GoHome',
                             'failed': 'GoHome',
                         })
        StateMachine.add('Girar6', Girar(),
                         transitions={'spun': 'GoPoint7'})

        StateMachine.add('GoPoint7', GoPoint(1.0, -4.0, 1.0, selected_topic),
                         transitions={
                             'point_reached': 'Girar7',
                             'color_detected': 'ApproachColor7',
                             'failed': 'GoHome'
                         })
        StateMachine.add('ApproachColor7', ApproachColor(selected_topic, TOPIC_SCAN),
                         transitions={
                             'object_reached': 'GoHome',
                             'failed': 'GoHome',
                         })
        StateMachine.add('Girar7', Girar(),
                         transitions={'spun': 'GoHome'})

        StateMachine.add('GoHome', GoHome(),
                         transitions={
                             'home_reached': 'end',
                             'failed': 'end'
                         })
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    sm.execute()
    rospy.spin()
