#!/usr/bin/env python

# ============================================ #
#            LIBRERIAS NECESARIAS              #
# ============================================ #

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
import speech_recognition as sr
from gtts import gTTS
import os

# ============================================ #
#              TOPICS Y VARIABLES              #
# ============================================ #

TOPIC_VEL = "/cmd_vel"      # para enviar comandos de velocidad
TOPIC_SCAN = '/scan'        # para recibir datos del láser

# Definición de ángulos de escaneo (radianes)
ANG_IZQ = 30 * math.pi / 180.0
ANG_DER = -ANG_IZQ
AUX = 0

# ============================================ #
#           CLASES/ESTADOS DEFINIDOS           #
# ============================================ #

# Función de síntesis de voz con gTTS
def speak(text):
    tts = gTTS(text=text, lang='es')
    tts.save("output.mp3")
    os.system("mpg321 output.mp3")

# Esta clase es la encargada de reconocer la petición por voz del usuario
# -----------------------------------------------------------------------
#class VoiceControl:
#    def __init__(self):
#        self.recognizer = sr.Recognizer()
#        self.commands = ["klau", "nico", "otto", "iker"]
#
#    def record_audio(self, duration=5):
#        with sr.Microphone() as source:
#            rospy.loginfo("Grabando audio...")
#            self.recognizer.adjust_for_ambient_noise(source)
#            audio = self.recognizer.listen(source, timeout=duration)
#            rospy.loginfo("Grabación finalizada.")
#            return audio
#
#    def recognize_speech(self, audio):
#        try:
#            rospy.loginfo("Reconociendo... ")
#            recognized_text = self.recognizer.recognize_google(audio, language="es-ES")
#            rospy.loginfo(f"Texto reconocido: {recognized_text}")
#            return recognized_text.strip().lower()
#        except sr.UnknownValueError:
#            rospy.logwarn("No se pudo reconocer el audio")
#            return ""
#        except sr.RequestError as e:
#            rospy.logerr(f"Error con el servicio de reconocimiento de voz: {e}")
#            return ""
#
#def recognize_speech():
#    voice_control = VoiceControl()
#    speak("Por favor, diga el nombre de la persona a buscar: klau, nico, otto o iker.")
#    audio = voice_control.record_audio()
#    recognized_text = voice_control.recognize_speech(audio)
#    return recognized_text    


# Estado encargado de moverse a puntos asignados en el entorno
# ------------------------------------------------------------
class GoPoint(State):
    # Constructor de la clase
    def __init__(self, x: float, y: float, w: float, color_topic):
        # Llama al constructor de la clase base 'State', con los posibles resultados 'point_reached', 'failed' y 'color_detected'
        super().__init__(outcomes=['point_reached', 'failed', 'color_detected'])
        
        # Almacena las coordenadas de destino (x, y) y la orientación (w) para e
        self.x = x
        self.y = y
        self.w = w

        # Inicializa la variable que indica si el color ha sido detectado (inicialmente es 'False')
        self.color_detected = False

        # Establece conexión con MoveBase
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Suscripción al topic para la detección del color
        rospy.Subscriber(color_topic, Bool, self.color_callback)

    # Para obtener el mensaje de detección de color
    def color_callback(self, msg):
        self.color_detected = msg.data

    # Ejecución de MoveBase
    def execute(self, userdata):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Utiliza las coordenadas pasadas por parametro
        goal.target_pose.pose.position.x = self.x   
        goal.target_pose.pose.position.y = self.y
        goal.target_pose.pose.orientation.w = self.w

        rospy.loginfo("Movimiento a la habitación específica...")
        self.client.send_goal(goal)


        # El robot se seguirá moviendo mientras no detecte color o no llegue al objetivo 
        while not rospy.is_shutdown():
            # Si se detectó el color durante el movimiento, cancela la meta y regresa el resultado 'color_detected'
            if self.color_detected:
                rospy.loginfo("Color detectado durante el movimiento. Retornando.")
                self.client.cancel_goal()
                return 'color_detected'

            # Si la meta fue alcanzada, regresa el resultado 'point_reached' y cambia de estado
            # Si la meta fue abortada o rechazada, regresa el resultado 'failed'
            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                return 'point_reached'
            elif state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
                return 'failed'

            rospy.sleep(0.1)

        self.client.cancel_goal()
        return 'failed'

# Estado encargado de hacer una vuelta al llegar al punto para buscar a la persona
# --------------------------------------------------------------------------------
class Girar(State):
    # Constructor
    def __init__(self):
        super().__init__(outcomes=['spun'])
        
        # Publicador para enviar mensajes de tipo 'Twist' al tópico de velocidad (TOPIC_VEL)
        self.pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=5)
        
        self.angular_speed = 1.0
        # Calcula el tiempo necesario para realizar una vuelta completa (2*pi radianes) a la velocidad angular dada        
        self.spin_time = 2 * math.pi / self.angular_speed

    # Método que se ejecuta cuando se llama a la acción de girar
    def execute(self, userdata):
        rospy.loginfo("Iniciando una vuelta sobre sí mismo...")
        cmd = Twist()
        cmd.angular.z = self.angular_speed

        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        
        # Mientras no haya transcurrido el tiempo necesario para una vuelta completa, continúa girando
        while rospy.Time.now() - start_time < rospy.Duration(self.spin_time):
            self.pub.publish(cmd)
            rate.sleep()

        # Una vez completada la vuelta, detiene el robot (velocidad angular en 0)
        cmd.angular.z = 0
        self.pub.publish(cmd)
        rospy.loginfo("Vuelta completada.")
        return 'spun'

# Estado encargado de hablar con la persona una vez la haya encontrado
# --------------------------------------------------------------------
class ApproachColor(State):
    # Constructor de la clase
    def __init__(self, color_topic):
        State.__init__(self, outcomes=['encontrado', 'failed'])
        
        self.color_detected = False
        self.color_sub = rospy.Subscriber(color_topic, Bool, self.color_callback)
    
    # Función que maneja los mensajes recibidos del tópico de color
    def color_callback(self, msg):
        global AUX # Variable auxiliar para no repetir el mensaje
        # Si se detecta a la persona, se le avisa mediante voz
        if self.color_detected and AUX == 0:
            rospy.loginfo("Persona encontrada")
            rospy.sleep(5)
            speak("Acompáñame.")
            rospy.sleep(5)
            AUX = 1
        self.color_detected = msg.data

    def execute(self, userdata):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.color_detected:
                self.color_sub.unregister()  # Deja de escuchar al tópico
                self.color_detected = False  # Reinicia la detección
                return 'encontrado'
            rate.sleep()
        return 'failed'

# Estado que permite la vuelta al punto de inicio
# ------------------------------------------------
class GoHome(State):
    def __init__(self):
        super().__init__(outcomes=['home_reached', 'failed'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Coordenadas del punto inicial
        goal.target_pose.pose.position.x = -1.0
        goal.target_pose.pose.position.y = -11.0
        goal.target_pose.pose.orientation.w = 1.0

        speak("Hola! la persona en situación de dependencia te necesita!")
        rospy.loginfo("Regresando a la posición de inicio...")
        self.client.send_goal(goal)

        while not rospy.is_shutdown():
            state = self.client.get_state()
            # Una vez llegado al punto de inicio, se avisa a la persona
            if state == GoalStatus.SUCCEEDED:
                speak("Hemos llegado")
                rospy.sleep(1)
                return 'home_reached'
            elif state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
                return 'failed'
            rospy.sleep(0.1)

        self.client.cancel_goal()
        return 'failed'

# ============================================ #
#           MAIN Y MÁQUINA DE ESTADOS          #
# ============================================ #

if __name__ == '__main__':
    rospy.init_node('navigation_with_states')

    color_choice = None

    # Petición de la persona a buscar
    speak("Ingrese la persona a buscar (klau, nico, otto, iker): ")
    color_choice = input("Ingrese la persona a buscar (klau, nico, otto, iker): ").strip().lower()

    # En función de la persona a la que se busca, se suscribe al topic correspondiente
    color_topic_mapping = {
        "nico": "/green_detected",
        "iker": "/red_detected",
        "klau": "/blue_detected",
        "otto": "/yellow_detected"
    }

    # Si la persona introducida es incorrecta, se acaba el programa y se avisa
    if color_choice not in color_topic_mapping:
        rospy.logerr("Persona no válida. Por favor, elija entre klau, nico, otto o iker")
        exit()

    # Una vez que se elije a la persona, el robot avisa de que va en su búsqueda
    selected_topic = color_topic_mapping[color_choice]
    speak(f"Buscando a {color_choice}.")
    rospy.loginfo(f"Buscando a la persona: {color_choice} en el tópico: {selected_topic}")

    # MÁQUINA DE ESTADOS
    # ------------------
    sm = StateMachine(outcomes=['end'])

    with sm:
        # Se definen los puntos de navegación con sus respectivas transiciones
        StateMachine.add('GoPoint1', GoPoint(9.0, -4.0, 1.0, selected_topic),
                        transitions={
                            'point_reached': 'Girar1',
                            'color_detected': 'ApproachColor1',
                            'failed': 'GoHome'
                        })
        StateMachine.add('ApproachColor1', ApproachColor(selected_topic),
                        transitions={
                            'encontrado': 'GoHome',
                            'failed': 'GoHome'
                        })
        StateMachine.add('Girar1', Girar(),
                        transitions={'spun': 'GoPoint2'})

        StateMachine.add('GoPoint2', GoPoint(13.0, -9.0, 1.0, selected_topic),
                        transitions={
                            'point_reached': 'Girar2',
                            'color_detected': 'ApproachColor2',
                            'failed': 'GoHome'
                        })
        StateMachine.add('ApproachColor2', ApproachColor(selected_topic),
                        transitions={
                            'encontrado': 'GoHome',
                            'failed': 'GoHome'
                        })
        StateMachine.add('Girar2', Girar(),
                        transitions={'spun': 'GoPoint3'})

        StateMachine.add('GoPoint3', GoPoint(5.0, -2.0, 1.0, selected_topic),
                        transitions={
                            'point_reached': 'Girar3',
                            'color_detected': 'ApproachColor3',
                            'failed': 'GoHome'
                        })
        StateMachine.add('ApproachColor3', ApproachColor(selected_topic),
                        transitions={
                            'encontrado': 'GoHome',
                            'failed': 'GoHome'
                        })
        StateMachine.add('Girar3', Girar(),
                        transitions={'spun': 'GoPoint4'})

        StateMachine.add('GoPoint4', GoPoint(6.0, 3.0, 1.0, selected_topic),
                        transitions={
                            'point_reached': 'Girar4',
                            'color_detected': 'ApproachColor4',
                            'failed': 'GoHome'
                        })
        StateMachine.add('ApproachColor4', ApproachColor(selected_topic),
                        transitions={
                            'encontrado': 'GoHome',
                            'failed': 'GoHome'
                        })
        StateMachine.add('Girar4', Girar(),
                        transitions={'spun': 'GoPoint5'})

        StateMachine.add('GoPoint5', GoPoint(-1.0, 4.0, 1.0, selected_topic),
                        transitions={
                            'point_reached': 'Girar5',
                            'color_detected': 'ApproachColor5',
                            'failed': 'GoHome'
                        })
        StateMachine.add('ApproachColor5', ApproachColor(selected_topic),
                        transitions={
                            'encontrado': 'GoHome',
                            'failed': 'GoHome'
                        })
        StateMachine.add('Girar5', Girar(),
                        transitions={'spun': 'GoPoint6'})

        StateMachine.add('GoPoint6', GoPoint(-4.0, 0.0, 1.0, selected_topic),
                        transitions={
                            'point_reached': 'Girar6',
                            'color_detected': 'ApproachColor6',
                            'failed': 'GoHome'
                        })
        StateMachine.add('ApproachColor6', ApproachColor(selected_topic),
                        transitions={
                            'encontrado': 'GoHome',
                            'failed': 'GoHome'
                        })
        StateMachine.add('Girar6', Girar(),
                        transitions={'spun': 'GoPoint7'})

        StateMachine.add('GoPoint7', GoPoint(1.0, -4.0, 1.0, selected_topic),
                        transitions={
                            'point_reached': 'Girar7',
                            'color_detected': 'ApproachColor7',
                            'failed': 'GoHome'
                        })
        StateMachine.add('ApproachColor7', ApproachColor(selected_topic),
                        transitions={
                            'encontrado': 'GoHome',
                            'failed': 'GoHome'
                        })
        StateMachine.add('Girar7', Girar(),
                        transitions={'spun': 'GoPoint8'})

        StateMachine.add('GoPoint8', GoPoint(-3.0, 0.0, 1.0, selected_topic),
                        transitions={
                            'point_reached': 'end',
                            'color_detected': 'ApproachColor8',
                            'failed': 'GoHome'
                        })
        StateMachine.add('ApproachColor8', ApproachColor(selected_topic),
                        transitions={
                            'encontrado': 'GoHome',
                            'failed': 'GoHome'
                        })
        StateMachine.add('GoHome', GoHome(),
                 transitions={
                     'home_reached': 'end',
                     'failed': 'end'
                 })

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    sm.execute()
    rospy.spin()

