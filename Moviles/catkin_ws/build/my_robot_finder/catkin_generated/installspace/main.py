#!/usr/bin/env python3

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

TOPIC_VEL = "/cmd_vel"
TOPIC_SCAN = '/scan'

# Definición de ángulos de escaneo (radianes)
ANG_IZQ = 30 * math.pi / 180.0
ANG_DER = -ANG_IZQ
AUX = 0

# Función de síntesis de voz con gTTS
def speak(text):
    tts = gTTS(text=text, lang='es')
    tts.save("output.mp3")
    os.system("mpg321 output.mp3")

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

class Girar(State):
    def __init__(self):
        super().__init__(outcomes=['spun'])
        self.pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=5)
        self.angular_speed = 1.0
        self.spin_time = 2 * math.pi / self.angular_speed

    def execute(self, userdata):
        rospy.loginfo("Iniciando una vuelta sobre sí mismo...")
        cmd = Twist()
        cmd.angular.z = self.angular_speed

        start_time = rospy.Time.now()
        rate = rospy.Rate(10)

        while rospy.Time.now() - start_time < rospy.Duration(self.spin_time):
            self.pub.publish(cmd)
            rate.sleep()

        cmd.angular.z = 0
        self.pub.publish(cmd)
        rospy.loginfo("Dos vueltas completas realizadas.")
        return 'spun'

class ApproachColor(State):
    def __init__(self, color_topic):
        State.__init__(self, outcomes=['encontrado', 'failed'])
        self.color_detected = False
        self.color_sub = rospy.Subscriber(color_topic, Bool, self.color_callback)

    def color_callback(self, msg):
        global AUX
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

class GoHome(State):
    def __init__(self):
        super().__init__(outcomes=['home_reached', 'failed'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = -1.0
        goal.target_pose.pose.position.y = -11.0
        goal.target_pose.pose.orientation.w = 1.0

        speak("Hola! la persona en situación de dependencia te necesita!")
        rospy.loginfo("Regresando a la posición de inicio...")
        self.client.send_goal(goal)

        while not rospy.is_shutdown():
            state = self.client.get_state()
            if state == GoalStatus.SUCCEEDED:
                speak("Hemos llegado")
                rospy.sleep(1)
                return 'home_reached'
            elif state in [GoalStatus.ABORTED, GoalStatus.REJECTED]:
                return 'failed'
            rospy.sleep(0.1)

        self.client.cancel_goal()
        return 'failed'

if __name__ == '__main__':
    rospy.init_node('navigation_with_states')

    color_choice = None

    speak("Ingrese la persona a buscar (klau, nico, otto, iker): ")
    color_choice = input("Ingrese la persona a buscar (klau, nico, otto, iker): ").strip().lower()


    color_topic_mapping = {
        "nico": "/green_detected",
        "iker": "/red_detected",
        "klau": "/blue_detected",
        "otto": "/yellow_detected"
    }

    if color_choice not in color_topic_mapping:
        rospy.logerr("Persona no válida. Por favor, elija entre klau, nico, otto o iker")
        exit()

    selected_topic = color_topic_mapping[color_choice]
    speak(f"Buscando a {color_choice}.")
    rospy.loginfo(f"Buscando a la persona: {color_choice} en el tópico: {selected_topic}")


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

