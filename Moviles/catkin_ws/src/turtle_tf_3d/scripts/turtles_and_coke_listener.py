#!/usr/bin/env python
import sys
import rospy
import math
import tf
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion

def main():
    rospy.init_node('turtles_and_coke_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    ctrl_c = False

    def shutdownhook():
        global ctrl_c
        print("Shutdown time! Stopping the node")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        try:
            # Obtener transformaciones entre 'coke_can' y las tortugas
            trans_turtle1, rot_turtle1 = listener.lookupTransform('coke_can', 'turtle1', rospy.Time(0))
            trans_turtle2, rot_turtle2 = listener.lookupTransform('coke_can', 'turtle2', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # Obtener diferencias en posición y orientación para las tortugas
        x_diff_turtle1 = trans_turtle1[0]
        euler_turtle1 = euler_from_quaternion(rot_turtle1)
        yaw_diff_turtle1 = euler_turtle1[2]

        x_diff_turtle2 = trans_turtle2[0]
        euler_turtle2 = euler_from_quaternion(rot_turtle2)
        yaw_diff_turtle2 = euler_turtle2[2]

        # Mostrar diferencias en posición y orientación
        print(f"Turtle 1 - Diferencia en posición X: {x_diff_turtle1:.2f}, Diferencia en orientación yaw: {yaw_diff_turtle1:.2f}")
        print(f"Turtle 2 - Diferencia en posición X: {x_diff_turtle2:.2f}, Diferencia en orientación yaw: {yaw_diff_turtle2:.2f}")

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

