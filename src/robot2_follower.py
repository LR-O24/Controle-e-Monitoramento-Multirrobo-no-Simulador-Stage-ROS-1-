#!/usr/bin/env python
"""
robot2_follower.py

Robo 2 - Seguidor/Perseguicao.
Assina simultaneamente /robot_0/odom e /robot_2/odom, calcula a
distancia euclidiana e o angulo relativo entre os dois robos, e usa um
controlador proporcional simples para ajustar velocidade linear e
angular publicadas em /robot_2/cmd_vel, de forma a perseguir o Robo 0
mantendo uma distancia de seguranca constante (nem colidir, nem
perder o alvo de vista). Para imediatamente se /emergency_stop=True.
"""

import math
import rospy
import tf.transformations as tft
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class Robot2Follower(object):
    def __init__(self):
        rospy.init_node('robot2_follower')

        # Distancia alvo que o robo 2 deve manter do robo 0
        self.desired_distance = rospy.get_param('~desired_distance', 1.5)  # metros
        self.min_distance = rospy.get_param('~min_distance', 0.8)          # distancia critica

        # Ganhos do controlador proporcional
        self.kp_linear = rospy.get_param('~kp_linear', 0.6)
        self.kp_angular = rospy.get_param('~kp_angular', 1.5)
        self.max_linear = rospy.get_param('~max_linear', 0.4)
        self.max_angular = rospy.get_param('~max_angular', 1.0)

        self.target_pose = None   # (x, y) do robo 0
        self.self_pose = None     # (x, y, yaw) do robo 2
        self.emergency_stop = False

        self.cmd_pub = rospy.Publisher('/robot_2/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/robot_0/odom', Odometry, self.target_odom_callback)
        rospy.Subscriber('/robot_2/odom', Odometry, self.self_odom_callback)
        rospy.Subscriber('/emergency_stop', Bool, self.emergency_callback)

        self.rate = rospy.Rate(10)

    def emergency_callback(self, msg):
        self.emergency_stop = msg.data

    def target_odom_callback(self, msg):
        self.target_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def self_odom_callback(self, msg):
        q = msg.pose.pose.orientation
        (_, _, yaw) = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.self_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def run(self):
        rospy.loginfo('robot2_follower: aguardando odometrias de robot_0 e robot_2...')
        while not rospy.is_shutdown() and (self.target_pose is None or self.self_pose is None):
            self.rate.sleep()

        rospy.loginfo('robot2_follower: iniciando perseguicao')

        while not rospy.is_shutdown():
            cmd = Twist()

            if self.emergency_stop:
                self.cmd_pub.publish(Twist())
                self.rate.sleep()
                continue

            tx, ty = self.target_pose
            sx, sy, syaw = self.self_pose

            dx = tx - sx
            dy = ty - sy
            distance = math.sqrt(dx * dx + dy * dy)
            angle_to_target = math.atan2(dy, dx)
            angle_error = normalize_angle(angle_to_target - syaw)

            distance_error = distance - self.desired_distance

            # Nao anda para frente se estiver abaixo da distancia critica
            if distance <= self.min_distance:
                linear_vel = 0.0
            else:
                linear_vel = max(0.0, min(self.kp_linear * distance_error, self.max_linear))

            angular_vel = max(-self.max_angular, min(self.kp_angular * angle_error, self.max_angular))

            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel

            self.cmd_pub.publish(cmd)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = Robot2Follower()
        node.run()
    except rospy.ROSInterruptException:
        pass
