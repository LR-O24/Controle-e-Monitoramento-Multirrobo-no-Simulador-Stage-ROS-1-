#!/usr/bin/env python
"""
robot0_odometry_nav.py

Robo 0 - Navegacao por odometria.
Executa uma trajetoria retangular fechada usando apenas os dados de
/robot_0/odom (posicao x,y e orientacao yaw obtida por conversao
quaternion -> Euler). Usa uma maquina de estados simples:
    1) andar reto ate percorrer o comprimento do lado
    2) girar no proprio eixo ate atingir o angulo alvo (+90 graus)
    3) repetir para os 4 lados do retangulo, depois recomeca o ciclo
Para imediatamente se receber True em /emergency_stop.
"""

import math
import rospy
import tf.transformations as tft
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


def normalize_angle(angle):
    """Mantem o angulo no intervalo [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class Robot0OdometryNav(object):
    def __init__(self):
        rospy.init_node('robot0_odometry_nav')

        # Parametros da trajetoria retangular
        self.side_length = rospy.get_param('~side_length', 3.0)   # metros
        self.linear_speed = rospy.get_param('~linear_speed', 0.3)  # m/s
        self.angular_speed = rospy.get_param('~angular_speed', 0.5)  # rad/s

        # Estado atual (atualizado pelo callback de odometria)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_odom = False

        # Estado inicial de referencia para cada segmento
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0

        # Maquina de estados: 'FORWARD' ou 'TURN'
        self.state = 'FORWARD'
        self.emergency_stop = False

        self.cmd_pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/robot_0/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/emergency_stop', Bool, self.emergency_callback)

        self.rate = rospy.Rate(10)  # 10 Hz

    def emergency_callback(self, msg):
        self.emergency_stop = msg.data

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (_, _, yaw) = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw = yaw

        if not self.have_odom:
            self.start_x = self.x
            self.start_y = self.y
            self.start_yaw = self.yaw
            self.have_odom = True

    def distance_traveled_in_segment(self):
        dx = self.x - self.start_x
        dy = self.y - self.start_y
        return math.sqrt(dx * dx + dy * dy)

    def run(self):
        rospy.loginfo('robot0_odometry_nav: aguardando primeira odometria...')
        while not rospy.is_shutdown() and not self.have_odom:
            self.rate.sleep()

        rospy.loginfo('robot0_odometry_nav: iniciando trajetoria retangular')

        while not rospy.is_shutdown():
            cmd = Twist()

            if self.emergency_stop:
                # Robo parado, aguardando o fim da emergencia
                self.cmd_pub.publish(Twist())
                self.rate.sleep()
                continue

            if self.state == 'FORWARD':
                traveled = self.distance_traveled_in_segment()
                if traveled < self.side_length:
                    cmd.linear.x = self.linear_speed
                else:
                    # Chegou ao final do lado: prepara o giro
                    self.state = 'TURN'
                    self.start_yaw = self.yaw
                    rospy.loginfo('robot0: lado concluido (%.2f m), iniciando giro', traveled)

            elif self.state == 'TURN':
                target_delta = math.pi / 2.0  # 90 graus
                current_delta = normalize_angle(self.yaw - self.start_yaw)
                if abs(current_delta) < target_delta - 0.03:
                    cmd.angular.z = self.angular_speed
                else:
                    # Giro concluido: comeca novo lado
                    self.state = 'FORWARD'
                    self.start_x = self.x
                    self.start_y = self.y
                    rospy.loginfo('robot0: giro concluido, iniciando novo lado')

            self.cmd_pub.publish(cmd)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = Robot0OdometryNav()
        node.run()
    except rospy.ROSInterruptException:
        pass
