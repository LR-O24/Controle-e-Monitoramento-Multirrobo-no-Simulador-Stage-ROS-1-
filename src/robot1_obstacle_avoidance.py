#!/usr/bin/env python
"""
robot1_obstacle_avoidance.py

Robo 1 - Desvio autonomo de obstaculos.
Le continuamente /robot_1/base_scan (sensor_msgs/LaserScan). Analisa um
setor frontal do feixe de leituras para detectar objetos proximos.
Se a distancia minima nesse setor ficar abaixo de um limiar de
seguranca, o robo para de andar reto e gira em direcao ao lado com
maior espaco livre ate desobstruir o caminho. Caso contrario segue
reto normalmente. Para imediatamente se /emergency_stop for True.
"""

import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class Robot1ObstacleAvoidance(object):
    def __init__(self):
        rospy.init_node('robot1_obstacle_avoidance')

        self.safe_distance = rospy.get_param('~safe_distance', 0.8)  # metros
        self.linear_speed = rospy.get_param('~linear_speed', 0.3)
        self.angular_speed = rospy.get_param('~angular_speed', 0.6)
        self.front_sector_deg = rospy.get_param('~front_sector_deg', 40)

        self.emergency_stop = False
        self.latest_scan = None

        self.cmd_pub = rospy.Publisher('/robot_1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/robot_1/base_scan_1', LaserScan, self.scan_callback)
        rospy.Subscriber('/emergency_stop', Bool, self.emergency_callback)

        self.rate = rospy.Rate(10)

    def emergency_callback(self, msg):
        self.emergency_stop = msg.data

    def scan_callback(self, msg):
        self.latest_scan = msg

    def get_sector_indices(self, scan):
        """Calcula os indices do array 'ranges' correspondentes ao
        setor frontal definido por self.front_sector_deg."""
        half_sector_rad = math.radians(self.front_sector_deg / 2.0)
        center_index = len(scan.ranges) // 2
        index_span = int(half_sector_rad / scan.angle_increment) if scan.angle_increment > 0 else 20
        start = max(0, center_index - index_span)
        end = min(len(scan.ranges), center_index + index_span)
        return start, end

    def run(self):
        rospy.loginfo('robot1_obstacle_avoidance: aguardando primeiro scan...')
        while not rospy.is_shutdown() and self.latest_scan is None:
            self.rate.sleep()

        rospy.loginfo('robot1_obstacle_avoidance: iniciando navegacao com desvio')

        while not rospy.is_shutdown():
            cmd = Twist()

            if self.emergency_stop:
                self.cmd_pub.publish(Twist())
                self.rate.sleep()
                continue

            scan = self.latest_scan
            start, end = self.get_sector_indices(scan)
            front_ranges = [r for r in scan.ranges[start:end]
                             if not math.isnan(r) and not math.isinf(r) and r > 0.0]

            min_front = min(front_ranges) if front_ranges else float('inf')

            if min_front < self.safe_distance:
                # Decide o lado do giro comparando espaco livre esquerda/direita
                left_ranges = [r for r in scan.ranges[end:] if r > 0.0 and not math.isinf(r)]
                right_ranges = [r for r in scan.ranges[:start] if r > 0.0 and not math.isinf(r)]
                left_free = sum(left_ranges) / len(left_ranges) if left_ranges else 0.0
                right_free = sum(right_ranges) / len(right_ranges) if right_ranges else 0.0

                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_speed if left_free >= right_free else -self.angular_speed
            else:
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = Robot1ObstacleAvoidance()
        node.run()
    except rospy.ROSInterruptException:
        pass
