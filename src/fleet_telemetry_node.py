#!/usr/bin/env python
"""
fleet_telemetry_node.py

No central de telemetria e seguranca da frota.
- Assina as odometrias dos 3 robos (/robot_0/odom, /robot_1/odom,
  /robot_2/odom).
- Calcula, para cada robo: distancia total percorrida (acumulada por
  integracao do deslocamento entre callbacks) e velocidade escalar
  instantanea.
- Expoe o servico customizado /get_fleet_status, que retorna o resumo
  de distancia acumulada, velocidade e estado de cada robo.
- Monitora a distancia entre cada par de robos. Se algum par ficar
  abaixo da distancia critica de colisao, publica True em
  /emergency_stop para parar toda a frota imediatamente.
"""

import math
import itertools
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from multirobot_fleet.srv import GetFleetStatus, GetFleetStatusResponse


ROBOT_NAMES = ['robot_0', 'robot_1', 'robot_2']


class RobotTracker(object):
    """Mantem o estado acumulado de um unico robo."""

    def __init__(self, name):
        self.name = name
        self.last_x = None
        self.last_y = None
        self.last_time = None
        self.total_distance = 0.0
        self.current_speed = 0.0
        self.x = 0.0
        self.y = 0.0

    def update(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        now = rospy.Time.now()

        if self.last_x is not None:
            dx = x - self.last_x
            dy = y - self.last_y
            dist = math.sqrt(dx * dx + dy * dy)
            dt = (now - self.last_time).to_sec()

            self.total_distance += dist
            self.current_speed = dist / dt if dt > 0 else 0.0

        self.last_x = x
        self.last_y = y
        self.last_time = now
        self.x = x
        self.y = y


class FleetTelemetryNode(object):
    def __init__(self):
        rospy.init_node('fleet_telemetry_node')

        # Distancia critica entre dois robos que dispara a emergencia
        self.critical_distance = rospy.get_param('~critical_distance', 0.5)

        self.trackers = {name: RobotTracker(name) for name in ROBOT_NAMES}

        self.emergency_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=10, latch=True)
        # Garante que o topico comeca em estado "sem emergencia"
        self.emergency_pub.publish(Bool(data=False))
        self._emergency_active = False

        for name in ROBOT_NAMES:
            rospy.Subscriber('/%s/odom' % name, Odometry,
                              self.make_odom_callback(name))

        rospy.Service('/get_fleet_status', GetFleetStatus, self.handle_get_fleet_status)

        self.check_rate = rospy.Rate(10)

    def make_odom_callback(self, name):
        def _callback(msg):
            self.trackers[name].update(msg)
        return _callback

    def handle_get_fleet_status(self, req):
        response = GetFleetStatusResponse()
        for name in ROBOT_NAMES:
            tracker = self.trackers[name]
            response.robot_names.append(name)
            response.distances_traveled.append(tracker.total_distance)
            response.current_speeds.append(tracker.current_speed)
            status = 'EMERGENCY_STOP' if self._emergency_active else 'OK'
            response.status.append(status)
        return response

    def check_collision_risk(self):
        """Verifica todos os pares de robos e ativa/desativa a
        emergencia conforme a distancia critica."""
        any_pair_critical = False

        for name_a, name_b in itertools.combinations(ROBOT_NAMES, 2):
            tracker_a = self.trackers[name_a]
            tracker_b = self.trackers[name_b]

            if tracker_a.last_x is None or tracker_b.last_x is None:
                continue

            dx = tracker_a.x - tracker_b.x
            dy = tracker_a.y - tracker_b.y
            distance = math.sqrt(dx * dx + dy * dy)

            if distance < self.critical_distance:
                any_pair_critical = True
                rospy.logwarn_throttle(
                    1.0, 'ALERTA: %s e %s a %.2f m (risco de colisao)',
                    name_a, name_b, distance)

        if any_pair_critical and not self._emergency_active:
            self._emergency_active = True
            self.emergency_pub.publish(Bool(data=True))
            rospy.logerr('EMERGENCY_STOP ativado: frota interrompida!')
        elif not any_pair_critical and self._emergency_active:
            self._emergency_active = False
            self.emergency_pub.publish(Bool(data=False))
            rospy.loginfo('Risco de colisao removido: emergencia desativada')

    def run(self):
        rospy.loginfo('fleet_telemetry_node: monitorando frota e servico /get_fleet_status ativo')
        while not rospy.is_shutdown():
            self.check_collision_risk()
            self.check_rate.sleep()


if __name__ == '__main__':
    try:
        node = FleetTelemetryNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
