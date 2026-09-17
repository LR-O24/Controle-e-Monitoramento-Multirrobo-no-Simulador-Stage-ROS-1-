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


# Lista dos robos que serao monitorados pelo no.
# Esses nomes tambem sao usados para construir os topicos de odometria.
ROBOT_NAMES = ['robot_0', 'robot_1', 'robot_2']


class RobotTracker(object):
    """Mantem o estado acumulado de um unico robo."""

    def __init__(self, name):
        # Nome do robo associado a este tracker.
        self.name = name

        # Armazena a ultima posicao e o ultimo instante recebidos.
        # None indica que ainda nao recebemos nenhuma odometria.
        self.last_x = None
        self.last_y = None
        self.last_time = None

        # Distancia total percorrida e velocidade atual.
        self.total_distance = 0.0
        self.current_speed = 0.0

        # Posicao atual do robo.
        self.x = 0.0
        self.y = 0.0

    def update(self, msg):
        # Extrai a posicao X e Y da mensagem de odometria.
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Guarda o instante atual para calcular o intervalo de tempo
        # entre esta leitura e a leitura anterior.
        now = rospy.Time.now()

        # Na primeira mensagem nao existe uma posicao anterior para
        # comparar. A partir da segunda mensagem, podemos calcular
        # deslocamento e velocidade.
        if self.last_x is not None:
            # Calcula o deslocamento do robo desde o ultimo callback.
            dx = x - self.last_x
            dy = y - self.last_y

            # Calcula a distancia percorrida nesse intervalo usando a distancia euclidiana:
            # dist = sqrt(dx^2 + dy^2)
            dist = math.sqrt(dx * dx + dy * dy)

            # Calcula quanto tempo passou desde a ultima atualizacao.
            dt = (now - self.last_time).to_sec()

            # Acumula o deslocamento na distancia total percorrida.
            self.total_distance += dist

            # Calcula a velocidade escalar media durante este pequeno intervalo. O teste dt > 0 evita divisao por zero.
            self.current_speed = dist / dt if dt > 0 else 0.0

        # A posicao e o instante atuais passam a ser os valores anteriores para o proximo callback.
        self.last_x = x
        self.last_y = y
        self.last_time = now

        # Atualiza a posicao atual do robo.
        self.x = x
        self.y = y


class FleetTelemetryNode(object):
    def __init__(self):
        # Inicializa este programa como um no ROS.
        rospy.init_node('fleet_telemetry_node')

        # Distancia minima permitida entre dois robos antes de considerar que existe risco de colisao.
        # O valor padrao e 0.5 metro, mas pode ser alterado por parametro ROS.
        self.critical_distance = rospy.get_param('~critical_distance', 0.5)

        # Cria um RobotTracker independente para cada robo. O dicionario permite acessar cada tracker pelo nome:
        # self.trackers['robot_0'], por exemplo.
        self.trackers = {name: RobotTracker(name) for name in ROBOT_NAMES}

        # Publisher usado para informar aos outros nos se a frota deve realizar uma parada de emergencia.
        # latch=True faz com que o ultimo valor publicado seja mantido,
        # permitindo que novos subscribers recebam imediatamente o estado atual do topico.
        self.emergency_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=10, latch=True)

        # Inicialmente a frota esta em operacao normal.
        self.emergency_pub.publish(Bool(data=False))
        self._emergency_active = False

        # Cria um subscriber para a odometria de cada robo. O callback de cada subscriber sera associado ao tracker correspondente.
        for name in ROBOT_NAMES:
            rospy.Subscriber('/%s/odom' % name, Odometry,
                              self.make_odom_callback(name))

        # Cria o servico que permite que outros nos consultem o estado atual da frota.
        rospy.Service('/get_fleet_status', GetFleetStatus, self.handle_get_fleet_status)

        # Define a frequencia do loop principal como 10 Hz. Portanto, a verificacao de risco de colisao ocorre
        # aproximadamente 10 vezes por segundo.
        self.check_rate = rospy.Rate(10)

    def make_odom_callback(self, name):
        # Cria um callback especifico para cada robo. Isso permite que uma mensagem recebida em, por exemplo,
        # /robot_1/odom atualize somente o tracker do robot_1.
        def _callback(msg):
            self.trackers[name].update(msg)
        return _callback

    def handle_get_fleet_status(self, req):
        # Cria a estrutura que sera enviada como resposta ao servico.
        response = GetFleetStatusResponse()

        # Adiciona os dados de cada robo na resposta.
        for name in ROBOT_NAMES:
            tracker = self.trackers[name]

            # Nome do robo.
            response.robot_names.append(name)

            # Distancia total acumulada pelo tracker.
            response.distances_traveled.append(tracker.total_distance)

            # Velocidade calculada a partir dos dois ultimos callbacks.
            response.current_speeds.append(tracker.current_speed)

            # O estado e definido para todos os robos de acordo com o estado geral da frota.
            status = 'EMERGENCY_STOP' if self._emergency_active else 'OK'
            response.status.append(status)

        return response

    def check_collision_risk(self):
        """Verifica todos os pares de robos e ativa/desativa a
        emergencia conforme a distancia critica."""

        # Indica se pelo menos um par de robos esta abaixo da distancia critica.
        any_pair_critical = False

        # Gera todas as combinacoes possiveis de dois robos: (robot_0, robot_1), (robot_0, robot_2) e (robot_1, robot_2).
        # Assim, cada par e verificado uma unica vez.
        for name_a, name_b in itertools.combinations(ROBOT_NAMES, 2):
            tracker_a = self.trackers[name_a]
            tracker_b = self.trackers[name_b]

            # Se algum dos dois robos ainda nao enviou sua primeira odometria, nao ha informacao suficiente para calcular
            # a distancia entre eles.
            if tracker_a.last_x is None or tracker_b.last_x is None:
                continue

            # Calcula a diferenca entre as coordenadas dos dois robos.
            dx = tracker_a.x - tracker_b.x
            dy = tracker_a.y - tracker_b.y

            # Calcula a distancia euclidiana entre os dois robos.
            distance = math.sqrt(dx * dx + dy * dy)

            # Se a distancia for menor que o limite definido, considera-se que existe risco de colisao.
            if distance < self.critical_distance:
                any_pair_critical = True

                # Registra um alerta no terminal. logwarn_throttle limita a frequencia das mensagens
                # para evitar inundar o terminal com alertas repetidos.
                rospy.logwarn_throttle(
                    1.0, 'ALERTA: %s e %s a %.2f m (risco de colisao)',
                    name_a, name_b, distance)

        # Se algum par entrou em distancia critica e a emergencia ainda nao estava ativa, publica o comando de parada.
        if any_pair_critical and not self._emergency_active:
            self._emergency_active = True

            # True no /emergency_stop informa aos nos responsaveis pelo movimento que a frota deve ser interrompida.
            self.emergency_pub.publish(Bool(data=True))

            rospy.logerr('EMERGENCY_STOP ativado: frota interrompida!')

        # Se nenhum par esta mais em distancia critica e a emergencia estava ativa, o codigo libera novamente o estado de emergencia.
        elif not any_pair_critical and self._emergency_active:
            self._emergency_active = False

            # False indica que nao existe mais um par em distancia critica.
            self.emergency_pub.publish(Bool(data=False))

            rospy.loginfo('Risco de colisao removido: emergencia desativada')

    def run(self):
        # Informa que o no esta funcionando e que o servico esta ativo.
        rospy.loginfo('fleet_telemetry_node: monitorando frota e servico /get_fleet_status ativo')

        # Loop principal do nó. Continua executando enquanto o ROS não tiver solicitado o encerramento do programa.
        while not rospy.is_shutdown():

            # Verifica se existe algum par de robos em distância critica.
            self.check_collision_risk()

            # Aguarda o proximo ciclo, mantendo aproximadamente 10 Hz.
            self.check_rate.sleep()


if __name__ == '__main__':
    try:
        # Cria o no de telemetria da frota.
        node = FleetTelemetryNode()

        # Inicia o loop principal de monitoramento.
        node.run()

    # Permite que o programa seja encerrado normalmente quando o ROS for interrompido.
    except rospy.ROSInterruptException:
        pass
