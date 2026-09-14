cmake_minimum_required(VERSION 3.0.2)
project(multirobot_fleet)

## Encontra o catkin e os pacotes ROS dos quais este pacote depende
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  message_generation
)

## Declara os arquivos de servico customizados (.srv) a serem gerados
add_service_files(
  FILES
  GetFleetStatus.srv
)

## Gera as mensagens/servicos, com base nas dependencias de mensagens usadas
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declara o pacote catkin: exporta as dependencias de runtime para quem
## depender deste pacote (necessario para o message_runtime funcionar)
catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Torna os scripts Python executaveis pelo rosrun/roslaunch e garante
## que o servico gerado (multirobot_fleet.srv) esteja disponivel antes
## de qualquer script que o importe
catkin_install_python(PROGRAMS
  src/robot0_odometry_nav.py
  src/robot1_obstacle_avoidance.py
  src/robot2_follower.py
  src/fleet_telemetry_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Instala tambem launch files, worlds e bitmaps junto com o pacote
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)

install(DIRECTORY bitmaps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/bitmaps
)
