# Controle e Monitoramento Multirrobô no Simulador Stage (ROS 1)

Projeto acadêmico em **ROS 1** para controle e monitoramento de uma frota de
3 robôs móveis (Pioneer 3DX) operando simultaneamente no simulador 2D
**Stage** (`stage_ros`), com execução distribuída em rede local
(multi-máquina).

🔗 Repositório: [github.com/LR-O24/Controle-e-Monitoramento-Multirrobo-no-Simulador-Stage-ROS-1-](https://github.com/LR-O24/Controle-e-Monitoramento-Multirrobo-no-Simulador-Stage-ROS-1-)

## Sumário

- [Visão geral](#visão-geral)
- [Prerequisites](#prerequisites)
- [Build & Setup](#build--setup)
- [Instruções Multi-PC](#instruções-multi-pc)
- [How to Run](#how-to-run)
- [Chamando o serviço /get_fleet_status](#chamando-o-serviço-get_fleet_status)
- [Estrutura do repositório](#estrutura-do-repositório)
- [Equipe](#equipe)

## Visão geral

| Robô | Comportamento | Tópicos principais |
|------|----------------|---------------------|
| `robot_0` | Navegação por odometria (trajetória retangular) | `/robot_0/odom`, `/robot_0/cmd_vel` |
| `robot_1` | Desvio autônomo de obstáculos | `/robot_1/base_scan`, `/robot_1/cmd_vel` |
| `robot_2` | Seguidor/perseguição do `robot_0` | `/robot_0/odom`, `/robot_2/odom`, `/robot_2/cmd_vel` |

Um nó central (`fleet_telemetry_node`) assina as odometrias dos 3 robôs,
acumula a distância percorrida por cada um, calcula a velocidade escalar
instantânea, expõe o serviço customizado `/get_fleet_status` e monitora a
distância entre pares de robôs, publicando `True` em `/emergency_stop`
sempre que houver risco de colisão.

## Prerequisites

- Ubuntu 20.04 (recomendado) com **ROS Noetic**, ou Ubuntu 18.04 com
  **ROS Melodic**.
- Pacote `stage_ros`:

  ```bash
  sudo apt update
  sudo apt install ros-$ROS_DISTRO-stage-ros
  ```

- Ferramentas de build padrão do ROS 1:

  ```bash
  sudo apt install python3-catkin-tools build-essential
  ```

- Dependências Python (já inclusas em qualquer instalação padrão do ROS):
  `rospy`, `tf`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`.
- Todas as máquinas envolvidas (mestre e clientes) devem estar na
  **mesma rede local** e conseguir se `ping` mutuamente.

## Build & Setup

Clone o repositório dentro do seu workspace catkin e compile:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/LR-O24/Controle-e-Monitoramento-Multirrobo-no-Simulador-Stage-ROS-1-.git multirobot_fleet

cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

> O repositório é clonado com o nome de pasta `multirobot_fleet` para que
> ele corresponda ao nome do pacote ROS declarado no `package.xml` e
> usado em todos os comandos `roslaunch`/`rosrun` deste README.

> ⚠️ **Permissão de execução dos scripts.** Se os arquivos deste
> repositório foram adicionados pela página de upload web do GitHub, o
> bit de execução (`+x`) é perdido no processo. Rode o comando abaixo
> logo após o clone e antes de compilar, para garantir que os nós Python
> possam ser executados pelo `roslaunch`/`rosrun`:
>
> ```bash
> chmod +x ~/catkin_ws/src/multirobot_fleet/src/*.py
> ```

> Recomenda-se adicionar `source ~/catkin_ws/devel/setup.bash` ao final do
> seu `~/.bashrc` para não precisar repetir esse comando a cada novo
> terminal.

Repita este processo (clone + `catkin_make` + `source`) em **todas as
máquinas** que forem executar algum nó do projeto, inclusive as clientes.

## Instruções Multi-PC

O projeto foi desenhado para rodar de forma distribuída: uma máquina
**mestre** executa o `roscore` e o simulador Stage, enquanto uma ou mais
máquinas **clientes** executam os nós de comportamento dos robôs.

### 1. Descubra o IP de cada máquina

```bash
hostname -I
```

Anote o IP da máquina mestre (ex.: `192.168.0.10`) e o IP de cada cliente
(ex.: `192.168.0.11`).

### 2. Configure a máquina MESTRE

No terminal (ou no `~/.bashrc`) da máquina que vai rodar o `roscore` e o
Stage:

```bash
export ROS_MASTER_URI=http://192.168.0.10:11311
export ROS_IP=192.168.0.10
```

### 3. Configure cada máquina CLIENTE

No terminal (ou no `~/.bashrc`) de cada máquina que vai rodar os nós de
comportamento:

```bash
export ROS_MASTER_URI=http://192.168.0.10:11311
export ROS_IP=192.168.0.11
```

> Substitua os IPs pelos valores reais da sua rede. `ROS_MASTER_URI` deve
> apontar **sempre** para o IP do mestre, em todas as máquinas.

### 4. Teste a conectividade

Antes de rodar qualquer nó, valide a comunicação:

```bash
# na maquina cliente, com o roscore ja rodando na mestre
rostopic list
```

Se a lista de tópicos aparecer normalmente, a rede está configurada
corretamente.

## How to Run

### Opção A — Tudo em uma única máquina (teste local)

```bash
roslaunch multirobot_fleet fleet_simulation.launch
```

Esse launch já sobe o `roscore` (implicitamente, se ainda não estiver
rodando), o simulador Stage com o mundo `fleet_arena.world` e os 4 nós do
projeto (robôs 0, 1, 2 e telemetria).

### Opção B — Execução distribuída (multi-máquina)

**Na máquina MESTRE** (com as variáveis de ambiente da seção anterior já
exportadas):

```bash
roscore
```

Em outro terminal na mesma máquina mestre, suba apenas o simulador Stage:

```bash
rosrun stage_ros stageros $(rospack find multirobot_fleet)/worlds/fleet_arena.world
```

**Na(s) máquina(s) CLIENTE(s)** (com `ROS_MASTER_URI`/`ROS_IP` já
exportados):

```bash
roslaunch multirobot_fleet fleet_clients.launch
```

Isso sobe os nós `robot0_odometry_nav`, `robot1_obstacle_avoidance` e
`robot2_follower`, que passam a se comunicar com o Stage rodando na
máquina mestre.

Em uma terceira máquina (ou na própria mestre), rode o nó de telemetria:

```bash
rosrun multirobot_fleet fleet_telemetry_node.py
```

### Visualizar no RViz (opcional)

```bash
rosrun rviz rviz
```

Adicione os displays `LaserScan` (`/robot_1/base_scan`) e `Odometry`
para cada robô, ajustando o `Fixed Frame` para `map` ou `odom`.

## Chamando o serviço /get_fleet_status

Com o `fleet_telemetry_node` em execução, chame o serviço customizado em
qualquer terminal que tenha acesso ao `roscore`:

```bash
rosservice call /get_fleet_status "{}"
```

Saída esperada (exemplo):

```
robot_names: ['robot_0', 'robot_1', 'robot_2']
distances_traveled: [12.4, 18.9, 13.1]
current_speeds: [0.28, 0.31, 0.25]
status: ['OK', 'OK', 'OK']
```

Para acompanhar o mecanismo de emergência em tempo real:

```bash
rostopic echo /emergency_stop
```

## Estrutura do repositório

A raiz deste repositório **é** o pacote ROS (não há uma pasta `catkin_ws/src/`
versionada no Git — essa estrutura de workspace é montada localmente por
quem clona o projeto, conforme mostrado em [Build & Setup](#build--setup)).

```
Controle-e-Monitoramento-Multirrobo-no-Simulador-Stage-ROS-1-/   (raiz do repositório)
├── bitmaps/
│   └── arena.png                    # bitmap do mapa usado pelo Stage
├── launch/
│   ├── fleet_simulation.launch      # sobe Stage + todos os nós (uso local)
│   └── fleet_clients.launch         # sobe apenas os nós (uso em PC cliente)
├── src/
│   ├── robot0_odometry_nav.py       # navegação por odometria (robô 0)
│   ├── robot1_obstacle_avoidance.py # desvio de obstáculos (robô 1)
│   ├── robot2_follower.py           # perseguição do robô 0 (robô 2)
│   └── fleet_telemetry_node.py      # telemetria, serviço e emergência
├── srv/
│   └── GetFleetStatus.srv           # definição do serviço customizado
├── worlds/
│   └── fleet_arena.world            # mundo do Stage (mapa + 3 robôs)
├── CMakeLists.txt
├── package.xml
└── README.md
```

Ao seguir os comandos de `git clone` da seção **Build & Setup**, esse
conteúdo é posicionado dentro de `~/catkin_ws/src/multirobot_fleet/` na
máquina de cada desenvolvedor, formando a estrutura padrão do catkin:

```
~/catkin_ws/
└── src/
    └── multirobot_fleet/   ← conteúdo deste repositório
        ├── bitmaps/
        ├── launch/
        ├── src/
        ├── srv/
        ├── worlds/
        ├── CMakeLists.txt
        └── package.xml
```

## Equipe

| Nome | Contribuição |
|------|--------------|
| _Integrante 1_ | Robô 0 — navegação por odometria |
| _Integrante 2_ | Robô 1 — desvio de obstáculos |
| _Integrante 3_ | Robô 2 — perseguição |
| _Integrante 4_ | Nó de telemetria, serviço e testes multi-máquina |

## Vídeo demonstrativo

📹 Link do vídeo (3–5 min): `<inserir link do Loom/YouTube/Drive aqui>`

O vídeo mostra: o Stage com os 3 robôs executando seus comportamentos, a
chamada do serviço `/get_fleet_status` no terminal, e a comunicação
multi-máquina em ação (comandos rodando em dois computadores diferentes
da mesma rede).
