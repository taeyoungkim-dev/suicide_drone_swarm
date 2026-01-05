import rclpy
import math
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, OffboardControlMode


#--------------------------------수정---------------------------------#
NUM_OF_DRONE = 10
#.world 좌표계 기준 목표 위치
#풍선은 1.5정도 좌표 차이가 나기 때문에 .world 파일 z 좌표를 1.5정도 빼주길 권장
INPUT_TARGET_LOCATION = [0.0, -10.0, 5.0]
#드론 소환 (3,0,-0.2) , (6,0,-0.2) , (9,0,-0.2) , (12,0,-0.2) ...
DRONE_SPAWN_X_GAP = 3.0
DRONE_SPAWN_Y = 0
DRONE_SPAWN_Z = -0.2

#대형 패턴 함수
#[10,0,-5] 기준
#대형 패턴 변경 시 이 부분을 수정
#줄대형
FORMATION_PATTERN = lambda i, n: [10.0, (i - (n-1)/2) * 2.0, -5.0]
#V자 대형
#FORMATION_PATTERN = lambda i, n: [10.0 - abs(i - (n-1)/2) * 2.0, (i - (n-1)/2) * 2.0, -5.0]
#역 V자 대형
#FORMATION_PATTERN = lambda i, n: [10.0 + abs(i - (n-1)/2) * 2.0, (i - (n-1)/2) * 2.0, -5.0]
#원형 대형.
#경로 겹침 문제 발생. 드론 8대 이상일 때 겹침 현상 심함
# CIRCLE_RADIUS = 5.0
# _sorted_circle = sorted([
#     [10.0 + CIRCLE_RADIUS * math.cos(2 * math.pi * k / NUM_OF_DRONE), 
#      CIRCLE_RADIUS * math.sin(2 * math.pi * k / NUM_OF_DRONE), 
#      -5.0] 
#     for k in range(NUM_OF_DRONE)
# ], key=lambda p: p[0])
# FORMATION_PATTERN = lambda i, n: _sorted_circle[i]
#-------------------------------------------------------------------#


TARGET_DRONE_INDEX = [i for i in range(2,NUM_OF_DRONE+2)]
TARGET_ABSOLUTE = [INPUT_TARGET_LOCATION[1],INPUT_TARGET_LOCATION[0],-INPUT_TARGET_LOCATION[2]]

class DroneController(Node):

    def __init__(self,drone_id) :

        super().__init__(f'drone_{drone_id}_node')
        self.drone_id = drone_id
        self.namespace = f"/drone{drone_id}"

        try:
            self.sys_id = drone_id
        except ValueError:
            self.sys_id = 1

        #드론 스폰위치
        self.drone_spawn_absolute = [
            int(drone_id-1)*DRONE_SPAWN_X_GAP,
            DRONE_SPAWN_Y,
            DRONE_SPAWN_Z
        ]
        #목표 위치
        self.target_absolute = TARGET_ABSOLUTE

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        #Publisher
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            f'{self.namespace}/fmu/in/offboard_control_mode',
            qos
        )

        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            f'{self.namespace}/fmu/in/trajectory_setpoint',
            qos
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            f'{self.namespace}/fmu/in/vehicle_command',
            qos
        )

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0
        #CHANGE
        self.warmup_steps = 50

    def send_offboard(self):
        off_msg = OffboardControlMode()
        off_msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        #위치 기반 이동 선언
        #CHANGE
        off_msg.position = True
        #off_msg.velocity 속도 기반
        #off_msg.attitude 기체 기울기(자세)
        #Publish
        self.offboard_pub.publish(off_msg)

    #TODO 궤도 항법 적용 필요 
    def send_trajectory(self):
        traj_msg = TrajectorySetpoint()
        #메세지 생성 시각 기록
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        
        #-----------------------궤도 항법 계산-----------------------#
        #대형 갖추기
        formation_position = FORMATION_PATTERN(self.drone_id - 2, NUM_OF_DRONE)
        formation_position_local = [
            float(formation_position[0] - self.drone_spawn_absolute[0]),
            float(formation_position[1] - self.drone_spawn_absolute[1]),
            float(formation_position[2] - self.drone_spawn_absolute[2]),
        ]
        
        traj_msg.position[0] = formation_position_local[0]
        traj_msg.position[1] = formation_position_local[1]
        traj_msg.position[2] = formation_position_local[2]
        #TODO 어디를 볼 것인가 못정함
        traj_msg.yaw = 0.0
        #TODO 대형 포지션에 도착했는지 확인
        #TODO 목표 공격
        #publish
        self.trajectory_pub.publish(traj_msg)

    def send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = self.sys_id
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def timer_callback(self):

        self.send_offboard()
        if self.counter == 0:
            print(f"{self.drone_id} offboard published")
        #Warmup
        #warmup_step을 넘어야 탈출
        if self.counter < self.warmup_steps:
            if self.counter == 10:
                print(f"{self.drone_id} Warming up")
            self.counter += 1
            return
        
        self.send_trajectory()
        if self.counter == self.warmup_steps:
            print(f"{self.drone_id} trajectory published")
        
        if self.warmup_steps+10<=self.counter<self.warmup_steps+20:
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            if self.warmup_steps + 10 == self.counter:
                print(f"{self.drone_id} offboard command published")

        if self.warmup_steps+20<=self.counter<self.warmup_steps+30:
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            if self.warmup_steps + 20 == self.counter:
                print(f"{self.drone_id} arm command published")
        
        self.counter += 1

def main(args=None):
    rclpy.init()
    executor = MultiThreadedExecutor()
    nodes = []

    for drone_id in TARGET_DRONE_INDEX:
        node = DroneController(drone_id)
        nodes.append(node)
        executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        print("\n시스템 종료")
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
        
        






        
