#TODO
#formation 잡을 때와 attack 할 때 yaw 설정
#공격할 때 드론 끼리 부딧힘. formation 간격 늘려보기
#드론이 일제히 출발하지 않음. 모든 드론이 준비되면 일제히 공격하는 로직 필요
import rclpy
import math
import threading
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, OffboardControlMode, VehicleOdometry


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
FORMATION_PATTERN = lambda i, n: [10.0, (i - (n-1)/2) * 4.0, -5.0]
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


class MissionSync:
    """Shared barrier so every drone jumps to phase 2 together."""

    def __init__(self, total_drones: int) -> None:
        self.total = total_drones
        self.ready_set = set()
        self.attack_released = False
        self._lock = threading.Lock()

    def mark_ready(self, drone_id: int) -> None:
        with self._lock:
            self.ready_set.add(drone_id)
            if len(self.ready_set) >= self.total:
                self.attack_released = True

    def attack_allowed(self) -> bool:
        with self._lock:
            return self.attack_released

class DroneController(Node):

    def __init__(self,drone_id, mission_sync: MissionSync) :

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
        
        #현재 드론 위치
        self.current_absolute = [0.0,0.0,0.0]
        
        #0:대형 이동, 1: 도착 후 대기, 2: 목표 공격
        self.mission_phase = 0
        self.mission_sync = mission_sync
        self.ready_reported = False
        
        #도착 후 대기 시간 카운터
        self.wait_counter = 0
        self.how_much_wait = 0
        
        # 마지막 목표 벡터
        self.last_aiming = [0.0,0.0,0.0]

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

        #Subscriber
        self.odometry_sub = self.create_subscription(
            VehicleOdometry,
            f'{self.namespace}/fmu/out/vehicle_odometry',
            self.odometry_cb,
            qos
        )
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0
        #CHANGE
        self.warmup_steps = 30

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
 
    def send_trajectory(self):
        traj_msg = TrajectorySetpoint()
        #메세지 생성 시각 기록
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        
        #-----------------------궤도 항법 계산-----------------------#
        #대형 갖추기
        if self.mission_phase == 0:
            #대형 위치 계산
            formation_position = FORMATION_PATTERN(self.drone_id - 2, NUM_OF_DRONE)
            #드론 스폰 중심 좌표계로 변환
            formation_position_local = [
                float(formation_position[0] - self.drone_spawn_absolute[0]),
                float(formation_position[1] - self.drone_spawn_absolute[1]),
                float(formation_position[2] - self.drone_spawn_absolute[2]),
            ]
            #msg position 작성
            traj_msg.position[0] = formation_position_local[0]
            traj_msg.position[1] = formation_position_local[1]
            traj_msg.position[2] = formation_position_local[2]
            #TODO 어디를 볼 것인가 못정함
            traj_msg.yaw = 0.0
            #publish
            self.trajectory_pub.publish(traj_msg)
            #대형 도착 확인
            distance = math.sqrt(
                (self.current_absolute[0] - (self.drone_spawn_absolute[0] + formation_position_local[0]))**2 +
                (self.current_absolute[1] - (self.drone_spawn_absolute[1] + formation_position_local[1]))**2 +
                (self.current_absolute[2] - (self.drone_spawn_absolute[2] + formation_position_local[2]))**2
            )
            #CHANGE 도착 허용 오차 조정
            if distance < 0.5:
                self.mission_phase = 1
                print(f"{self.drone_id} 대형 도착 완료")
        
        elif self.mission_phase == 1:
            #대형 도착 후 대기
            #대기 중에도 위치 유지 명령 지속 전송
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

            self.trajectory_pub.publish(traj_msg)
            
            #대형 도착 여부를 한 번만 보고
            if not self.ready_reported:
                self.mission_sync.mark_ready(self.drone_id)
                self.ready_reported = True

            #모든 드론이 phase1이면 일제히 phase2로 전환
            if self.mission_sync.attack_allowed():
                self.mission_phase = 2
                print(f"{self.drone_id} 목표 공격 시작")
                
        #목표 공격 단계        
        elif self.mission_phase == 2:
            vec_x = self.target_absolute[0] - self.current_absolute[0]
            vec_y = self.target_absolute[1] - self.current_absolute[1]
            vec_z = self.target_absolute[2] - self.current_absolute[2]
            distance = math.sqrt(vec_x**2 + vec_y**2 + vec_z**2)
            
            if distance > 0.1:
                unit_x = vec_x / distance
                unit_y = vec_y / distance
                unit_z = vec_z / distance
                
                #CHANGE 목표 지점보다 약간 더 나아간 지점 설정
                overshoot_distance = 5.0
                traj_msg.position[0] = float(self.target_absolute[0] + unit_x * overshoot_distance - self.drone_spawn_absolute[0])
                traj_msg.position[1] = float(self.target_absolute[1] + unit_y * overshoot_distance - self.drone_spawn_absolute[1])
                traj_msg.position[2] = float(self.target_absolute[2] + unit_z * overshoot_distance - self.drone_spawn_absolute[2])
                self.last_aiming = [traj_msg.position[0], traj_msg.position[1], traj_msg.position[2]]
                #TODO 어디를 볼 것인가 못정함
                traj_msg.yaw = 0.0
            
            else :
                traj_msg.position[0] = self.last_aiming[0]
                traj_msg.position[1] = self.last_aiming[1]
                traj_msg.position[2] = self.last_aiming[2]
                traj_msg.yaw = 0.0
            
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
        
    def odometry_cb(self, msg):
        self.current_absolute[0] = msg.position[0] + self.drone_spawn_absolute[0]
        self.current_absolute[1] = msg.position[1] + self.drone_spawn_absolute[1]
        self.current_absolute[2] = msg.position[2] + self.drone_spawn_absolute[2]

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

    mission_sync = MissionSync(NUM_OF_DRONE)

    for drone_id in TARGET_DRONE_INDEX:
        node = DroneController(drone_id, mission_sync)
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
        
        






        
