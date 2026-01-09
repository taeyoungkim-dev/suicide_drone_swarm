#TODO
#드론 끼리 충돌 - 부딧혀도 대부분 알아서 자리 찾아감.
#목표 물체가 엄청 작으면 z축 좌표 오차로 빗나감.
import rclpy
import math
import threading
import numpy as np # 벡터 계산을 위해 numpy 권장 (없으면 math로 대체 가능하나 편리함)
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, OffboardControlMode, VehicleOdometry

#--------------------------------설정---------------------------------#
NUM_OF_DRONE = 10
INPUT_TARGET_LOCATION = [0.0, -50.0, 20.0]
DRONE_SPAWN_X_GAP = 3.0
DRONE_SPAWN_Y = 0
DRONE_SPAWN_Z = -0.2
ATTACK_STAGING_RADIUS = 10.0 # 목표물 반경 n 미터 에서 대기
Z_SCALE_RATIO = 0.5 # 피보나치 구 압축 비율. 작을수록 납작해짐.
OVERSHOOT_DISTANCE = 5.0 # 오버슈트 거리
FORMATION_DISTANCE_THRESHOLD = 0.5 # 대형 도착 허용 오차
STAGING_DISTANCE_THRESHOLD = 0.5 # 포위 도착 허용 오차


# 줄대형 유지
FORMATION_PATTERN = lambda i, n: [10.0, (i - (n-1)/2) * 3.0, -5.0]

TARGET_DRONE_INDEX = [i for i in range(2, NUM_OF_DRONE + 2)]
TARGET_ABSOLUTE = [INPUT_TARGET_LOCATION[1], INPUT_TARGET_LOCATION[0], -INPUT_TARGET_LOCATION[2]]
#-------------------------------------------------------------------#

class MissionSync:
    """모든 드론의 Phase 동기화를 위한 클래스"""
    def __init__(self, total_drones: int) -> None:
        self.total = total_drones
        self.ready_set_p1 = set() # Phase 1 완료 (대형)
        self.ready_set_p2 = set() # Phase 2 완료 (포위)
        self.p2_start = False
        self.p3_start = False
        self._lock = threading.Lock()

    def mark_ready_phase1(self, drone_id: int) -> None:
        with self._lock:
            self.ready_set_p1.add(drone_id)
            if len(self.ready_set_p1) >= self.total:
                self.p2_start = True

    def mark_ready_phase2(self, drone_id: int) -> None:
        with self._lock:
            self.ready_set_p2.add(drone_id)
            if len(self.ready_set_p2) >= self.total:
                self.p3_start = True

    def check_phase2_start(self) -> bool:
        with self._lock: return self.p2_start

    def check_phase3_start(self) -> bool:
        with self._lock: return self.p3_start

def get_fibonacci_sphere_points(samples, radius, center):
    points = []
    phi = math.pi * (3. - math.sqrt(5.))  # 황금각

    # [핵심 1] 높이 압축 비율 (0.4 = 구를 위아래로 40% 납작하게 만듦)
    z_scale_ratio = Z_SCALE_RATIO

    for i in range(samples):
        # [핵심 2] 변수 역할 변경
        # 기존 코드의 y는 -1~1로 선형적으로 변하는 값이었음.
        # 이를 높이(z_pos)로 사용하여 드론을 위에서 아래로 균등 배치함.
        
        # 1(맨 위) -> -1(맨 아래) 로 변함
        z_pos = 1 - (i / float(samples - 1)) * 2 
        
        # 해당 높이에서의 수평 원 반지름
        radius_at_h = math.sqrt(1 - z_pos * z_pos) * radius
        
        theta = phi * i  # 각도 증가

        # 수평 좌표 (X, Y) 계산
        x = math.cos(theta) * radius_at_h
        y = math.sin(theta) * radius_at_h
        
        # [핵심 3] 높이(Z) 좌표 계산 및 압축 적용
        # z_pos(-1~1) * 반지름 * 압축비율
        z = z_pos * radius * z_scale_ratio

        # 최종 좌표 변환 (Center 이동)
        px = center[0] + x
        py = center[1] + y
        pz = center[2] + z  # 여기가 높이!
        
        points.append([px, py, pz])
        
    return points

class DroneController(Node):
    def __init__(self, drone_id, mission_sync: MissionSync, assigned_staging_point):
        super().__init__(f'drone_{drone_id}_node')
        self.drone_id = drone_id
        self.namespace = f"/drone{drone_id}"
        
        try:
            self.sys_id = drone_id
        except ValueError:
            self.sys_id = 1

        self.drone_spawn_absolute = [
            int(drone_id-1)*DRONE_SPAWN_X_GAP,
            DRONE_SPAWN_Y,
            DRONE_SPAWN_Z
        ]
        
        self.target_absolute = TARGET_ABSOLUTE
        self.current_absolute = [0.0, 0.0, 0.0]
        self.mission_sync = mission_sync
        
        # Phase 2에서 갈 위치 (할당받은 포위 지점)
        self.staging_point = assigned_staging_point
        
        # 0:이륙/대형, 1:대기, 2:포위이동, 3:타격
        self.mission_phase = 0
        self.ready_p1_reported = False
        self.ready_p2_reported = False
        
        self.counter = 0
        self.warmup_steps = 30
        self.attack_yaw = None

        # QoS 및 Pub/Sub 설정 (기존과 동일)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.offboard_pub = self.create_publisher(OffboardControlMode, f'{self.namespace}/fmu/in/offboard_control_mode', qos)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, f'{self.namespace}/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(VehicleCommand, f'{self.namespace}/fmu/in/vehicle_command', qos)
        self.odometry_sub = self.create_subscription(VehicleOdometry, f'{self.namespace}/fmu/out/vehicle_odometry', self.odometry_cb, qos)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def calculate_yaw(self, target_pos):
        """현재 위치에서 목표 위치를 바라보는 Yaw 각도 계산 (NED 좌표계)"""
        dx = target_pos[0] - self.current_absolute[0]
        dy = target_pos[1] - self.current_absolute[1]
        # atan2(y, x) -> PX4 NED에서는 atan2(dy, dx)가 일반적인 방위각
        return math.atan2(dy, dx)

    def send_offboard(self):
        off_msg = OffboardControlMode()
        off_msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        # 모든 Phase에서 위치 기반 이동 사용
        off_msg.position = True
        off_msg.velocity = False
        off_msg.acceleration = False
        off_msg.attitude = False
        off_msg.body_rate = False
        self.offboard_pub.publish(off_msg)
 
    def send_trajectory(self):
        traj_msg = TrajectorySetpoint()
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        
        # [Phase 0] 대형 갖추기
        if self.mission_phase == 0:
            HOVER_ALTITUDE = -1.0
            HOVER_WAIT_TIME = 30
            SEQUENCE_INTERVAL = 5
            
            formation_pos = FORMATION_PATTERN(self.drone_id - 2, NUM_OF_DRONE)
            # Global -> Local 변환
            target_local = [
                formation_pos[0] - self.drone_spawn_absolute[0],
                formation_pos[1] - self.drone_spawn_absolute[1],
                formation_pos[2] - self.drone_spawn_absolute[2]
            ]

            flight_time = self.counter - (self.warmup_steps + 30)
            my_start_time = HOVER_WAIT_TIME + (self.drone_id - 2) * SEQUENCE_INTERVAL

            if flight_time < my_start_time:
                traj_msg.position = [0.0, 0.0, HOVER_ALTITUDE]
            else:
                traj_msg.position = [float(target_local[0]), float(target_local[1]), float(target_local[2])]

            traj_msg.yaw = 0.0 # 이륙시는 정면

            # 도착 체크
            dist = math.dist(self.current_absolute, formation_pos)
            if dist < FORMATION_DISTANCE_THRESHOLD:
                self.mission_phase = 1
                print(f"{self.drone_id} Phase 1(Formation) Complete")

        # [Phase 1] 대기 및 Phase 2 동기화
        elif self.mission_phase == 1:
            formation_pos = FORMATION_PATTERN(self.drone_id - 2, NUM_OF_DRONE)
            traj_msg.position = [
                float(formation_pos[0] - self.drone_spawn_absolute[0]),
                float(formation_pos[1] - self.drone_spawn_absolute[1]),
                float(formation_pos[2] - self.drone_spawn_absolute[2])
            ]
            traj_msg.yaw = 0.0

            if not self.ready_p1_reported:
                self.mission_sync.mark_ready_phase1(self.drone_id)
                self.ready_p1_reported = True
            
            if self.mission_sync.check_phase2_start():
                self.mission_phase = 2
                print(f"{self.drone_id} Phase 2(Surround) Start")

        # [Phase 2] 포위 (Staging Point 이동)
        elif self.mission_phase == 2:
            # 목표: 할당받은 구면 좌표
            target_pos = self.staging_point
            
            traj_msg.position = [
                float(target_pos[0] - self.drone_spawn_absolute[0]),
                float(target_pos[1] - self.drone_spawn_absolute[1]),
                float(target_pos[2] - self.drone_spawn_absolute[2])
            ]
            
            # Yaw: 타겟(중심)을 바라보게 설정
            traj_msg.yaw = self.calculate_yaw(self.target_absolute)

            dist = math.dist(self.current_absolute, target_pos)
            # 도착 체크
            #CHANGEABLE 도착 허용 오차 조정
            if dist < STAGING_DISTANCE_THRESHOLD:
                if not self.ready_p2_reported:
                    self.mission_sync.mark_ready_phase2(self.drone_id)
                    self.ready_p2_reported = True
            
            # 모든 드론이 포위 완료하면 공격 시작
            if self.mission_sync.check_phase3_start():
                self.mission_phase = 3
                print(f"{self.drone_id} Phase 3(Strike) Start")

        # [Phase 3] 타격 (중심으로 돌진) - Position 기반, 오버슈트로 감속 방지
        elif self.mission_phase == 3:
            # 현재 위치에서 타겟까지의 방향 벡터 계산
            vec_x = self.target_absolute[0] - self.current_absolute[0]
            vec_y = self.target_absolute[1] - self.current_absolute[1]
            vec_z = self.target_absolute[2] - self.current_absolute[2]
            
            # 거리 계산
            distance = math.sqrt(vec_x**2 + vec_y**2 + vec_z**2)
            
            # 단위 벡터로 정규화 (방향만 추출)
            if distance > 0.01:  # 0으로 나누기 방지
                unit_x = vec_x / distance
                unit_y = vec_y / distance
                unit_z = vec_z / distance
            else:
                # 이미 타겟에 도착했으면 현재 위치 유지
                unit_x = 0.0
                unit_y = 0.0
                unit_z = 0.0
            
            # 타겟을 지나 5m 더 뒤쪽의 오버슈트 목표 위치 계산 (Absolute 좌표)
            overshoot_distance = OVERSHOOT_DISTANCE
            overshoot_target_absolute = [
                self.target_absolute[0] + unit_x * overshoot_distance,
                self.target_absolute[1] + unit_y * overshoot_distance,
                self.target_absolute[2] + unit_z * overshoot_distance
            ]
            
            # Absolute -> Local 좌표 변환
            overshoot_target_local = [
                overshoot_target_absolute[0] - self.drone_spawn_absolute[0],
                overshoot_target_absolute[1] - self.drone_spawn_absolute[1],
                overshoot_target_absolute[2] - self.drone_spawn_absolute[2]
            ]
            
            # 오버슈트 목표 위치 명령 (드론이 감속 없이 타겟을 통과)
            traj_msg.position = [
                float(overshoot_target_local[0]),
                float(overshoot_target_local[1]),
                float(overshoot_target_local[2])
            ]
            
            # Yaw: 타겟을 계속 바라보도록 설정
            if self.attack_yaw is None:
                self.attack_yaw = self.calculate_yaw(self.target_absolute)
            traj_msg.yaw = self.attack_yaw

        self.trajectory_pub.publish(traj_msg)

    # ... (send_command, odometry_cb, timer_callback은 기존과 동일하거나 위 로직에 포함됨) ...
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
        if self.counter < self.warmup_steps:
            self.counter += 1
            return
        
        self.send_trajectory()
        
        if self.warmup_steps+10 <= self.counter < self.warmup_steps+20:
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        if self.warmup_steps+20 <= self.counter < self.warmup_steps+30:
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        
        self.counter += 1

def main(args=None):
    rclpy.init()
    executor = MultiThreadedExecutor()
    nodes = []

    mission_sync = MissionSync(NUM_OF_DRONE)
    
    # 공격 대기 지점(Staging Points) 미리 계산
    staging_points = get_fibonacci_sphere_points(NUM_OF_DRONE, ATTACK_STAGING_RADIUS, TARGET_ABSOLUTE)

    # enumerate를 써서 각 드론에게 고유한 Staging Point 할당
    for idx, drone_id in enumerate(TARGET_DRONE_INDEX):
        # staging_points[idx]를 해당 드론에게 전달
        node = DroneController(drone_id, mission_sync, staging_points[idx])
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