#TODO : 요구사항 정의서 완성해서 cursor로 기능 완성하기
import rclpy
import math
import threading
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, OffboardControlMode, VehicleOdometry

#--------------------------------설정---------------------------------#
NUM_OF_DRONE = 10
INPUT_TARGET_LOCATION = [0.0, -100.0, 50.0]

DRONE_SPAWN_X_GAP = 3.0
DRONE_SPAWN_Y = 0
DRONE_SPAWN_Z = -0.2

# [공격 파라미터]
ATTACK_SPHERE_RADIUS = 10.0  # 타격 전 정렬할 구면 반지름
CURVE_HANDLE_SCALE = 0.8     # 궤적을 얼마나 크게 부풀릴지 (클수록 둥글게)
ATTACK_SPEED = 25.0          # 공격 속도

FORMATION_PATTERN = lambda i, n: [10.0, (i - (n-1)/2) * 4.0, -50.0]
TARGET_DRONE_INDEX = [i for i in range(2, NUM_OF_DRONE+2)]
TARGET_ABSOLUTE = [INPUT_TARGET_LOCATION[1], INPUT_TARGET_LOCATION[0], -INPUT_TARGET_LOCATION[2]]

POINTS_SPHERE = []
#-------------------------------------------------------------------#

def generate_fibonacci_sphere(samples, radius, center):
    points = []
    phi = math.pi * (3. - math.sqrt(5.)) 
    for i in range(samples):
        y = 1 - (i / float(samples - 1)) * 2 
        r = math.sqrt(1 - y * y)
        theta = phi * i 
        x = math.cos(theta) * r
        z = math.sin(theta) * r
        
        px = center[0] + x * radius
        py = center[1] + y * radius
        pz = center[2] + z * radius
        points.append([px, py, pz])
    return points

class MissionSync:
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
    def __init__(self, drone_id, mission_sync: MissionSync) :
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
        
        idx = self.drone_id - 2
        if 0 <= idx < len(POINTS_SPHERE):
            self.sphere_point = POINTS_SPHERE[idx]
        else:
            self.sphere_point = self.target_absolute

        self.current_absolute = [0.0, 0.0, 0.0]
        self.mission_phase = 0
        self.mission_sync = mission_sync
        self.ready_reported = False
        self.wait_counter = 0
        self.warmup_steps = 30
        self.counter = 0
        
        self.bezier_start_time = 0
        self.flight_duration = 5.0
        self.bezier_p0 = None 
        self.bezier_p1 = None 
        self.bezier_p2 = None 
        self.bezier_p3 = None 

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

    def get_bezier_point(self, t, p0, p1, p2, p3):
        u = 1 - t
        tt = t * t
        uu = u * u
        uuu = uu * u
        ttt = tt * t
        p = [0.0]*3
        for i in range(3):
            p[i] = (uuu * p0[i]) + (3 * uu * t * p1[i]) + (3 * u * tt * p2[i]) + (ttt * p3[i])
        return p

    def get_bezier_velocity(self, t, p0, p1, p2, p3, duration):
        u = 1 - t
        d = [0.0]*3
        for i in range(3):
            d[i] = 3 * u * u * (p1[i] - p0[i]) + 6 * u * t * (p2[i] - p1[i]) + 3 * t * t * (p3[i] - p2[i])
        if duration > 0:
            return [val / duration for val in d]
        return [0.0, 0.0, 0.0]

    def send_offboard(self):
        off_msg = OffboardControlMode()
        off_msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        off_msg.position = True
        
        # [수정] 이륙 및 대기 단계(Phase 0, 1)에서는 속도 제어를 끕니다.
        # 공격 단계(Phase 2, 3)에서만 미사일 기동을 위해 속도 제어를 켭니다.
        if self.mission_phase >= 2:
            off_msg.velocity = True
        else:
            off_msg.velocity = False
            
        self.offboard_pub.publish(off_msg)
 
    def send_trajectory(self):
        traj_msg = TrajectorySetpoint()
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        
        # [Phase 0: 대형 형성]
        if self.mission_phase == 0:
            HOVER_ALTITUDE = -1.0         
            HOVER_WAIT_TIME = 30       
            SEQUENCE_INTERVAL = 5       
            formation_pos = FORMATION_PATTERN(self.drone_id - 2, NUM_OF_DRONE)
            target_local = [
                formation_pos[0] - self.drone_spawn_absolute[0],
                formation_pos[1] - self.drone_spawn_absolute[1],
                formation_pos[2] - self.drone_spawn_absolute[2],
            ]
            flight_time = self.counter - (self.warmup_steps + 30)
            my_start_time = HOVER_WAIT_TIME + (self.drone_id - 2) * SEQUENCE_INTERVAL
            
            if flight_time < my_start_time:
                traj_msg.position = [0.0, 0.0, HOVER_ALTITUDE]
            else:
                traj_msg.position = [float(target_local[0]), float(target_local[1]), float(target_local[2])]
            
            traj_msg.yaw = 0.0
            # Velocity 모드가 꺼져 있으므로 NaN을 보내도 무관하거나 무시됨
            traj_msg.velocity = [float('nan')]*3
            
            self.trajectory_pub.publish(traj_msg)
            
            dist = math.sqrt((self.current_absolute[0]-formation_pos[0])**2 + (self.current_absolute[1]-formation_pos[1])**2 + (self.current_absolute[2]-formation_pos[2])**2)
            if dist < 0.5:
                self.mission_phase = 1
                print(f"{self.drone_id} 대기")
        
        # [Phase 1: 대기]
        elif self.mission_phase == 1:
            formation_pos = FORMATION_PATTERN(self.drone_id - 2, NUM_OF_DRONE)
            target_local = [
                formation_pos[0] - self.drone_spawn_absolute[0],
                formation_pos[1] - self.drone_spawn_absolute[1],
                formation_pos[2] - self.drone_spawn_absolute[2],
            ]
            traj_msg.position = [float(target_local[0]), float(target_local[1]), float(target_local[2])]
            dx = self.target_absolute[0] - self.current_absolute[0]
            dy = self.target_absolute[1] - self.current_absolute[1]
            traj_msg.yaw = math.atan2(dy, dx)
            traj_msg.velocity = [float('nan')]*3
            self.trajectory_pub.publish(traj_msg)
            
            if not self.ready_reported:
                self.mission_sync.mark_ready(self.drone_id)
                self.ready_reported = True

            if self.mission_sync.attack_allowed():
                self.mission_phase = 2
                self.bezier_start_time = self.get_clock().now().nanoseconds
                
                # 베지에 곡선 설정 (Start -> Sphere Point)
                self.bezier_p0 = np.array(self.current_absolute)
                self.bezier_p3 = np.array(self.sphere_point)
                
                target_center = np.array(self.target_absolute)
                
                # Attack Vector: Sphere Point에서 Target을 바라보는 방향
                vec_attack = target_center - self.bezier_p3
                dist_attack = np.linalg.norm(vec_attack)
                if dist_attack > 0: dir_attack = vec_attack / dist_attack
                else: dir_attack = np.array([0,0,1])
                
                # P2: 진입 유도점 (공격 방향의 반대쪽 뒤로 뺌)
                dist_total = np.linalg.norm(self.bezier_p3 - self.bezier_p0)
                scale_factor = dist_total * CURVE_HANDLE_SCALE
                self.bezier_p2 = self.bezier_p3 - (dir_attack * scale_factor)
                
                # P1: 출발 유도점
                self.bezier_p1 = self.bezier_p0 + (self.bezier_p2 - self.bezier_p0) * 0.3
                
                # 리스트 변환
                self.bezier_p0 = list(self.bezier_p0)
                self.bezier_p1 = list(self.bezier_p1)
                self.bezier_p2 = list(self.bezier_p2)
                self.bezier_p3 = list(self.bezier_p3)
                
                curve_len = dist_total * 1.5 
                self.flight_duration = curve_len / ATTACK_SPEED
                print(f"{self.drone_id} 미사일 기동 시작")

        # [Phase 2: 곡선 비행]
        elif self.mission_phase == 2:
            now = self.get_clock().now().nanoseconds
            elapsed = (now - self.bezier_start_time) / 1e9
            
            if self.flight_duration > 0:
                t = elapsed / self.flight_duration
            else: t = 1.0
            
            if t >= 1.0:
                self.mission_phase = 3
                return

            next_pos = self.get_bezier_point(t, self.bezier_p0, self.bezier_p1, self.bezier_p2, self.bezier_p3)
            next_vel = self.get_bezier_velocity(t, self.bezier_p0, self.bezier_p1, self.bezier_p2, self.bezier_p3, self.flight_duration)
            
            traj_msg.position[0] = float(next_pos[0] - self.drone_spawn_absolute[0])
            traj_msg.position[1] = float(next_pos[1] - self.drone_spawn_absolute[1])
            traj_msg.position[2] = float(next_pos[2] - self.drone_spawn_absolute[2])
            traj_msg.velocity[0] = float(next_vel[0])
            traj_msg.velocity[1] = float(next_vel[1])
            traj_msg.velocity[2] = float(next_vel[2])
            
            vx, vy = next_vel[0], next_vel[1]
            if abs(vx) > 0.1 or abs(vy) > 0.1:
                traj_msg.yaw = math.atan2(vy, vx)
            self.trajectory_pub.publish(traj_msg)
            
        # [Phase 3: 직선 타격]
        elif self.mission_phase == 3:
            dx = self.target_absolute[0] - self.current_absolute[0]
            dy = self.target_absolute[1] - self.current_absolute[1]
            dz = self.target_absolute[2] - self.current_absolute[2]
            dist = math.sqrt(dx**2 + dy**2 + dz**2)
            
            if dist > 0.1:
                ux, uy, uz = dx/dist, dy/dist, dz/dist
                traj_msg.velocity[0] = float(ux * ATTACK_SPEED)
                traj_msg.velocity[1] = float(uy * ATTACK_SPEED)
                traj_msg.velocity[2] = float(uz * ATTACK_SPEED)
                
                # 목표물 뒤로 10m 관통 좌표 설정
                traj_msg.position[0] = float(self.target_absolute[0] + ux * 10.0 - self.drone_spawn_absolute[0])
                traj_msg.position[1] = float(self.target_absolute[1] + uy * 10.0 - self.drone_spawn_absolute[1])
                traj_msg.position[2] = float(self.target_absolute[2] + uz * 10.0 - self.drone_spawn_absolute[2])
                traj_msg.yaw = math.atan2(uy, ux)
            else:
                pass
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
        if self.counter < self.warmup_steps:
            self.counter += 1
            return
        self.send_trajectory()
        if self.warmup_steps+10<=self.counter<self.warmup_steps+20:
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        if self.warmup_steps+20<=self.counter<self.warmup_steps+30:
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.counter += 1

def main(args=None):
    global POINTS_SPHERE
    POINTS_SPHERE = generate_fibonacci_sphere(NUM_OF_DRONE, ATTACK_SPHERE_RADIUS, TARGET_ABSOLUTE)
    
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
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
