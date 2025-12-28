import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, OffboardControlMode

# === ⚙️ 설정 구역 ===
# 1. 제어할 드론 ID 목록 (2번부터 6번까지)
# 가장 먼 드론(6번)이 움직이지 않던 문제를 해결합니다.
TARGET_DRONE_IDS = [2, 3, 4, 5, 6]

# 2. X, Y축 반전 설정 (True로 설정 시 X와 Y 입력을 맞바꿈)
# "움직임이 거꾸로 된 것 같다"는 문제를 해결합니다.
SWAP_XY = True 

# 3. 절대 좌표 설정
# 목표: X=30m 지점 (풍선 뒤쪽 관통), Y=9m (대열 중앙)
GLOBAL_TARGET_X = 22.0
GLOBAL_TARGET_Y = 12.0
GLOBAL_TARGET_Z = -7.0

# 4. 드론 배치 간격 (Y축으로 3m씩)
SPAWN_Y_INTERVAL = 3.0
# ===================

class DroneController(Node):
    def __init__(self, drone_id):
        super().__init__(f'{drone_id}_node')
        self.drone_id = drone_id
        self.namespace = f"/{drone_id}"
        
        try:
            self.sys_id = int(drone_id.replace("drone", ""))
        except ValueError:
            self.sys_id = 1

        # 🧠 좌표 계산
        # 1. 나의 절대 시작 위치 추정 (Y축 배치 가정)
        # drone2 -> 6m, drone3 -> 9m ...
        my_spawn_global_y = self.sys_id * SPAWN_Y_INTERVAL
        my_spawn_global_x = 0.0
        
        # 2. 이동해야 할 거리 계산 (목표 - 내 위치)
        calc_x = GLOBAL_TARGET_X - my_spawn_global_x
        calc_y = GLOBAL_TARGET_Y - my_spawn_global_y
        
        # 3. [핵심 수정] X, Y축 스왑 적용
        if SWAP_XY:
            # X명령에 Y계산값을, Y명령에 X계산값을 넣음
            self.target_pos = [calc_y, calc_x, GLOBAL_TARGET_Z]
            print(f"🔄 {self.drone_id} (XY 반전됨) -> 목표: 앞(X)={calc_y:.1f}, 우(Y)={calc_x:.1f}")
        else:
            self.target_pos = [calc_x, calc_y, GLOBAL_TARGET_Z]
            print(f"⚔️ {self.drone_id} -> 목표: 앞(X)={calc_x:.1f}, 우(Y)={calc_y:.1f}")

        # QoS 및 Publisher 설정
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        self.offboard_pub = self.create_publisher(OffboardControlMode, f'{self.namespace}/fmu/in/offboard_control_mode', qos)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, f'{self.namespace}/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(VehicleCommand, f'{self.namespace}/fmu/in/vehicle_command', qos)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0
        self.warmup_steps = 50

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
        # 1. Heartbeat
        off_msg = OffboardControlMode()
        off_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        off_msg.position = True
        off_msg.velocity = False
        off_msg.acceleration = False
        self.offboard_pub.publish(off_msg)

        # 2. 워밍업
        if self.counter < self.warmup_steps:
            if self.counter % 20 == 0:
                print(f"⏳ {self.drone_id} 대기 중... {self.counter}/{self.warmup_steps}")
            self.counter += 1
            return 

        # 3. 위치 명령 전송
        traj_msg = TrajectorySetpoint()
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        traj_msg.position = self.target_pos
        traj_msg.yaw = 0.0 
        self.trajectory_pub.publish(traj_msg)

        # 4. 시퀀스 제어
        step_active = self.counter - self.warmup_steps
        
        if 10 <= step_active < 20:
            if step_active % 5 == 0: # 메시지 너무 자주 뜨지 않게 조절
                print(f"[{self.drone_id}] Offboard 모드!")
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

        if 20 <= step_active < 35:
            if step_active % 5 == 0:
                print(f"[{self.drone_id}] 시동 및 이륙!")
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        self.counter += 1

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    nodes = []
    
    print(f"🔥 드론 ID {TARGET_DRONE_IDS} 제어 시작 (XY 반전: {SWAP_XY})")

    # [수정] 지정된 ID 리스트(2,3,4,5,6)에 대해서만 노드 생성
    for i in TARGET_DRONE_IDS:
        drone_name = f"drone{i}"
        node = DroneController(drone_name)
        nodes.append(node)
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        print("\n🛑 시스템 종료.")
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()