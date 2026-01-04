import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, OffboardControlMode
import pymap3d as pm

# === ⚙️ 설정 구역 ===
TARGET_DRONE_IDS = [2, 3, 4, 5, 6]
SWAP_XY = True 

# 좌표 설정
GLOBAL_TARGET_X = -10.0
GLOBAL_TARGET_Y = 10.0
GLOBAL_TARGET_Z = -6.0 -1.0
SPAWN_Y_INTERVAL = 3.0

# [추가] 드론 간 순차 출발 간격 (초 단위)
# 1.5초마다 한 대씩 이륙합니다.
LAUNCH_GAP_SEC = 0.0
# ===================

class DroneController(Node):
    # [수정] start_delay 인자 추가 (기본값 0)
    def __init__(self, drone_id, start_delay_sec=0.0):
        # calling parent class contructor
        super().__init__(f'{drone_id}_node')
        # attributes
        self.drone_id = drone_id
        self.namespace = f"/{drone_id}"
        
        try:
            self.sys_id = int(drone_id.replace("drone", ""))
        except ValueError:
            self.sys_id = 1

        # 🧠 좌표 계산
        my_spawn_global_y = self.sys_id * SPAWN_Y_INTERVAL
        my_spawn_global_x = 0.0
        
        calc_x = GLOBAL_TARGET_X - my_spawn_global_x
        calc_y = GLOBAL_TARGET_Y - my_spawn_global_y

        if SWAP_XY:
            self.target_pos = [calc_y, calc_x, GLOBAL_TARGET_Z]
            print(f"🔄 {self.drone_id} (지연 {start_delay_sec}초) -> 목표: 앞={calc_y:.1f}, 우={calc_x:.1f}")
        else:
            self.target_pos = [calc_x, calc_y, GLOBAL_TARGET_Z]
            print(f"⚔️ {self.drone_id} (지연 {start_delay_sec}초) -> 목표: 앞={calc_x:.1f}, 우={calc_y:.1f}")

        # QoS 및 Publisher 설정
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        self.offboard_pub = self.create_publisher(OffboardControlMode, f'{self.namespace}/fmu/in/offboard_control_mode', qos)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, f'{self.namespace}/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(VehicleCommand, f'{self.namespace}/fmu/in/vehicle_command', qos)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0
        self.warmup_steps = 50
        
        # [핵심] 지연 시간을 스텝(0.1초 단위)으로 변환하여 저장
        self.delay_steps = int(start_delay_sec * 10)

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
        # 1. Heartbeat (항상 전송)
        off_msg = OffboardControlMode()
        off_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        off_msg.position = True
        off_msg.velocity = False
        off_msg.acceleration = False
        self.offboard_pub.publish(off_msg)

        # 2. 워밍업 (모든 드론 공통 대기)
        if self.counter < self.warmup_steps:
            if self.counter % 20 == 0:
                print(f"⏳ {self.drone_id} 시스템 안정화 중... {self.counter}/{self.warmup_steps}")
            self.counter += 1
            return 

        # --- 워밍업 끝 ---

        # 3. 위치 명령 전송 (미리 보내놔야 Offboard 전환 시 튀지 않음)
        traj_msg = TrajectorySetpoint()
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        traj_msg.position = self.target_pos
        traj_msg.yaw = 0.0 
        self.trajectory_pub.publish(traj_msg)

        # 4. 시퀀스 제어 (시간차 적용)
        # 전체 실행 시간에서 워밍업 시간을 뺌
        time_since_warmup = self.counter - self.warmup_steps
        
        # [핵심] 내 지연 시간만큼 명령 시점을 뒤로 미룸
        # 예: delay가 20이면, 남들이 0초에 할 때 나는 2초(20스텝) 뒤에 시작
        my_sequence_step = time_since_warmup - self.delay_steps

        # 아직 내 차례가 아니면 대기 (명령 안 보냄)
        if my_sequence_step < 0:
            if my_sequence_step % 10 == 0:
                print(f"✋ {self.drone_id} 대기 중... (출격까지 {abs(my_sequence_step)/10:.1f}초)")
            self.counter += 1
            return

        # 내 차례가 되면 명령 시작!
        
        # 1초 구간: Offboard 모드 요청
        if 10 <= my_sequence_step < 20:
            if my_sequence_step % 5 == 0:
                print(f"🚀 [{self.drone_id}] Offboard 모드 요청!")
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

        # 2초 구간: 시동 및 이륙
        if 20 <= my_sequence_step < 35:
            if my_sequence_step % 5 == 0:
                print(f"🔥 [{self.drone_id}] 시동 및 이륙!")
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        self.counter += 1

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    nodes = []
    
    print(f"🔥 드론 ID {TARGET_DRONE_IDS} 순차 타격 시작 (간격: {LAUNCH_GAP_SEC}초)")

    # [수정] 인덱스(i)를 사용하여 지연 시간 계산
    for index, drone_id_num in enumerate(TARGET_DRONE_IDS):
        drone_name = f"drone{drone_id_num}"
        
        # 0번(첫 번째) 드론은 지연 0초
        # 1번(두 번째) 드론은 1.5초, 2번은 3.0초...
        delay = index * LAUNCH_GAP_SEC
        
        node = DroneController(drone_name, start_delay_sec=delay)
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