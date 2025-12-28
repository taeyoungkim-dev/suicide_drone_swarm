import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, OffboardControlMode

class DroneController(Node):
    def __init__(self, drone_id, target_pos):
        super().__init__(f'{drone_id}_node')
        
        self.drone_id = drone_id
        self.target_pos = target_pos
        self.namespace = f"/{drone_id}"
        
        # 이름에서 숫자만 추출 (예: "drone2" -> 2, "drone10" -> 10)
        # 이 숫자를 시스템 ID로 사용합니다.
        try:
            self.sys_id = int(drone_id.replace("drone", ""))
        except ValueError:
            self.sys_id = 1 # 실패 시 기본값
            
        print(f"🚀 {self.drone_id} (SysID: {self.sys_id}) Ready! Target: {self.target_pos}")

        # QoS 설정
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher 생성
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, f'{self.namespace}/fmu/in/offboard_control_mode', qos)
        
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, f'{self.namespace}/fmu/in/trajectory_setpoint', qos)
        
        self.command_pub = self.create_publisher(
            VehicleCommand, f'{self.namespace}/fmu/in/vehicle_command', qos)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0

    def send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = self.sys_id  # 추출한 ID 사용 (drone2 -> 2)
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

        # 2. 위치 명령
        traj_msg = TrajectorySetpoint()
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        traj_msg.position = self.target_pos
        traj_msg.yaw = 0.0 # 북쪽(X축) 보기
        self.trajectory_pub.publish(traj_msg)

        # 3. 시퀀스 제어
        if 10 <= self.counter < 20:
            if self.counter % 5 == 0:
                print(f"[{self.drone_id}] Requesting Offboard...")
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

        if 20 <= self.counter < 30:
            if self.counter % 5 == 0:
                print(f"[{self.drone_id}] Arming...")
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        self.counter += 1

def main():
    rclpy.init()
    
    # === 🛠️ 수정됨: 실제 생성된 drone2, drone3 사용 ===
    # 풍선 위치: x=0, y=15, z=5 (PX4 기준 z=-5.5)
    
    # Drone 2: 왼쪽(y=10)에서 시작했다고 가정하고 공격
    d2 = DroneController('drone2', [12.0, 0.0, -6.3])
    
    # Drone 3: 오른쪽(y=20)에서 시작했다고 가정하고 공격
    d3 = DroneController('drone3', [10.0, 0.0, -6.3])
    
    executor = MultiThreadedExecutor()
    executor.add_node(d2)
    executor.add_node(d3)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        d2.destroy_node()
        d3.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()