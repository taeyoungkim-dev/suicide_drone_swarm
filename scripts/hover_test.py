import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, OffboardControlMode

class HoverNode(Node):
    def __init__(self):
        super().__init__('hover_node')
        
        # === [중요] 네임스페이스 설정 ===
        # 아까 ros2 topic list에서 확인한 이름이 'drone1'이었으므로 이걸 붙여야 합니다.
        self.namespace = "/drone1" 
        self.target_pos = [0.0, 0.0, -5.0]

        print(f"🚁 Hovering Test Start! Target: {self.target_pos}")

        # QoS 설정
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher 생성 (주소 수정됨: namespace 추가)
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
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) # 시간 도장 추가
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def timer_callback(self):
        # 1. Heartbeat (Offboard 모드 유지용)
        off_msg = OffboardControlMode()
        off_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        off_msg.position = True
        off_msg.velocity = False
        off_msg.acceleration = False
        self.offboard_pub.publish(off_msg)

        # 2. 위치 명령 전송
        traj_msg = TrajectorySetpoint()
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        traj_msg.position = self.target_pos
        traj_msg.yaw = 0.0 
        self.trajectory_pub.publish(traj_msg)

        # 3. 시퀀스 제어
        # 1초 뒤: Offboard 모드 변경 (안전을 위해 10번 반복 전송)
        if 10 <= self.counter < 20:
            if self.counter % 5 == 0:
                print(f">> Requesting Offboard Mode... ({self.counter})")
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

        # 2초 뒤: 시동 (Arming) (안전을 위해 10번 반복 전송)
        if 20 <= self.counter < 30:
            if self.counter % 5 == 0:
                print(f">> Arming... ({self.counter})")
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        self.counter += 1

def main():
    rclpy.init()
    node = HoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()