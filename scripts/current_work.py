import rclpy
import math
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, OffboardControlMode

#--------------------------------수정---------------------------------#
NUM_OF_DRONE = 10
INPUT_TARGET_LOCATION = []
#드론 소환 (0,3,0) , (0,6,0) , (0,9,0) , (0,12,0) ...
DRONE_SPAWN_Y_GAP = 3.0
#-------------------------------------------------------------------#


TARGET_DRONE_IDS = [i for i in range(2,NUM_OF_DRONE+2)]
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
        self.drone_spawn_absolute = [0,int(drone_id)*DRONE_SPAWN_Y_GAP,0.2]
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

        self.timer = self.create_timer(0.02, self.timer_callback)
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
        #목표 지점 local 좌표
        traj_msg.position = self.target_local
        #Heading 방향
        traj_msg.yaw = math.atan2(
            self.target_local[1] - self.current_local.y,
            self.target_local[0] - self.current_local.x
        )
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

    for drone_id in TARGET_DRONE_IDS:
        node = DroneController(drone_id)






        
