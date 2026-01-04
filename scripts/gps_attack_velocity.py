import math
import pymap3d as pm
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, OffboardControlMode, VehicleGlobalPosition, VehicleLocalPosition, VehicleOdometry

ORIGIN_LAT = 47.397742
ORIGIN_LON = 8.545594
ORIGIN_ALT = 0.0

###Warning!!!! 목표물 X 좌표를 반드시 음수로 해야함.
BALLOON_GAZEBO_X = -10.0  # 풍선 빨강축
BALLOON_GAZEBO_Y = 10.0  # 풍선 초록축
BALLOON_GAZEBO_Z = 3.0-1  # 풍선 높이

# 2. 좌표 변환 (발견한 규칙 적용)
# 드론 X (North) <== 풍선 Y (Green)
# 드론 Y (East)  <== 풍선 X (Red)
# 드론 Z (Down)  <== 풍선 Z (부호는 위로 가야 하니까 -)

TARGET_ABSOLUTE = [
    BALLOON_GAZEBO_Y,                   # 드론 X 자리에 풍선 Y 값 넣기
    BALLOON_GAZEBO_X,                   # 드론 Y 자리에 풍선 X 값 넣기
    -BALLOON_GAZEBO_Z #offset 적용
]

class Global_position_control(Node):
    def __init__(self):
        #Calling Node constructor
        super().__init__('global_position_control')

        #Attribute
        self.namespace = "/drone2"
        
        try:
            self.sys_id = int(self.namespace.replace("/drone",""))
        except ValueError:
            self.sys_id = 1

        #Qos 설정
        qos = QoSProfile(
            #UDP 같이 데이터를 빠르게 보내기만 하고 잘 도착했는지는 확인하지 않겠다.
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability = DurabilityPolicy.VOLATILE,
            #데이터를 Queue에 보관해서 정해진 갯수만큼만 보관하겠다. 옛날 데이터는 버린다.
            history = HistoryPolicy.KEEP_LAST,
            #Queue 크기
            depth = 1
        )

        #Publisher
        # 외부 컴퓨터로 제어할 것이라는 것을 알려주는 데이터 + 외부 컴퓨터가 살아있다는 데이터 + 위치,속도,pitch 드론의 무엇을 제어할지
        # 데이터가 계속 오지 않으면 드론은 저절로 offboard 모드를 끊고 failsafe로 들어감.
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            f'{self.namespace}/fmu/in/offboard_control_mode',
            qos
        )
        # 위치 제어의 실제 목표 지점 전달
        # 대부분 지속적으로 보내는게 좋음
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            f'{self.namespace}/fmu/in/trajectory_setpoint',
            qos
        )
        # 상태 변경. 시동켜기,끄기 offboard 모드 전환 이착륙 비상정지등을 할 수 있음
        # 보통 안전을 위해 10번정도 보냄.
        self.command_pub = self.create_publisher(
            VehicleCommand, 
            f'{self.namespace}/fmu/in/vehicle_command', 
            qos
        )

        #Subscriber
        #message를 받았다면 _cb 함수들이 실행된다.
        #따로 main함수에서 호출하지 않아도 constructor가 생성하자마자 데이터를 읽기 시작한다.
        self.global_sub = self.create_subscription(
            VehicleGlobalPosition, 
            f'{self.namespace}/fmu/out/vehicle_global_position',
            self.global_cb,qos
        )
        self.local_sub = self.create_subscription(
            VehicleOdometry, 
            f'{self.namespace}/fmu/out/vehicle_odometry',
            self.local_cb, qos
        )

        #global == GPS 좌표
        #local == 드론 스폰 중심 NED 좌표
        #absolute == 맵 중심 NED 좌표
        #TODO : attribute 중에 필요 없는거 삭제하기
        self.current_global = None
        self.current_local = None
        self.start_global = None
        self.is_neccesary_point_calculated = False
        self.target_absolute = TARGET_ABSOLUTE
        self.target_global = [None,None,None]
        self.target_local = [None,None,None]
        
        self.p_gain = 1.0       # 비례 제어 상수 (값이 클수록 목표에 빨리 반응하지만 오버슈트 위험)
        self.max_speed = 5.0    # 최대 속도 (m/s)
        
        self.timer = self.create_timer(0.1,self.timer_callback)
        self.counter = 0
        
        #GPS 잡고 있다는 log 한번만 띄우기 위한 flag
        self.wait_log_printed = False

        print("Controller constructed done")

    # 이륙하기 전에 스폰 위치 GPS 좌표 받기
    # current_global이 채워져 있어야 함
    # start_global과 target_local이 채워짐
    def set_start_global(self):
        #현재 위치를 start_global로 지정
        self.start_global = [
            self.current_global.lat,
            self.current_global.lon,
            self.current_global.alt
        ]

    # start_global이 채워져 있어야 함.
    def set_target_local(self):
        self.target_global = pm.ned2geodetic(
            *TARGET_ABSOLUTE[:3], 
            ORIGIN_LAT, ORIGIN_LON, ORIGIN_ALT
        )
        self.target_local = pm.geodetic2ned(
            *self.target_global,
            *self.start_global
        )
        self.target_local = [float(x) for x in self.target_local]

    def set_neccessary_point(self):
        self.set_start_global()
        self.set_target_local()
        self.is_neccesary_point_calculated = True
    #GPS 좌표 받기
    def global_cb(self,msg):
        self.current_global = msg
    
    #드론 스폰위치 기준 좌표 받기
    def local_cb(self,msg):
        self.current_local = msg
        class LocalPos:
                pass
        pos_data = LocalPos()
        pos_data.x = msg.position[0]
        pos_data.y = msg.position[1]
        pos_data.z = msg.position[2]
        self.current_local = pos_data
    #Command publishing
    def send_command(self,command,param1=0.0,param2=0.0):
        msg = VehicleCommand()
        #명령 작성 시간 기록. nanoseconds -> microseconds 단위 맞추기
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        #어떤 명령인지 기록
        msg.command = command
        #세부 옵션
        msg.param1 = param1
        msg.param2 = param2
        #수신인 기록
        msg.target_system = self.sys_id
        msg.target_component = 1
        #발신인 기록
        msg.source_system = self.sys_id
        msg.source_component = 1
        #외부 컴퓨터에서 온 명령임을 기록
        msg.from_external = True
        
        #보내기
        self.command_pub.publish(msg)

    #offboard publishing
    def send_offboard(self):
        off_msg = OffboardControlMode()
        off_msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        off_msg.position = False
        #속도 기반 이동 선언
        off_msg.velocity = True
        #off_msg.attitude 기체 기울기(자세)
        #Publish
        self.offboard_pub.publish(off_msg)

    #trajecotry publishing
    def send_velocity_trajectory(self):
        traj_msg = TrajectorySetpoint()
        #메세지 생성 시각 기록
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        
        err_x = self.target_local[0] - self.current_local.x
        err_y = self.target_local[1] - self.current_local.y
        err_z = self.target_local[2] - self.current_local.z
        
        vel_x = err_x * self.p_gain
        vel_y = err_y * self.p_gain
        vel_z = err_z * self.p_gain
        
        current_speed_sq = vel_x**2 + vel_y**2 + vel_z**2
        if current_speed_sq > self.max_speed**2:
            scale = self.max_speed / math.sqrt(current_speed_sq)
            vel_x *= scale
            vel_y *= scale
            vel_z *= scale
            
        traj_msg.position = [float('nan'), float('nan'), float('nan')]
        traj_msg.velocity = [float(vel_x), float(vel_y), float(vel_z)]
        
        # Yaw는 목표 지점을 바라보도록 설정
        traj_msg.yaw = math.atan2(err_y, err_x)

        self.trajectory_pub.publish(traj_msg)
        
    def timer_callback(self):
        #Heartbeat
        self.send_offboard()
        
        if self.current_global is None or self.current_local is None:
            if self.counter % 10 == 0: # 1초마다 출력
                print(f"대기 중... Global: {self.current_global is not None}, Local: {self.current_local is not None}")
            self.counter += 1
            return
        
        #start_global, target_local 설정
        if not self.is_neccesary_point_calculated:
            self.set_neccessary_point()
            print(f"초기화 완료: Target Local 설정됨 {self.target_local}")
            # 초기화 직후에는 바로 비행 로직으로 넘어가지 않고 한 틱 쉬거나, 바로 진행해도 무방함
            # 안전을 위해 여기서 return 해서 다음 틱부터 비행 시작
            return      
        
        #trajectory publish
        self.send_velocity_trajectory()
        
        #offboard
        if 40 <= self.counter < 50:
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            
            # 로그는 40일 때 한 번만 출력
            if self.counter == 40:
                print("Offboard 모드 전환 명령 전송 (10회 반복 시작)")
        if 50 <= self.counter < 60:
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            
            # 로그는 50일 때 한 번만 출력
            if self.counter == 50:
                print("Arming(시동) 명령 전송 (10회 반복 시작)")
        
        #counter 증가
        self.counter += 1
        
def main():
    rclpy.init()
    node = Global_position_control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    