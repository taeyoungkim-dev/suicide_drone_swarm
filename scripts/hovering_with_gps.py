import math
import pymap3d as pm
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, OffboardControlMode, VehicleGlobalPosition, VehicleLocalPosition

ORIGIN_LAT = 47.397742
ORIGIN_LON = 8.545594
ORIGIN_ALT = 0

TARGET_POSTION = [0,0,-3]



class Global_position_control(Node):
    def __init__(self):
        #Calling Node constructor
        super().__init__('global_position_control')

        #Attribute
        self.namespace = "/drone2"
        self.sys_id = int(self.namespace.replace("/drone",""))

        #Qos 설정
        qos = QoSProfile(
            #UDP 같이 데이터를 빠르게 보내기만 하고 잘 도착했는지는 확인하지 않겠다.
            reliability = ReliabilityPolicy.BEST_EFFORT,
            #Publisher(쓰는 애)가 살아있는 한 subscriber(읽는 애)가 쓰는 타이밍 보다 늦게 들어오더라도
            #데이터를 삭제하지 않고 읽을 수 있도록 남겨두겠다.
            durability = DurabilityPolicy.TRANSIENT_LOCAL,
            #데이터를 Queue에 보관해서 정해진 갯수만큼만 보관하겠다. 옛날 데이터는 버린다.
            history = HistoryPolicy.KEEP_LAST,
            #Queue 크기
            depth = 1
        )

        #Publisher
        # 외부 컴퓨터로 제어할 것이라는 것을 알려주는 데이터 + 외부 컴퓨터가 살아있다는 데이터 + 위치,속도,pitch 드론의 무엇을 제어할지
        # 데이터가 계속 오지 않으면 드론은 저절로 offboard 모드를 끊고 failsafe로 들어감.
        self.offboard_pub = self.create_publisher(OffboardControlMode, f'{self.namespace}/fmu/in/offboard_control_mode', qos)
        # 위치 제어의 실제 목표 지점 전달
        # 대부분 지속적으로 보내는게 좋음
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, f'{self.namespace}/fmu/in/trajectory_setpoint', qos)
        # 상태 변경. 시동켜기,끄기 offboard 모드 전환 이착륙 비상정지등을 할 수 있음
        # 보통 안전을 위해 10번정도 보냄.
        self.command_pub = self.create_publisher(VehicleCommand, f'{self.namespace}/fmu/in/vehicle_command', qos)

        #Subscriber
        #message를 받았다면 _cb 함수들이 실행된다.
        self.global_sub = self.create_subscription(VehicleGlobalPosition, f'{self.namespace}/fmu/out/vehicle_global_position',self.global_cb,qos)
        self.local_sub = self.create_subscription(VehicleLocalPosition, f'{self.namespace}/fmu/out/vehicle_local_position',self.local_cb,qos)

        #global == GPS 좌표
        #local == 드론 스폰 중심 NED 좌표
        #absolute == 맵 중심 NED 좌표
        self.current_global = None
        self.current_local = None
        self.start_global = None
        self.setpoint_calculated = False
        self.target_absolute_postion = TARGET_POSTION
        
        self.timer = self.create_timer(0.1,self.timer_callback)
        self.counter = 0

        print("Controller constructed done")

    # 이륙하기 전에 스폰 위치 GPS 좌표 받기
    def set_start_positon(self):
        #현재 위치를 start_global로 지정
        self.start_global = self.current_global
        #start_global을 start_absolute로 변경
        self.start_absolute_position = 
        #TODO 
        #target_absolute_positon - start_absolute 로 목표 벡터 계산

        self.target_local_postion = 

    #GPS 좌표 받기
    def global_cb(self,msg):
        self.current_global = msg
    
    #드론 스폰위치 기준 좌표 받기
    def local_cb(self,msg):
        self.current_local = msg
    
    #두 위치 사이의 거리를 미터 단위로 변환
    def get_distance_meters(self,lat1,lon1,lat2,lon2):

        R = 6371000.0
        d_lat = math.radians(lat2 - lat1)
        d_lon = math.radians(lon2 - lon1)
        lat1_rad = math.radians(lat1)

        # x: North, y: East
        x = d_lat * R
        y = d_lon * R * math.cos(lat1_rad)
        
        return x, y
    
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
        off_msg.position = True
        self.offboard_pub.publish(off_msg)

    def send_trajectory(self):
        traj_msg = TrajectorySetpoint()
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds/1000)
        traj_msg.position = #TODO
    def timer_callback(self):
        #Heartbeat
        off_msg = OffboardControlMode()
        off_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        #TODO


    #GPS 좌표를 드론 스폰 기준 좌표로 바꾸는 함수
    def local_to_global(self, local):
        # ned2geodetic: 로컬 NED를 위경도로 변환
        lat, lon, alt = pm.ned2geodetic(
            local[0], local[1], local[2],
            ORIGIN_LAT, 
            ORIGIN_LON, 
            ORIGIN_LAT
        )
        return lat, lon, alt

    #드론 스폰 기준 좌표를 GPS 좌표로 바꾸는 함수
    def global_to_local(self, lat, lon, alt):
        x, y, z = pm.geodetic2ned(
            lat, lon, alt,
            ORIGIN_LAT, 
            ORIGIN_LON, 
            ORIGIN_LAT
        )
        return x, y, z