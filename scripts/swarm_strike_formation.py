import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    TrajectorySetpoint,
    VehicleCommand,
    OffboardControlMode,
    VehicleGlobalPosition,
    SensorGps,
)

#Global variable
TARGET_DRONE_IDS = [2,3,4,5,6]

Drone_spawn_gap = 3.0

class DroneController(Node):
    def __init__(self, drone_id, start_delay_sec=0.0):
        #calling parent class contructor
        super().__init__(f'{drone_id}_node')
        #attributes
        self.drone_id = drone_id
        self.namespace = f"/{drone_id}"

        try: 
            self.sys_id = int(drone_id.replace("drone", ""))
        except ValueError:
            # if error occurs during parsing, set sys_id to 1
            # It won't move anyway
            print(f"Error in parsing drone_id = {drone_id}")
            self.sys_id = 1
        
        # QoS 설정 (PX4 bridge의 out 토픽과 호환)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 최신 GPS/글로벌 포지션 저장용 변수
        self.latest_gpos = None  # VehicleGlobalPosition
        self.latest_sensor_gps = None  # SensorGps (원시 GPS)

        # PX4 → ROS2 bridge 출토픽 구독 (각 드론 네임스페이스별)
        # 주로 사용: vehicle_global_position (deg/deg/m 단위의 위경도/고도)
        self.gpos_sub = self.create_subscription(
            VehicleGlobalPosition,
            f"{self.namespace}/fmu/out/vehicle_global_position",
            self._gpos_cb,
            qos,
        )

        # 필요 시: 원시 GPS 데이터 (fix_type, 위성수 등 확인)
        self.sensor_gps_sub = self.create_subscription(
            SensorGps,
            f"{self.namespace}/fmu/out/sensor_gps",
            self._sensor_gps_cb,
            qos,
        )

        # 1Hz로 GPS 상태 로그 (데이터 흐름 확인용)
        self.gps_log_timer = self.create_timer(1.0, self._log_gps_status)

    # 콜백: VehicleGlobalPosition
    def _gpos_cb(self, msg: VehicleGlobalPosition):
        self.latest_gpos = msg

    # 콜백: SensorGps (원시 GPS)
    def _sensor_gps_cb(self, msg: SensorGps):
        self.latest_sensor_gps = msg

    # 주기적 상태 출력 (테스트/디버그용)
    def _log_gps_status(self):
        if self.latest_gpos is None:
            self.get_logger().info(f"[{self.drone_id}] GPS 대기 중… (vehicle_global_position 미수신)")
            return

        lat = self.latest_gpos.lat  # degrees
        lon = self.latest_gpos.lon  # degrees
        alt = self.latest_gpos.alt  # meters AMSL

        # 일부 필드는 펌웨어/브랜치에 따라 없을 수 있으므로 getattr 사용
        eph = getattr(self.latest_gpos, 'eph', float('nan'))
        epv = getattr(self.latest_gpos, 'epv', float('nan'))

        # 원시 GPS fix_type 정보가 있으면 함께 출력
        fix = self.latest_sensor_gps.fix_type if self.latest_sensor_gps else None

        self.get_logger().info(
            f"[{self.drone_id}] GPS: lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f} m, eph={eph:.2f}, epv={epv:.2f}, fix={fix}"
        )




