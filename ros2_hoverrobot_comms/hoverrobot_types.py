from collections import namedtuple
from enum import Enum
import struct 

class RobotStatusCode(Enum):
    STATUS_ROBOT_INIT = 0
    STATUS_ROBOT_DISABLE = 1
    STATUS_ROBOT_ARMED = 2
    STATUS_ROBOT_STABILIZED = 3
    STATUS_ROBOT_CHARGING = 4
    STATUS_ROBOT_ERROR = 5
    STATUS_ROBOT_ERROR_LIMIT_SPEED = 6
    STATUS_ROBOT_ERROR_LIMIT_ANGLE = 7
    STATUS_ROBOT_ERROR_MCB_CONNECTION = 8
    STATUS_ROBOT_ERROR_IMU = 9
    STATUS_ROBOT_ERROR_HALL_L = 10
    STATUS_ROBOT_ERROR_HALL_R = 11
    STATUS_ROBOT_ERROR_TEMP = 12
    STATUS_ROBOT_ERROR_BATTERY = 13
    STATUS_ROBOT_TEST_MODE = 14

class CommandsRobotCode(Enum):
    COMMAND_CALIBRATE_IMU = 0
    COMMAND_SAVE_LOCAL_CONFIG = 1
    COMMAND_ARMED_ROBOT = 2
    COMMAND_DEARMED_ROBOT = 3
    COMMAND_CLEAN_WHEELS = 4
    COMMAND_VIBRATION_TEST = 5 
    COMMAND_MOVE_FORWARD = 6
    COMMAND_MOVE_BACKWARD = 7
    COMMAND_MOVE_ABS_YAW = 8
    COMMAND_MOVE_REL_YAW = 9

class RobotHeaderPackage(Enum):
    HEADER_PACKAGE_STATUS = 0xAB01           # key que indica que el paquete recibido es dynamic data
    HEADER_PACKAGE_CONTROL = 0xAB02          # key que indica que el paquete enviado es de control
    HEADER_PACKAGE_SETTINGS = 0xAB03         # key que indica que el paquete enviado es de configuracion
    HEADER_PACKAGE_COMMAND = 0xAB04          # key que indica que el paquete enviado es un comando

FORMAT_COMMAND_ROBOT = "<2H1h"  # 2 uint16, 1 int16, Cambia a ! si es big endian
# COMMAND_ROBOT_PACKET_SIZE = struct.calcsize(FORMAT_COMMAND_ROBOT)
RobotCommandData = namedtuple('RobotCommandsData', [
    'headerPackage',
    'command',
    'value'
])

FORMAT_CONTROL_ROBOT = "<H2h"  # 1 uint16, 2 int16, Cambia a ! si es big endian
# CONTROL_ROBOT_PACKET_SIZE = struct.calcsize(FORMAT_CONTROL_ROBOT)
RobotControlData = namedtuple('RobotControlData', [
    'headerPackage',
    'angularVel',
    'linearVel'
])

FORMAT_DYNAMYC_ROBOT = "<6H17h2H"  # 6 uint16, 17 int16, 2 uint16, Cambia a ! si es big endian
DYNAMIC_ROBOT_PACKET_SIZE = struct.calcsize(FORMAT_DYNAMYC_ROBOT)
RobotDynamicData = namedtuple('RobotData', [    # TODO: reemplazar el 'RobotData'
    'headerPackage',
    'isCharging',
    'batVoltage',
    'imuTemp',
    'mcbTemp',
    'mainboardTemp',
    'speedR',
    'speedL',
    'currentR',
    'currentL',
    'pitch',
    'roll',
    'yaw',
    'collisionFL',
    'collisionFR',
    'collisionRL',
    'collisionRR',
    'posInMeters',
    'outputYawControl',
    'setPointAngle',
    'setPointPos',
    'setPointYaw',
    'setPointSpeed',
    'centerAngle',
    'statusCode'
])