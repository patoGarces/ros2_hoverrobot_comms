import struct
import queue 
import time 
import threading
from ros2_hoverrobot_comms.hoverrobot_client_socket import SocketClient
from ros2_hoverrobot_comms.hoverrobot_client_serial import SerialClient
from ros2_hoverrobot_comms.hoverrobot_types import DYNAMIC_ROBOT_PACKET_SIZE, FORMAT_DYNAMYC_ROBOT, FORMAT_COMMAND_ROBOT, FORMAT_CONTROL_ROBOT, RobotStatusCode, RobotDynamicData, CommandsRobotCode, RobotHeaderPackage
from ros2_hoverrobot_comms.base_transport import BaseTransport, Transport

class HoverRobotComms():

    def __init__(self, logger, transport, reconnectDelay):

        self.queueSender = queue.Queue()
        self.queueReceiver = queue.Queue()
        self.queueDynamicData = queue.Queue()

        if transport == Transport.TCP_IP:
            self.transportModule = SocketClient(logger= logger, reconnectDelay=reconnectDelay, sendQueue=self.queueSender, recvQueue=self.queueReceiver)
        else:
            self.transportModule = SerialClient(logger= logger, sendQueue=self.queueSender, recvQueue=self.queueReceiver)
        self.logger = logger

        # Buffer global para acumular fragmentos de paquetes
        self.receive_buffer = b''

        self.parsedDynamicData = None
        self.thread = threading.Thread(target=self.__processData, daemon=True)
        self.thread.start()

    def connectToRobot(self, serverIp, serverPort):
        self.transportModule.connect(serverIp, serverPort)

    def isRobotConnected(self): 
        return self.transportModule.isConnected()
    
    def sendControl(self, linearVel, angularVel):
        if (self.queueSender is not None and self.isRobotConnected()
            and self.parsedDynamicData is not None
            and self.parsedDynamicData.statusCode == RobotStatusCode.STATUS_ROBOT_STABILIZED.value
            ):
            packed = struct.pack(FORMAT_CONTROL_ROBOT, 
                        RobotHeaderPackage.HEADER_PACKAGE_CONTROL.value, 
                        int(angularVel * 100.00),
                        int(linearVel * 100.00), 
                    )
            self.queueSender.put(packed)
            return True
        else:
            return False

    def sendCommand(self, command, value): 
        self.logger.info(f'intento de sendCommand, isConnected: {self.isRobotConnected()}, {self.parsedDynamicData is not None}')
        if (self.queueSender is not None and self.isRobotConnected() 
            and self.parsedDynamicData is not None
            and self.parsedDynamicData.statusCode == RobotStatusCode.STATUS_ROBOT_STABILIZED.value
            ):
            packed = struct.pack(FORMAT_COMMAND_ROBOT, 
                        RobotHeaderPackage.HEADER_PACKAGE_COMMAND.value, 
                        command.value, 
                        int(value))
            self.queueSender.put(packed)
            return True
        else:
            return False
        
    def getRobotStatus(self):
        return getattr(self.parsedDynamicData, "statusCode", RobotStatusCode.STATUS_ROBOT_INIT)
    
    def getQueueDynamicData(self):
        return self.queueDynamicData

    def __processData(self):
        while True:
            try:
                while not self.queueReceiver.empty():
                    raw_data = self.queueReceiver.get_nowait()
                    self.receive_buffer += raw_data

                    while True:
                        sync_pos = self.receive_buffer.find(struct.pack('<H', RobotHeaderPackage.HEADER_PACKAGE_STATUS.value))
                        if sync_pos == -1:
                            self.receive_buffer = b''
                            break

                        if sync_pos > 0:
                            self.receive_buffer = self.receive_buffer[sync_pos:]

                        if len(self.receive_buffer) < DYNAMIC_ROBOT_PACKET_SIZE:
                            break

                        packet = self.receive_buffer[:DYNAMIC_ROBOT_PACKET_SIZE]
                        self.receive_buffer = self.receive_buffer[DYNAMIC_ROBOT_PACKET_SIZE:]

                        try:
                            unpacked = struct.unpack(FORMAT_DYNAMYC_ROBOT, packet)
                            self.parsedDynamicData = RobotDynamicData(*unpacked)
                            if self.queueDynamicData is not None:
                                self.queueDynamicData.put(self.parsedDynamicData)
                        except struct.error as e:
                            self.logger.warning(f"Error al parsear paquete: {e}")

                # Delay descanso solo si la cola está vacía
                time.sleep(0.001)

            except Exception as e:
                self.logger.error(f"Error en __processData: {e}")

    def sendMockStatus(self): 

        command = RobotDynamicData(
            headerPackage = RobotHeaderPackage.HEADER_PACKAGE_STATUS.value,
            isCharging = True,
            batVoltage = 1,
            imuTemp = 2,
            mcbTemp = 3,
            mainboardTemp = 4,
            speedR = 5,
            speedL = 6,
            currentR = 7,
            currentL = 8,
            pitch = 9,
            roll = 10,
            yaw = 11,
            collisionFL = 12,
            collisionFR = 13,
            collisionRL = 14,
            collisionRR = 15,
            posInMeters = 16,
            outputYawControl =17,
            setPointAngle = 18,
            setPointPos = 19,
            setPointYaw = 20,
            setPointSpeed = 21,
            statusCode = RobotStatusCode.STATUS_ROBOT_STABILIZED.value
        )

        self.logger.info(f'mock de statusRobot para pruebas: {command}')
        if (self.queueSender is not None and self.isRobotConnected()):
            packed = struct.pack(
                FORMAT_DYNAMYC_ROBOT, 
                command.headerPackage,
                command.isCharging,
                command.batVoltage,
                command.imuTemp,
                command.mcbTemp,
                command.mainboardTemp,
                command.speedR,
                command.speedL,
                command.currentR,
                command.currentL,
                command.pitch,
                command.roll,
                command.yaw,
                command.collisionFL,
                command.collisionFR,
                command.collisionRL,
                command.collisionRR,
                command.posInMeters,
                command.outputYawControl,
                command.setPointAngle,
                command.setPointPos,
                command.setPointYaw,
                command.setPointSpeed,
                command.statusCode
                )
            self.queueSender.put(packed)
            return True
        else:
            return False     


SERVER_IP = '192.168.0.101'
SERVER_PORT = 8080
RECONNECT_DELAY = 5  # segundos entre reintentos

def _get_default_logger():
        # Logger simple para modo “standalone”
        import logging
        logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
        return logging.getLogger("SerialClient")

if __name__ == "__main__":

    logger = _get_default_logger()
    hoverRobotComms = HoverRobotComms(logger= logger, transport= Transport.SERIAL, reconnectDelay= RECONNECT_DELAY)

    hoverRobotComms.connectToRobot(None, None)

    queueReceive = hoverRobotComms.getQueueDynamicData()
    while(True):
        while(not hoverRobotComms.isRobotConnected()):
            time.sleep(1)

        # print('esperando status stabilized')
        # while(hoverRobotComms.getRobotStatus() != RobotStatusCode.STATUS_ROBOT_STABILIZED.value):
        #     time.sleep(1)

        # print('esperando estabilizar todo')
        # time.sleep(5)
        if (hoverRobotComms.isRobotConnected()):

            try:
                data = queueReceive.get()
                print(f"{time.time():.3f}: Recibido {data.statusCode}")
            except queue.Empty:
                print("No hay datos recibidos aún.")


            # hoverRobotComms.sendMockStatus()
            # print('enviado comando ')
            # time.sleep(0.5)

            # hoverRobotComms.sendCommand(CommandsRobotCode.COMMAND_MOVE_FORWARD, 3)

            # hoverRobotComms.sendControl(linearVel=1.2, angularVel=3.4)
            # time.sleep(5)

        else: 
            print('Esperando conexion')
        # time.sleep(3)

    
