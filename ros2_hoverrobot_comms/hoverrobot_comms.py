import struct
import queue 
import time 
import threading
from ros2_hoverrobot_comms.hoverrobot_client import SocketClient
from ros2_hoverrobot_comms.hoverrobot_types import DYNAMIC_ROBOT_PACKET_SIZE, FORMAT_DYNAMYC_ROBOT, FORMAT_COMMAND_ROBOT, RobotStatusCode, RobotDynamicData, CommandsRobotCode, RobotHeaderPackage

SERVER_IP = '192.168.0.101'
SERVER_PORT = 8080
RECONNECT_DELAY = 5  # segundos entre reintentos

class HoverRobotComms():

    def __init__(self, serverIp, serverPort, reconnectDelay):

        self.queueSender = queue.Queue()
        # self.queueSender = None
        self.queueReceiver = queue.Queue()
        self.socketClient = SocketClient(serverIp, serverPort, reconnectDelay, sendQueue=self.queueSender, recvQueue=self.queueReceiver)

        self.queueDynamicData = queue.Queue()

        # Buffer global para acumular fragmentos de paquetes
        self.receive_buffer = b''

        self.parsedDynamicData = None

        self.thread = threading.Thread(target=self.__processData, daemon=True)
        self.thread.start()

    def isRobotConnected(self): 
        return self.socketClient.isConnected()

    def sendCommand(self, command, value): 
        if (self.queueSender is not None and self.socketClient.isConnected and self.parsedDynamicData.statusCode == RobotStatusCode.STATUS_ROBOT_STABILIZED.value):
            packed = struct.pack(FORMAT_COMMAND_ROBOT, 
                        RobotHeaderPackage.HEADER_PACKAGE_COMMAND.value, 
                        command.value, 
                        int(value))
            self.queueSender.put(packed)
            return True
        else:
            print('socket desconectado, queueSender is None o robot no estabilizado')
            return False
        
    def getRobotStatus(self):
        return getattr(self.parsedDynamicData, "statusCode", RobotStatusCode.STATUS_ROBOT_INIT)
    
    def getQueueDynamicData(self):
        return self.queueDynamicData

    def __processData(self): 
        while True:
            try:
                raw_data = self.queueReceiver.get(timeout=1)
                self.receive_buffer += raw_data

                while True:
                    # Buscar el header (2 bytes en little endian: 0x01 0xAB si es 0xAB01)
                    sync_pos = self.receive_buffer.find(struct.pack('<H', RobotHeaderPackage.HEADER_PACKAGE_STATUS.value))

                    if sync_pos == -1:
                        # No se encontró header, descartamos todo lo anterior
                        self.receive_buffer = b''
                        break

                    # Si hay datos antes del header, los descartamos
                    if sync_pos > 0:
                        self.receive_buffer = self.receive_buffer[sync_pos:]

                    # ¿Tenemos al menos un paquete completo desde el header?
                    if len(self.receive_buffer) < DYNAMIC_ROBOT_PACKET_SIZE:
                        # Esperamos a que lleguen más datos
                        break

                    # Extraemos un paquete alineado
                    packet = self.receive_buffer[:DYNAMIC_ROBOT_PACKET_SIZE]
                    self.receive_buffer = self.receive_buffer[DYNAMIC_ROBOT_PACKET_SIZE:]

                    try:
                        unpacked = struct.unpack(FORMAT_DYNAMYC_ROBOT, packet)
                        self.parsedDynamicData = RobotDynamicData(*unpacked)

                        if (self.queueDynamicData is not None):
                            self.queueDynamicData.put(self.parsedDynamicData)
                        # print(f">> status: {RobotStatusCode(self.parsedDynamicData.statusCode).name}\t>> battery: {self.parsedDynamicData.batVoltage}\t >> pitch: {self.parsedDynamicData.pitch}")

                    except struct.error as e:
                        print(f"Error al parsear paquete: {e}")
                        # Podrías descartar un byte o limpiar el buffer si esto ocurre frecuentemente

            except queue.Empty:
                pass

            time.sleep(0.025)        

if __name__ == "__main__":

    hoverRobotComms = HoverRobotComms(SERVER_IP, SERVER_PORT, RECONNECT_DELAY)

    while(True):
        print('esperando conexion')
        while(not hoverRobotComms.isRobotConnected()):
            time.sleep(1)

        print('esperando status stabilized')
        while(hoverRobotComms.getRobotStatus() != RobotStatusCode.STATUS_ROBOT_STABILIZED.value):
            time.sleep(1)

        print('esperando estabilizar todo')
        time.sleep(5)
        if (hoverRobotComms.isRobotConnected()):
            # hoverRobotComms.sendCommand(CommandsRobotCode.COMMAND_MOVE_FORWARD, 3)
            # hoverRobotComms.sendCommand(CommandsRobotCode.COMMAND_MOVE_REL_YAW, -4500)

            hoverRobotComms.sendCommand(CommandsRobotCode.COMMAND_MOVE_FORWARD, 100)
            time.sleep(2)
            hoverRobotComms.sendCommand(CommandsRobotCode.COMMAND_MOVE_REL_YAW, 9000)
            time.sleep(2)

            hoverRobotComms.sendCommand(CommandsRobotCode.COMMAND_MOVE_FORWARD, 100)
            time.sleep(2)
            hoverRobotComms.sendCommand(CommandsRobotCode.COMMAND_MOVE_REL_YAW, 9000)
            time.sleep(2)

            hoverRobotComms.sendCommand(CommandsRobotCode.COMMAND_MOVE_FORWARD, 100)
            time.sleep(5)

            hoverRobotComms.sendCommand(CommandsRobotCode.COMMAND_MOVE_FORWARD, -100)
            time.sleep(2)
            hoverRobotComms.sendCommand(CommandsRobotCode.COMMAND_MOVE_REL_YAW, -9000)
            time.sleep(2)

            hoverRobotComms.sendCommand(CommandsRobotCode.COMMAND_MOVE_FORWARD, -100)
            time.sleep(2)
            hoverRobotComms.sendCommand(CommandsRobotCode.COMMAND_MOVE_REL_YAW, -9000)
            time.sleep(2)

            hoverRobotComms.sendCommand(CommandsRobotCode.COMMAND_MOVE_FORWARD, -100)

        else: 
            print('Esperando conexion')
        time.sleep(3)

    
