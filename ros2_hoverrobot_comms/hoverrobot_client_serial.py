import threading
import queue
import serial.tools.list_ports

from ros2_hoverrobot_comms.base_transport import BaseTransport

class SerialClient(BaseTransport):
    def __init__(self, logger, sendQueue, recvQueue):
        super().__init__(logger, sendQueue, recvQueue)

        self.connected_event = threading.Event()  # Para saber si está conectado
        self.SerialPort = None

    def isConnected(self):
        super().isConnected()
        return self.connected_event.is_set()

    def __receive_loop(self, recv_queue):
        try:
            # while True:
                # recv_queue.put(data)
            self.logger.info('vvoid')       # TODO: recibir data del serial
        except Exception as e:
            self.logger.error("Error en receive_loop:", e)

    def __send_loop(self, send_queue):
        try:
            while True:
                try:
                    message = send_queue.get(timeout=1)
                    # sock.sendall(message)         # TODO: enviar data del serial
                except queue.Empty:
                    continue
        except Exception as e:
            self.logger.error("Error en receive_loop:", e)

    def connect(self, *args, **kwargs):
        super().connect(*args, **kwargs)

        self.logger.info('SERIALCLIENT CONNECTADITO')

        recv_thread = threading.Thread(target=self.__receive_loop, args=(self.recvQueue), daemon=True)
        send_thread = threading.Thread(target=self.__send_loop, args=(self.sendQueue), daemon=True)

        if self.recvQueue is not None:
            recv_thread.start()
        else: 
            self.logger.error('Cola de recepcion no recibida, recepcion deshabilitada')
        
        if self.sendQueue is not None:
            send_thread.start()
        else: 
            self.logger.error('Cola de envio no recibida, envio deshabilitado')

        puertos = serial.tools.list_ports.comports()

        for p in puertos:
            descripcion = (p.description or "").lower()  # convierte a lowercase y evita None
            if "usb-serial" in descripcion:
                self.SerialPort = p.device  # ejemplo: /dev/ttyUSB0
        
        if (self.SerialPort is not None):
            self.connected_event.set()
            self.logger.info(f"✅ Puerto serie detectado: {self.SerialPort}")
        else: 
            self.logger.error('ERROR PUERTO NO ENCONTRADO')
            return False

        return True

    def disconnect(self, *args, **kwargs):
        super().disconnect(*args, **kwargs)
        self.logger.info('Client serial disconnect!')