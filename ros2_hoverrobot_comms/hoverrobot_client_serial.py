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

    def __receive_loop(self):
        try:
            while True:
                if self.connected_event.is_set():

                    data = self.serial.read_all()  # Lee todos los bytes disponibles
                    if data:
                        self.recvQueue.put(data)  # 🔹 En cola de bytes sin decodificar
                        # self.logger.info(f"RX {len(data)} bytes: {data.hex(' ')}")
        except Exception as e:
            self.logger.error("Error en receive_loop:", e)

    def __send_loop(self):
        try:
            while True:
                if self.connected_event.is_set():
                    try:
                        message = self.sendQueue.get(timeout=1)
                        if isinstance(message, bytes):
                            self.serial.write(message)
                            # self.logger.info(f"TX {len(message)} bytes: {message.hex(' ')}")
                        else:
                            self.logger.warn("Intento de enviar datos no-binarios (ignorado)")
                    except queue.Empty:
                        continue
        except Exception as e:
            self.logger.error("Error en receive_loop:", e)

    def connect(self, *args, **kwargs):
        super().connect(*args, **kwargs)

        puertos = serial.tools.list_ports.comports()

        for p in puertos:
            descripcion = (p.description or "").lower()  # convierte a lowercase y evita None
            if "usb-serial" in descripcion:
                self.SerialPort = p.device  # ejemplo: /dev/ttyUSB0
            elif "usb-to-serial" in descripcion:             # TODO: solo para probar en windows
                self.SerialPort = p.device  # ejemplo: prolific usb-to-serial comm port (com9)
            else:
                self.logger.warn(f'Puerto ignorado: {descripcion}, device: {p.device}')
        
        if (self.SerialPort is not None):
            
            self.logger.info(f"✅ Puerto serie detectado: {self.SerialPort}")

            self.serial = serial.Serial(
                port=self.SerialPort,
                baudrate=115200,        # TODO: pasar por parametro
                timeout=0.01
            )

            self.connected_event.set()

            recv_thread = threading.Thread(target=self.__receive_loop, daemon=True)
            send_thread = threading.Thread(target=self.__send_loop, daemon=True)

            if self.recvQueue is not None:
                recv_thread.start()
            else: 
                self.logger.error('Cola de recepcion no recibida, recepcion deshabilitada')
            
            if self.sendQueue is not None:
                send_thread.start()
            else: 
                self.logger.error('Cola de envio no recibida, envio deshabilitado')
        else: 
            self.logger.error('ERROR PUERTO NO ENCONTRADO')
            return False

        return True

    def disconnect(self, *args, **kwargs):
        super().disconnect(*args, **kwargs)
        self.serial.close()
        self.logger.info('Client serial disconnect!')

import time
import logging

if __name__ == "__main__":
    # 1️⃣ Logger básico de consola
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S"
    )
    logger = logging.getLogger("SerialTest")

    # 2️⃣ Crear colas de envío y recepción
    sendQueue = queue.Queue()
    recvQueue = queue.Queue()

    # 3️⃣ Instanciar tu cliente serial
    client = SerialClient(logger, sendQueue, recvQueue)

    # 4️⃣ Intentar conectar (autodetecta el puerto)
    if not client.connect():
        logger.error("No se pudo conectar al puerto serie.")
        exit(1)

    logger.info("Cliente conectado. Enviando datos de prueba...")

    # 5️⃣ Enviar algunos bytes de ejemplo cada 2 segundos
    try:
        contador = 0
        while True:
            msg = bytes([0xAB, 0xCD, contador & 0xFF])  # ejemplo simple
            sendQueue.put(msg)
            logger.info(f"Mensaje encolado: {msg.hex(' ')}")

            # Ver si llegó algo
            try:
                data = recvQueue.get(timeout=1)
                logger.info(f"Recibido: {data.hex(' ')}")
            except queue.Empty:
                logger.debug("No hay datos recibidos aún.")

            contador += 1
            time.sleep(2)

    except KeyboardInterrupt:
        logger.info("Interrumpido por el usuario.")
    finally:
        client.disconnect()