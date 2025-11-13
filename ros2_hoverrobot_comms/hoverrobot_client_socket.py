import socket
import threading
import time
import queue

# from ros2_hoverrobot_comms.base_transport import BaseTransport
from base_transport import BaseTransport

# SERVER_IP = '192.168.1.35'
SERVER_IP = '192.168.0.101'
SERVER_PORT = 8080
RECONNECT_DELAY = 5  # segundos entre reintentos

class SocketClient(BaseTransport):

    def __init__(self, logger, reconnectDelay, sendQueue=None, recvQueue=None):
        super().__init__(logger, sendQueue, recvQueue)
        self.reconnectDelay = reconnectDelay

        self.stop_event = threading.Event()
        self.connected_event = threading.Event()  # Para saber si está conectado
        self.connectedSocket = None

    def isConnected(self):
        return self.connected_event.is_set()

    def __receive_loop(self, sock, queue):
        try:
            while not self.stop_event.is_set():
                data = sock.recv(1024)
                if not data:
                    self.logger.error("Servidor cerró la conexión.")
                    break
                queue.put(data)
        except Exception as e:
            if not self.stop_event.is_set():            # solo loggea si no fue stop intencional
                self.logger.error("Error en receive_loop:", e)
        finally:
            self.stop_event.set()
            self.connected_event.clear()

    def __send_loop(self, sock, send_queue):
        try:
            while not self.stop_event.is_set():
                try:
                    message = send_queue.get(timeout=1)
                    sock.sendall(message)
                except queue.Empty:
                    continue
        except (BrokenPipeError, ConnectionResetError, OSError) as e:
            if not self.stop_event.is_set():            # solo loggea si no fue stop intencional
                self.logger.error("Error de envío, conexión perdida.")
        finally:
            self.stop_event.set()
            self.connected_event.clear()

    def connect(self, serverIp, serverPort):
        self.stop_event.set()
        time.sleep(0.1)     # Le doy tiempo a que los threads terminen

        self.logger.info('connect en SocketClient()')

        try:
            self.logger.info(f"Intentando conectar a {serverIp}:{serverPort}...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            sock.settimeout(self.reconnectDelay)

            sock.connect((serverIp, serverPort))

            self.connectedSocket = sock
            self.stop_event.clear()
            self.connected_event.set()

            recv_thread = threading.Thread(target=self.__receive_loop, args=(sock, self.recvQueue), daemon=True)
            send_thread = threading.Thread(target=self.__send_loop, args=(sock, self.sendQueue), daemon=True)

            if self.recvQueue is not None:
                recv_thread.start()
            else: 
                self.logger.error('Cola de recepcion no recibida, recepcion deshabilitada')
            
            if self.sendQueue is not None:
                send_thread.start()
            else: 
                self.logger.error('Cola de envio no recibida, envio deshabilitado')

            return True

            # Esperar que cualquiera de los threads termine (por error o desconexión)
            # while not self.stop_event.is_set():
            #     time.sleep(0.1)

        except Exception as e:
            self.logger.error(f"Error de conexión o envío: {e}")
            self.stop_event.set()
            self.connected_event.clear()
            if 'sock' in locals():  # solo cierro si se creó
                try:
                    sock.close()
                except:
                    pass
            return False

    def disconnect(self):
        """Desconecta y limpia recursos"""
        self.stop_event.set()
        self.connected_event.clear()
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None


if __name__ == "__main__":

    queueSender = queue.Queue()
    # queueReceiver = None
    queueReceiver = queue.Queue()
    socketClient = SocketClient(SERVER_IP, SERVER_PORT, RECONNECT_DELAY, sendQueue=queueSender, recvQueue=queueReceiver)

    while True:
        if socketClient.connected_event.is_set():
            if queueSender is not None:
                queueSender.put('Hola pepito\n')
        else:
            print("No conectado, esperando para enviar...")

        # Mostrar datos recibidos si hay
        try:
            if queueReceiver is not None:
                data = queueReceiver.get(timeout=1)
                print('Datos recibidos:', data)
        except queue.Empty:
            pass

        time.sleep(1)
