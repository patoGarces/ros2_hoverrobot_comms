import socket
import threading
import time
import queue

# SERVER_IP = '192.168.1.35'
SERVER_IP = '192.168.0.101'
SERVER_PORT = 8080
RECONNECT_DELAY = 5  # segundos entre reintentos

class SocketClient:

    def __init__(self, serverIp, serverPort, reconnectDelay, sendQueue=None, recvQueue=None):
        self.serverIp = serverIp
        self.serverPort = serverPort
        self.reconnectDelay = reconnectDelay

        self.sendQueue = sendQueue
        self.recvQueue = recvQueue

        self.stop_event = threading.Event()
        self.connected_event = threading.Event()  # Para saber si está conectado

        self.thread = threading.Thread(target=self.__socketConnect, daemon=True)
        self.thread.start()

    def isConnected(self):
        return self.connected_event.is_set()

    def __receive_loop(self, sock, queue):
        try:
            while not self.stop_event.is_set():
                data = sock.recv(1024)
                if not data:
                    print("Servidor cerró la conexión.")
                    break
                queue.put(data)  # ⬅️ Mandamos los bytes crudos
        except Exception as e:
            print("Error en receive_loop:", e)
        finally:
            self.stop_event.set()
            self.connected_event.clear()  # Perdió conexión

    def __send_loop(self, sock, send_queue):
        try:
            while not self.stop_event.is_set():
                try:
                    message = send_queue.get(timeout=1)  # Espera hasta que haya algo para enviar
                    sock.sendall(message)
                except queue.Empty:
                    continue
                except (BrokenPipeError, ConnectionResetError, OSError):
                    print("Error de envío, conexión perdida.")
                    break
        finally:
            self.stop_event.set()
            self.connected_event.clear()  # Perdió conexión

    def __socketConnect(self):
        while True:
            try:
                print(f"Intentando conectar a {self.serverIp}:{self.serverPort}...", flush=True)
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

                sock.settimeout(self.reconnectDelay)

                sock.connect((self.serverIp, self.serverPort))

                self.stop_event.clear()
                self.connected_event.set()

                recv_thread = threading.Thread(target=self.__receive_loop, args=(sock, self.recvQueue), daemon=True)
                send_thread = threading.Thread(target=self.__send_loop, args=(sock, self.sendQueue), daemon=True)

                if self.recvQueue is not None:
                    recv_thread.start()
                else: 
                    print('Cola de recepcion no recibida, recepcion deshabilitada')
                
                if self.sendQueue is not None:
                    send_thread.start()
                else: 
                    print('Cola de envio no recibida, envio deshabilitado')

                # Esperar que cualquiera de los threads termine (por error o desconexión)
                while not self.stop_event.is_set():
                    time.sleep(0.1)

            except Exception as e:
                print(f"Error de conexión o envío: {e}", flush=True)
            finally:
                self.stop_event.set()
                self.connected_event.clear()
                try:
                    sock.close()
                except:
                    pass
                print(f"[REINTENTO] Reintentando en {self.reconnectDelay} segundos...\n", flush=True)
                time.sleep(self.reconnectDelay)


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
