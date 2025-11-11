import abc
import queue
from enum import Enum

class Transport(Enum):
    SERIAL = 0
    TCP_IP = 1

class BaseTransport(abc.ABC):       # ABC: Abstract Base Class
    """
    Interfaz base para definir un canal de comunicación con el robot.
    Puede ser implementada por TCP, UART, etc.
    """

    def __init__(self, logger, sendQueue: queue.Queue, recvQueue: queue.Queue):
        self.logger = logger
        self.sendQueue = sendQueue
        self.recvQueue = recvQueue

    @abc.abstractmethod
    def connect(self, *args, **kwargs) -> bool:
        """Establece la conexión (por IP o por puerto serie)."""
        pass

    @abc.abstractmethod
    def disconnect(self, *args, **kwargs):
        """Establece la conexión (por IP o por puerto serie)."""
        pass

    @abc.abstractmethod
    def isConnected(self) -> bool:
        """Retorna True si la conexión sigue activa."""
        pass

    # @abc.abstractmethod
    # def close(self):
    #     """Cierra la conexión limpiamente."""
    #     pass