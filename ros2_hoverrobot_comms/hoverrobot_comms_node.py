import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import State
import threading
from std_msgs.msg import String
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from tf2_ros import TransformBroadcaster
import tf_transformations

import queue
import time
import math

from ros2_hoverrobot_comms.hoverrobot_comms import HoverRobotComms
from ros2_hoverrobot_comms.base_transport import Transport

SERVER_IP = '192.168.0.101'
SERVER_PORT = 8080
RECONNECT_DELAY = 5

class HoverRobotCommsNode(LifecycleNode):
    def __init__(self):
        super().__init__('hoverrobot_comms_node')
        
        self.logger = self.get_logger()
        
        # Variables de estado
        self.latest_linear = 0.0
        self.latest_angular = 0.0
        self.last_was_zero = False
        self.dt = 0.025
        
        # Inicializar comunicación (pero NO conectar aún)
        self.hoverRobotComms = HoverRobotComms(
            logger=self.logger, 
            transport= Transport.SERIAL,        # TODO: hacer configurable via param
            reconnectDelay=RECONNECT_DELAY
        )
        self.queueDynamicData = self.hoverRobotComms.getQueueDynamicData()
        
        # Variables de odometría
        self.distAnt = 0
        self.x = 0
        self.y = 0
        
        # Publishers y timers se crean en configure/activate
        self.timer = None
        self.thread = None
        self.connection_check_timer = None
        
        self.logger.info('Nodo creado (unconfigured)')

    # ============= LIFECYCLE CALLBACKS =============
    
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Estado: unconfigured → inactive
        Aquí creamos publishers, subscriptions, pero NO iniciamos operaciones
        """
        self.logger.info('Configurando nodo...')
        
        try:
            # Crear publishers (pero no activados aún)
            self.dynamic_data_publishers = self.create_lifecycle_publisher(
                String, 'hoverrobot_dynamic_data', 10
            )
            self.odom_publisher = self.create_lifecycle_publisher(
                Odometry, 'odom', 10
            )
            self.range_rear_left_publisher = self.create_lifecycle_publisher(
                Range, 'collision_sensor_RL', 10
            )
            self.range_rear_right_publisher = self.create_lifecycle_publisher(
                Range, 'collision_sensor_RR', 10
            )
            self.range_front_left_publisher = self.create_lifecycle_publisher(
                Range, 'collision_sensor_FL', 10
            )
            self.range_front_right_publisher = self.create_lifecycle_publisher(
                Range, 'collision_sensor_FR', 10
            )
            
            self.tf_broadcaster = TransformBroadcaster(self)
            
            # Subscription (siempre activo)
            self.subscription = self.create_subscription(
                Twist, '/cmd_vel', self.__cmdVelCallback, 10
            )
            
            self.logger.info('✓ Nodo configurado')
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.logger.error(f'Error en configuración: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Estado: inactive → active
        Aquí intentamos conectar al robot y activar publicaciones
        """
        self.logger.info('Activando nodo... intentando conectar al robot')
        
        # Limpio el timer de reconexion
        if hasattr(self, 'reconnect_timer') and self.reconnect_timer:
            self.reconnect_timer.cancel()
            self.reconnect_timer = None

        # Intentar conectar (con timeout)
        max_attempts = 3
        for attempt in range(max_attempts):
            if self.hoverRobotComms.isRobotConnected():
                self.logger.info(f'✓ Robot conectado (intento {attempt + 1})')
                break
            
            self.hoverRobotComms.connectToRobot(serverIp=SERVER_IP, serverPort=SERVER_PORT)
            self.logger.warn(f'Esperando conexión... ({attempt + 1}/{max_attempts})')
            time.sleep(1)
        
        if not self.hoverRobotComms.isRobotConnected():
            self.logger.error('✗ No se pudo conectar al robot')
            # return TransitionCallbackReturn.FAILURE
        
        # Activar publishers
        self.dynamic_data_publishers.on_activate(state)
        self.odom_publisher.on_activate(state)
        self.range_rear_left_publisher.on_activate(state)
        self.range_rear_right_publisher.on_activate(state)
        self.range_front_left_publisher.on_activate(state)
        self.range_front_right_publisher.on_activate(state)
        
        # Iniciar thread de publicación
        self.thread = threading.Thread(target=self.__publisherTask, daemon=True)
        self.thread.start()
        
        # Iniciar timer de comandos
        self.timer = self.create_timer(self.dt, self.timer_send_commands_callback)
        
        # Timer para monitorear conexión cada 2 segundos
        self.connection_check_timer = self.create_timer(
            2.0, self.__check_connection_callback
        )
        
        self.logger.info('✓ Nodo activo y robot conectado')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Estado: active → inactive
        Pausamos operaciones pero mantenemos la configuración
        """
        self.logger.warn('Desactivando nodo...')
        
        # Detener timers
        if self.timer:
            self.timer.cancel()
            self.timer = None
        if self.connection_check_timer:
            self.connection_check_timer.cancel()
            self.connection_check_timer = None
        
        # Desactivar publishers
        self.dynamic_data_publishers.on_deactivate(state)
        self.odom_publisher.on_deactivate(state)
        self.range_rear_left_publisher.on_deactivate(state)
        self.range_rear_right_publisher.on_deactivate(state)
        self.range_front_left_publisher.on_deactivate(state)
        self.range_front_right_publisher.on_deactivate(state)
        
        # Enviar velocidad cero antes de desactivar
        self.hoverRobotComms.sendControl(linearVel=0.0, angularVel=0.0)
        
        self.logger.info('✓ Nodo desactivado')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Estado: inactive → unconfigured
        Limpiamos recursos completamente
        """
        self.logger.info('Limpiando nodo...')
        
        # Destruir publishers
        self.destroy_publisher(self.dynamic_data_publishers)
        self.destroy_publisher(self.odom_publisher)
        self.destroy_publisher(self.range_rear_left_publisher)
        self.destroy_publisher(self.range_rear_right_publisher)
        self.destroy_publisher(self.range_front_left_publisher)
        self.destroy_publisher(self.range_front_right_publisher)
        
        self.logger.info('✓ Nodo limpiado')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Shutdown desde cualquier estado
        """
        self.logger.info('Apagando nodo...')
        
        # Detener robot
        if self.hoverRobotComms:
            self.hoverRobotComms.sendControl(linearVel=0.0, angularVel=0.0)
        
        return TransitionCallbackReturn.SUCCESS

    # ============= MONITOREO DE CONEXIÓN =============
    
    def __check_connection_callback(self):
        """
        Verifica periódicamente si el robot sigue conectado
        Si se desconecta, auto-transiciona a inactive
        """
        if not self.hoverRobotComms.isRobotConnected():
            self.logger.error('Robot desconectado')
            
            # Auto-transición a inactive
            self.trigger_deactivate()
            
            # Inicio timer de reconexión
            self.reconnect_timer = self.create_timer(5.0, self.__try_reconnect)

    def __try_reconnect(self):
        """
        Intenta reconectar y volver a activar el nodo
        """
        self.logger.info('Intentando reconectar...')
        
        if hasattr(self, 'reconnect_timer'):
            self.reconnect_timer.cancel()
            self.reconnect_timer = None

        self.trigger_activate()

    
    def __publisherTask(self):
        while True:

            try:
                robotDynamicData = self.queueDynamicData.get(timeout=1)
            except queue.Empty:
                continue

            # ... tu lógica de publicación original ...
            posInMeters = robotDynamicData.posInMeters / 100.00
            delta_dist = posInMeters - self.distAnt
            self.distAnt = posInMeters

            imuRoll = robotDynamicData.roll / 100.00
            imuPitch = robotDynamicData.pitch / 100.00
            imuYaw = robotDynamicData.yaw / -100.00

            collisionDistanceFrontLeftInMeters = robotDynamicData.collisionFL / 10000.00
            collisionDistanceFrontRightInMeters = robotDynamicData.collisionFR / 10000.00
            collisionDistanceRearLeftInMeters = robotDynamicData.collisionRL / 10000.00
            collisionDistanceRearRightInMeters = robotDynamicData.collisionRR / 10000.00

            self.yaw = math.radians(imuYaw)
            self.x += delta_dist * math.cos(self.yaw)
            self.y += delta_dist * math.sin(self.yaw)

            pitchInRads = 0.0
            quat = tf_transformations.quaternion_from_euler(0.0, pitchInRads, self.yaw)

            now = self.get_clock().now().to_msg()
            
            # TF
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.4
            t.transform.rotation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            self.tf_broadcaster.sendTransform(t)

            # Odometry
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation = t.transform.rotation
            self.odom_publisher.publish(odom)

            # Range sensors
            range_msg = Range()
            range_msg.header.stamp = now
            range_msg.min_range = 0.03
            range_msg.max_range = 1.00
            range_msg.radiation_type = 0
            range_msg.field_of_view = 0.26179

            range_msg.header.frame_id = 'range_front_left'
            range_msg.range = collisionDistanceFrontLeftInMeters
            self.range_front_left_publisher.publish(range_msg)

            range_msg.header.frame_id = 'range_front_right'
            range_msg.range = collisionDistanceFrontRightInMeters
            self.range_front_right_publisher.publish(range_msg)

            range_msg.header.frame_id = 'range_rear_left'
            range_msg.range = collisionDistanceRearLeftInMeters
            self.range_rear_left_publisher.publish(range_msg)

            range_msg.header.frame_id = 'range_rear_right'
            range_msg.range = collisionDistanceRearRightInMeters
            self.range_rear_right_publisher.publish(range_msg)

    def __cmdVelCallback(self, msg):
        self.latest_linear = msg.linear.x
        self.latest_angular = msg.angular.z

    def timer_send_commands_callback(self):
        if self.hoverRobotComms.isRobotConnected():
            self.latest_linear = max(-1.5, min(1.5, self.latest_linear))
            is_zero = (self.latest_linear == 0.0 and self.latest_angular == 0.0)

            if (not is_zero) or (is_zero and not self.last_was_zero):
                if not self.hoverRobotComms.sendControl(
                    linearVel=self.latest_linear,
                    angularVel=self.latest_angular * -1.0
                ):
                    self.logger.info(f'ERROR: Comando no enviado, statusRobot: {self.hoverRobotComms.getRobotStatus()}')

                self.last_was_zero = is_zero

        # else:
        #     self.logger.error('Timer de envio activado PERO robot desconectado!')


def main(args=None):
    rclpy.init(args=args)
    
    executor = rclpy.executors.SingleThreadedExecutor()
    node = HoverRobotCommsNode()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()