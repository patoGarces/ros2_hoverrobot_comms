import rclpy
from rclpy.node import Node 
import threading
from std_msgs.msg import String # TODO: reemplazar por el tipo de message especifico
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from tf2_ros import TransformBroadcaster
import tf_transformations

import queue

import time
import math

from ros2_hoverrobot_comms.hoverrobot_comms import HoverRobotComms, CommandsRobotCode
from ros2_hoverrobot_comms.hoverrobot_types import RobotStatusCode


SERVER_IP = '192.168.0.101'
SERVER_PORT = 8080
RECONNECT_DELAY = 5  # segundos entre reintentos

class HoverRobotCommsNode(Node):
    def __init__(self):
        super().__init__('hoverrobot_comms_node')

        self.publishers_ = self.create_publisher(String, 'hoverrobot_dynamic_data', 10)
        self.subscription = self.create_subscription(Twist,'/cmd_vel',self.__cmdVelCallback,10)

        # Timer para ejecutar a ritmo fijo (cada 100ms = 10Hz)
        self.dt = 0.025  # segundos
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # Última velocidad recibida
        self.latest_linear = 0.0  # m/s
        self.latest_angular = 0.0  # rad/s


        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.hoverRobotComms = HoverRobotComms(SERVER_IP, SERVER_PORT, RECONNECT_DELAY)
        self.queueDynamicData = self.hoverRobotComms.getQueueDynamicData()

        print('esperando conexion')
        while(not self.hoverRobotComms.isRobotConnected()):
            time.sleep(1)

        print('Robot conectado')

        self.thread = threading.Thread(target=self.__publishOdometers, daemon=True)
        self.thread.start()

    def __publishOdometers(self):

        self.distAnt = 0
        self.x = 0
        self.y = 0
        
        while(True):
            # print('dinamicData: ' + str(self.queueDynamicData.get(timeout=1)))

            # robotDynamicData = self.queueDynamicData.get(timeout=1)
            try:
                robotDynamicData = self.queueDynamicData.get(timeout=1)
            except queue.Empty:
                continue

            posInMeters = robotDynamicData.posInMeters/ 100.00
            delta_dist = posInMeters - self.distAnt
            self.distAnt = posInMeters

            imuRoll = robotDynamicData.roll / 100.00
            imuPitch = robotDynamicData.pitch / 100.00
            imuYaw = robotDynamicData.yaw / -100.00     # invierto sentido del yaw

            self.yaw = math.radians(imuYaw)
            self.x += delta_dist * math.cos(self.yaw)
            self.y += delta_dist * math.sin(self.yaw)

            quat = tf_transformations.quaternion_from_euler(0.0, 0.0, self.yaw)

            now = self.get_clock().now().to_msg()
            # → TF
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.4 # altura del robot
            t.transform.rotation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            self.tf_broadcaster.sendTransform(t)

            # → Odometry
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.orientation = t.transform.rotation
            self.odom_pub.publish(odom)

    def __cmdVelCallback(self, msg):

        self.latest_linear = msg.linear.x
        self.latest_angular = msg.angular.z

        # print(f'nuevo cmd_vel: linear: {msg.linear.x}, \t angular: {msg.angular.z}')

    def timer_callback(self):

        if (self.hoverRobotComms.isRobotConnected):

            if (self.latest_linear > 1.5): self.latest_linear = 1.5
            if (self.latest_linear < -1.5): self.latest_linear = -1.5

            if (self.latest_linear != 0.0 or self.latest_angular != 0.0):      # Solo envio comandos si son distintos de cero, sino no envio nada
                self.hoverRobotComms.sendControl(linearVel= self.latest_linear, angularVel= self.latest_angular*-1.00)

def main(args=None):
    rclpy.init(args=args)
    node = HoverRobotCommsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()