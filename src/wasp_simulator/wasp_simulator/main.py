import datetime
import numpy as np
import rclpy
from rclpy import qos
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from wasp_simulator.rover import Rover, reset_uav
from wasp_simulator.matrix_utils import hat, vee, q_to_R
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, Vector3
from wasp_simulator.gui import thread_gui
import threading

WORKER_THREAD = 10


class UAV(Node):
    def __init__(self):
        node_name = "uav"
        super().__init__(node_name)
        g = ReentrantCallbackGroup()
        self.t_pre = datetime.datetime.now()
        self.freq = 0.0
        self.rover = Rover()
        t_gui = threading.Thread(target=thread_gui, args=(self.rover,), daemon=True)
        t_gui.start()
        self.create_subscription(
            Imu,
            "/imu_plugin/out",
            callback=self.ros_imu_callback,
            qos_profile=qos.qos_profile_system_default,
            callback_group=g,
        )

        self.create_subscription(
            Odometry,
            topic="/odom",
            callback=self.ros_gps_callback,
            qos_profile=qos.qos_profile_system_default,
            callback_group=g,
        )

        self.create_timer(1 / 200, callback=self.controller_handler, callback_group=g)

        self.pub_uav_fm = self.create_publisher(
            Wrench,
            topic="/uav_fm",
            qos_profile=qos.qos_profile_system_default,
            callback_group=g,
        )

    def controller_handler(self):
        t = datetime.datetime.now()
        avg_number = 1000
        dt = (t - self.t_pre).total_seconds()
        self.freq = (self.freq * (avg_number - 1) + (1 / dt)) / avg_number
        self.rover.freq_control = self.freq

        fM = self.rover.run_controller()
        # print(fM)
        if (not self.rover.motor_on) or (self.rover.mode < 2):
            fM_message = Wrench(
                force=Vector3(x=0.0, y=0.0, z=0.0), torque=Vector3(x=0.0, y=0.0, z=0.0)
            )
        else:
            fM_message = Wrench(
                force=Vector3(x=0.0, y=0.0, z=fM[0][0]),
                torque=Vector3(x=fM[1][0], y=fM[2][0], z=fM[3][0]),
            )

        self.pub_uav_fm.publish(fM_message)

    def ros_imu_callback(self, message: Imu):
        q_gazebo = message.orientation
        a_gazebo = message.linear_acceleration
        W_gazebo = message.angular_velocity

        q = np.array([q_gazebo.x, q_gazebo.y, q_gazebo.z, q_gazebo.w])

        R_gi = q_to_R(q)  # IMU to Gazebo frame
        R_fi = self.rover.R_fg.dot(R_gi)  # IMU to FDCL frame (NED freme)

        # FDCL-UAV expects IMU accelerations without gravity.
        a_i = np.array([a_gazebo.x, a_gazebo.y, a_gazebo.z])
        a_i = R_gi.T.dot(R_gi.dot(a_i) - self.rover.ge3)

        W_i = np.array([W_gazebo.x, W_gazebo.y, W_gazebo.z])

        with self.rover.lock:
            self.rover.estimator.prediction(a_i, W_i)
            self.rover.estimator.imu_correction(R_fi, self.rover.V_R_imu)

    def ros_gps_callback(self, message: Odometry):
        x_gazebo = message.pose.pose.position
        v_gazebo = message.twist.twist.linear

        # Gazebo uses ENU frame, but NED frame is used in FDCL.
        x_g = np.array([x_gazebo.x, -x_gazebo.y, -x_gazebo.z])
        v_g = np.array([v_gazebo.x, -v_gazebo.y, -v_gazebo.z])

        with self.rover.lock:
            self.rover.estimator.gps_correction(
                x_g, v_g, self.rover.V_x_gps, self.rover.V_v_gps
            )


def main(args=None):
    rclpy.init(args=args)

    node = UAV()
    try:
        reset_uav()
        exec = MultiThreadedExecutor(WORKER_THREAD)
        exec.add_node(node)
        exec.spin()
    except KeyboardInterrupt:
        print("User exit")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
