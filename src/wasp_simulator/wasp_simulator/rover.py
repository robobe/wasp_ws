from wasp_simulator.matrix_utils import hat, vee, q_to_R
from wasp_simulator.control import Control
from wasp_simulator.estimator import Estimator
from wasp_simulator.trajectory import Trajectory

import datetime
import numpy as np
import pdb
import rclpy
import threading

from geometry_msgs.msg import Pose, Twist, Wrench
from geometry_msgs.msg import Vector3, Point, Quaternion
from gazebo_msgs.msg import EntityState 
from gazebo_msgs.srv import SetEntityState

class Rover:
    def __init__(self):

        self.on = True
        self.motor_on = False
        self.save_on = False
        self.mode = 0

        self.t0 = datetime.datetime.now()
        self.t = 0.0
        self.t_pre = 0.0
        self.freq_imu = 0.0
        self.freq_gps = 0.0
        self.freq_control = 0.0
        self.freq_log = 0.0

        self.x = np.zeros(3)
        self.v = np.zeros(3)
        self.a = np.zeros(3)
        self.R = np.identity(3)
        self.W = np.zeros(3)

        self.x_offset = np.zeros(3)
        self.yaw_offset = 0.0

        self.g = 9.81
        self.ge3 = np.array([0.0, 0.0, self.g])

        # Gazebo uses ENU frame, but NED frame is used in FDCL.
        # Note that ENU and the NED here refer to their direction order.
        # ENU: E - axis 1, N - axis 2, U - axis 3
        # NED: N - axis 1, E - axis 2, D - axis 3
        self.R_fg = np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0]
        ])

        self.V_R_imu = np.diag([0.01, 0.01, 0.01])
        self.V_x_gps = np.diag([0.01, 0.01, 0.01])
        self.V_v_gps = np.diag([0.01, 0.01, 0.01])

        self.control = Control()
        self.control.use_integral = True  # Enable integral control

        self.estimator = Estimator()
        self.trajectory = Trajectory()

        self.lock = threading.Lock()

    
    def update_current_time(self):
        t_now = datetime.datetime.now()
        self.t = (t_now - self.t0).total_seconds()


    def get_current_time(self):
        t_now = datetime.datetime.now()
        return (t_now - self.t0).total_seconds()
   
    
    def run_controller(self):
        self.update_current_time()

        with self.lock:
            states = self.estimator.get_states()
            desired = self.trajectory.get_desired(self.mode, states, \
                self.x_offset, self.yaw_offset)
            fM = self.control.run(states, desired)

            self.x, self.v, self.a, self.R, self.W = states
        return fM


    

    

def sent_entity_state_callback(future):
    try:
        response = future.result()
        print(response)
    except Exception as e:
        print("Service call failed: %r" % (e,))

def reset_uav():
    node = rclpy.create_node("reset_uav")
    client = node.create_client(SetEntityState, "/demo/set_entity_state")
    while not client.wait_for_service(1.0):
        node.get_logger().warning("Waiting for service...")
    
    request = SetEntityState.Request()
    request.state = EntityState()
    request.state.name = "uav"
    request.state.reference_frame = "world"
    init_position = Point(x=0.0, y=0.0, z=0.2)
    init_attitude = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    init_pose = Pose(position=init_position, orientation=init_attitude)

    zero_motion = Vector3(x=0.0, y=0.0, z=0.0)
    init_velocity = Twist(linear=zero_motion, angular=zero_motion)
    request.state.pose = init_pose
    request.state.twist = init_velocity
    
    
    future = client.call_async(request)
    future.add_done_callback(sent_entity_state_callback)


