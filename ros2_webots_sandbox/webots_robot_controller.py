import os
import math
import time

from controller import Robot, Node

import rclpy
from rclpy.node import Node as RclpyNode
from rclpy.time import Time
from sensor_msgs.msg import JointState, Imu, Image, CameraInfo
from geometry_msgs.msg import Pose

from bitbots_msgs.msg import JointCommand

CAMERA_DIVIDER = 8  # every nth timestep an image is published, this is n


class RobotController:
    def __init__(self, ros_node: Node = None, ros_active=False, robot='leo', robot_node=None, base_ns='',
                 recognize=False,
                 camera_active=True, foot_sensors_active=True):
        """
        The RobotController, a Webots controller that controls a single robot.
        The environment variable WEBOTS_ROBOT_NAME should be set to "amy", "rory", "jack" or "donna" if used with
        4_bots.wbt or to "amy" if used with 1_bot.wbt.

        :param ros_active: Whether ROS messages should be published
        :param robot: The name of the robot to use, currently one of wolfgang, darwin, nao, op3
        :param external_controller: Whether an external controller is used, necessary for RobotSupervisorController
        :param base_ns: The namespace of this node, can normally be left empty
        """
        self.ros_node = ros_node
        if self.ros_node is None:
            self.ros_node = RclpyNode('robot_controller')
        self.ros_active = ros_active
        self.recognize = recognize
        self.camera_active = camera_active
        self.use_gps = False
        self.foot_sensors_active = foot_sensors_active
        if robot_node is None:
            self.robot_node = Robot()
        else:
            self.robot_node = robot_node
        self.walkready = [0] * 20
        self.time = 0

        self.motors = []
        self.sensors = []
        # for direct access
        self.motors_dict = {}
        self.sensors_dict = {}
        self.timestep = int(self.robot_node.getBasicTimeStep())

        self.is_wolfgang = False
        if robot == 'wolfgang':
            self.is_wolfgang = True
            # how the names of the joint devices are in the proto file
            self.proto_motor_names = ["RShoulderPitch [shoulder]", "LShoulderPitch [shoulder]", "RShoulderRoll",
                                      "LShoulderRoll", "RElbow", "LElbow", "RHipYaw", "LHipYaw", "RHipRoll [hip]",
                                      "LHipRoll [hip]", "RHipPitch", "LHipPitch", "RKnee", "LKnee", "RAnklePitch",
                                      "LAnklePitch", "RAnkleRoll", "LAnkleRoll", "HeadPan", "HeadTilt"]
            # how the corresponding names of the joints are in the URDF and moveit config                                      
            self.external_motor_names = ["RShoulderPitch", "LShoulderPitch", "RShoulderRoll", "LShoulderRoll", "RElbow",
                                         "LElbow", "RHipYaw", "LHipYaw", "RHipRoll", "LHipRoll", "RHipPitch",
                                         "LHipPitch", "RKnee", "LKnee", "RAnklePitch", "LAnklePitch", "RAnkleRoll",
                                         "LAnkleRoll", "HeadPan", "HeadTilt"]
            self.initial_joint_positions = {"LAnklePitch": -30, "LAnkleRoll": 0, "LHipPitch": 30, "LHipRoll": 0,
                                            "LHipYaw": 0, "LKnee": 60, "RAnklePitch": 30, "RAnkleRoll": 0,
                                            "RHipPitch": -30, "RHipRoll": 0, "RHipYaw": 0, "RKnee": -60,
                                            "LShoulderPitch": 75, "LShoulderRoll": 0, "LElbow": 36,
                                            "RShoulderPitch": -75, "RShoulderRoll": 0, "RElbow": -36, "HeadPan": 0,
                                            "HeadTilt": 0}
            self.sensor_suffix = "_sensor"
            accel_name = "imu accelerometer"
            gyro_name = "imu gyro"
            gps_name = ""
            compass_name = ""
            camera_name = "camera"

        elif robot in ['darwin', 'robotis_op2']:
            self.proto_motor_names = ["ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR", "ArmLowerL",
                                      "PelvYR", "PelvYL", "PelvR", "PelvL", "LegUpperR", "LegUpperL", "LegLowerR",
                                      "LegLowerL", "AnkleR", "AnkleL", "FootR", "FootL", "Neck", "Head"]
            self.external_motor_names = ["RShoulderPitch", "LShoulderPitch", "RShoulderRoll", "LShoulderRoll", "RElbow",
                                         "LElbow", "RHipYaw", "LHipYaw", "RHipRoll", "LHipRoll", "RHipPitch",
                                         "LHipPitch", "RKnee", "LKnee", "RAnklePitch", "LAnklePitch", "RAnkleRoll",
                                         "LAnkleRoll", "HeadPan", "HeadTilt"]
            self.sensor_suffix = "S"
            accel_name = "Accelerometer"
            gyro_name = "Gyro"
            gps_name = ""
            compass_name = ""
            camera_name = "Camera"
        elif robot == 'nao':
            self.proto_motor_names = ["RShoulderPitch", "LShoulderPitch", "RShoulderRoll", "LShoulderRoll", "RElbowYaw",
                                      "LElbowYaw", "RHipYawPitch", "LHipYawPitch", "RHipRoll", "LHipRoll", "RHipPitch",
                                      "LHipPitch",
                                      "RKneePitch", "LKneePitch", "RAnklePitch", "LAnklePitch", "RAnkleRoll",
                                      "LAnkleRoll",
                                      "HeadYaw",
                                      "HeadPitch"]
            self.external_motor_names = self.proto_motor_names
            self.sensor_suffix = "S"
            accel_name = "accelerometer"
            gyro_name = "gyro"
            gps_name = ""
            camera_name = "CameraTop"
        elif robot == 'op3': #robotis
            self.proto_motor_names = ["ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR", "ArmLowerL",
                                      "PelvYR", "PelvYL", "PelvR", "PelvL", "LegUpperR", "LegUpperL", "LegLowerR",
                                      "LegLowerL", "AnkleR", "AnkleL", "FootR", "FootL", "Neck", "Head"]
            self.external_motor_names = ["r_sho_pitch", "l_sho_pitch", "r_sho_roll", "l_sho_roll",
                                         "r_el", "l_el", "r_hip_yaw", "l_hip_yaw", "r_hip_roll", "l_hip_roll",
                                         "r_hip_pitch", "l_hip_pitch", "r_knee", "l_knee", "r_ank_pitch",
                                         "l_ank_pitch", "r_ank_roll", "l_ank_roll", "head_pan", "head_tilt"]
            self.sensor_suffix = "S"
            accel_name = "Accelerometer"
            gyro_name = "Gyro"
            gps_name = ""
            compass_name = ""
            camera_name = "Camera"
        elif robot == 'rfc':
            self.proto_motor_names = ["RightShoulderPitch [shoulder]", "LeftShoulderPitch [shoulder]",
                                      "RightShoulderRoll", "LeftShoulderRoll", "RightElbow", "LeftElbow", "RightHipYaw",
                                      "LeftHipYaw", "RightHipRoll [hip]", "LeftHipRoll [hip]", "RightHipPitch",
                                      "LeftHipPitch", "RightKnee", "LeftKnee", "RightFootPitch", "LeftFootPitch",
                                      "RightFootRoll", "LeftFootRoll", "HeadYaw", "HeadPitch"]
            self.external_motor_names = ["RightShoulderPitch", "LeftShoulderPitch",
                                      "RightShoulderRoll", "LeftShoulderRoll", "RightElbow", "LeftElbow", "RightHipYaw",
                                      "LeftHipYaw", "RightHipRoll", "LeftHipRoll", "RightHipPitch",
                                      "LeftHipPitch", "RightKnee", "LeftKnee", "RightFootPitch", "LeftFootPitch",
                                      "RightFootRoll", "LeftFootRoll", "HeadYaw", "HeadPitch"]
            self.sensor_suffix = "_sensor"
            accel_name = "Accelerometer"
            gyro_name = "Gyroscope"
            gps_name = ""
            compass_name = ""
            camera_name = "Camera"
        elif robot == 'gankenkun': #CITBrains
            self.proto_motor_names = ["right_shoulder_pitch_joint [shoulder]", "left_shoulder_pitch_joint [shoulder]",
                                      "right_shoulder_roll_joint", "left_shoulder_roll_joint",
                                      "right_elbow_pitch_joint", "left_elbow_pitch_joint", "right_waist_yaw_joint",
                                      "left_waist_yaw_joint", "right_waist_roll_joint [hip]",
                                      "left_waist_roll_joint [hip]", "right_waist_pitch_joint",
                                      "left_waist_pitch_joint", "right_knee_pitch_joint", "left_knee_pitch_joint",
                                      "right_ankle_pitch_joint", "left_ankle_pitch_joint",
                                      "right_ankle_roll_joint", "left_ankle_roll_joint", "head_yaw_joint"]
            self.external_motor_names = self.proto_motor_names
            self.sensor_suffix = "_sensor"
            accel_name = "accelerometer"
            gyro_name = "gyro"
            gps_name = ""
            camera_name = "camera_sensor"
        elif robot == 'chape': #itandroids
            self.proto_motor_names = ["rightShoulderPitch[shoulder]", "leftShoulderPitch[shoulder]",
                                      "rightShoulderYaw", "leftShoulderYaw", "rightElbowYaw", "leftElbowYaw",
                                      "rightHipYaw", "leftHipYaw", "rightHipRoll[hip]", "leftHipRoll[hip]",
                                      "rightHipPitch", "leftHipPitch", "rightKneePitch", "leftKneePitch",
                                      "rightAnklePitch", "leftAnklePitch", "rightAnkleRoll", "leftAnkleRoll", "neckYaw",
                                      "neckPitch"]
            self.external_motor_names = ["rightShoulderPitch", "leftShoulderPitch",
                                      "rightShoulderYaw", "leftShoulderYaw", "rightElbowYaw", "leftElbowYaw",
                                      "rightHipYaw", "leftHipYaw", "rightHipRoll", "leftHipRoll",
                                      "rightHipPitch", "leftHipPitch", "rightKneePitch", "leftKneePitch",
                                      "rightAnklePitch", "leftAnklePitch", "rightAnkleRoll", "leftAnkleRoll", "neckYaw",
                                      "neckPitch"]
            self.sensor_suffix = "_sensor"
            accel_name = "Accelerometer"
            gyro_name = "Gyro"
            gps_name = ""
            compass_name = ""
            camera_name = "Camera"
        elif robot == 'mrl_hsl':
            self.proto_motor_names = ["Shoulder-R [shoulder]", "UpperArm-R", "LowerArm-R", "Shoulder-L [shoulder]",
                                      "UpperArm-L", "LowerArm-L", "HipYaw-R", "HipRoll-R [hip]", "HipPitch-R",
                                      "KneePitch-R", "AnklePitch-R", "AnkleRoll-R", "HipYaw-L", "HipRoll-L [hip]",
                                      "HipPitch-L", "KneePitch-L", "AnklePitch-L", "AnkleRoll-L", "NeckYaw",
                                      "HeadPitch"]
            self.external_motor_names = ["Shoulder-R", "UpperArm-R", "LowerArm-R", "Shoulder-L",
                                        "UpperArm-L", "LowerArm-L", "HipYaw-R", "HipRoll-R", "HipPitch-R",
                                        "KneePitch-R", "AnklePitch-R", "AnkleRoll-R", "HipYaw-L", "HipRoll-L",
                                        "HipPitch-L", "KneePitch-L", "AnklePitch-L", "AnkleRoll-L", "NeckYaw",
                                        "HeadPitch"]           
            self.sensor_suffix = "S"
            accel_name = "Accelerometer"
            gyro_name = "Gyro"
            gps_name = ""
            compass_name = ""
            camera_name = "Camera"
        elif robot == 'nugus': #NUbots
            self.proto_motor_names = ["neck_yaw", "head_pitch", "left_hip_yaw", "left_hip_roll [hip]",
                                      "left_hip_pitch", "left_knee_pitch", "left_ankle_pitch", "left_ankle_roll",
                                      "right_hip_yaw", "right_hip_roll [hip]", "right_hip_pitch", "right_knee_pitch",
                                      "right_ankle_pitch", "right_ankle_roll", "left_shoulder_pitch [shoulder]",
                                      "left_shoulder_roll", "left_elbow_pitch", "right_shoulder_pitch [shoulder]",
                                      "right_shoulder_roll", "right_elbow_pitch"]
            self.external_motor_names = ["neck_yaw", "head_pitch", "left_hip_yaw", "left_hip_roll",
                                      "left_hip_pitch", "left_knee_pitch", "left_ankle_pitch", "left_ankle_roll",
                                      "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee_pitch",
                                      "right_ankle_pitch", "right_ankle_roll", "left_shoulder_pitch",
                                      "left_shoulder_roll", "left_elbow_pitch", "right_shoulder_pitch",
                                      "right_shoulder_roll", "right_elbow_pitch"]
            self.sensor_suffix = "_sensor"
            accel_name = "accelerometer"
            gyro_name = "gyroscope"
            gps_name = ""
            compass_name = ""
            camera_name = "left_camera"
        elif robot == 'sahrv74': #Starkit
            self.proto_motor_names = ["right_shoulder_pitch [shoulder]", "right_shoulder_roll", "right_elbow",
                                      "left_shoulder_pitch [shoulder]", "left_shoulder_roll", "left_elbow",
                                      "right_hip_yaw", "right_hip_roll", "right_hip_pitch [hip]", "right_knee",
                                      "right_ankle_pitch", "right_ankle_roll", "left_hip_yaw", "left_hip_roll",
                                      "left_hip_pitch [hip]", "left_knee", "left_ankle_pitch", "left_ankle_roll",
                                      "head_yaw", "head_pitch"]
            self.external_motor_names = self.proto_motor_names
            self.sensor_suffix = "_sensor"
            accel_name = "accelerometer"
            gyro_name = "gyro"
            gps_name = ""
            compass_name = ""
            camera_name = "left_camera"
        elif robot == 'bez': #UTRA
            self.proto_motor_names = ["head_motor_0", "head_motor_1", "right_leg_motor_0", "right_leg_motor_1 [hip]",
                                      "right_leg_motor_2", "right_leg_motor_3", "right_leg_motor_4",
                                      "right_leg_motor_5", "left_leg_motor_0", "left_leg_motor_1 [hip]",
                                      "left_leg_motor_2", "left_leg_motor_3", "left_leg_motor_4", "left_leg_motor_5",
                                      "right_arm_motor_0 [shoulder]", "right_arm_motor_1",
                                      "left_arm_motor_0 [shoulder]", "left_arm_motor_1"]
            self.external_motor_names = ["head_motor_0", "head_motor_1", "right_leg_motor_0", "right_leg_motor_1",
                                      "right_leg_motor_2", "right_leg_motor_3", "right_leg_motor_4",
                                      "right_leg_motor_5", "left_leg_motor_0", "left_leg_motor_1",
                                      "left_leg_motor_2", "left_leg_motor_3", "left_leg_motor_4", "left_leg_motor_5",
                                      "right_arm_motor_0", "right_arm_motor_1",
                                      "left_arm_motor_0", "left_arm_motor_1"]
            self.sensor_suffix = "_sensor"
            accel_name = "imu accelerometer"
            gyro_name = "imu gyro"
            gps_name = ""
            compass_name = ""
            camera_name = "camera"
        elif robot == 'leo': #LEO Rover
            self.proto_motor_names = ["rocker_L_joint", "wheel_FL_joint", "wheel_RL_joint", "rocker_R_joint",
                                      "wheel_FR_joint", "wheel_RR_joint"]
            self.external_motor_names = self.proto_motor_names
            self.sensor_suffix = "_sensor"
            accel_name = ""
            gyro_name = ""
            gps_name = "gps"
            compass_name = "compass"
            camera_name = "camera"
        elif robot == 'bittle': #Petoi Bittle
            self.proto_motor_names = ["left-back-shoulder-joint", "left-back-knee-joint", "left-front-shoulder-joint",
                                      "left-front-knee-joint", "right-back-shoulder-joint", "right-back-knee-joint",
                                      "right-front-shoulder-joint", "right-front-knee-joint"]
            self.external_motor_names = self.proto_motor_names
            self.sensor_suffix = "_sensor"
            accel_name = ""
            gyro_name = ""
            gps_name = ""
            compass_name = ""
            camera_name = ""
        elif robot == 'unitree': #Unitree Go2 Edu
            self.proto_motor_names = ["FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
                                      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"]
            self.external_motor_names = self.proto_motor_names
            self.sensor_suffix = "_sensor"
            accel_name = ""
            gyro_name = ""
            gps_name = ""
            compass_name = ""
            camera_name = ""
        elif robot == "exomy": # ExoMy Rover
            self.proto_motor_names = ["MRB_joint", "STR_LR_joint", "DRV_LR_joint", "STR_RR_joint", "DRV_RR_joint", "RFB_joint",
                                      "STR_RF_joint", "DRV_RF_joint", "STR_RM_joint", "DRV_RM_joint", "LFB_joint",
                                      "STR_LF_joint", "DRV_LF_joint", "STR_LM_joint", "DRV_LM_joint", "pupil_left_joint",
                                      "pupil_right_joint"]
            self.external_motor_names = self.proto_motor_names
            self.sensor_suffix = "_sensor"
            accel_name = "p3d inertial"
            gyro_name = "p3d gyro"
            gps_name = ""
            compass_name = ""
            camera_name = ""
        else:
            self.ros_node.get_logger().error("Robot type not supported: %s" % robot)
            exit()

        self.motor_names_to_external_names = {}
        self.external_motor_names_to_motor_names = {}
        for i in range(len(self.proto_motor_names)):
            self.motor_names_to_external_names[self.proto_motor_names[i]] = self.external_motor_names[i]
            self.external_motor_names_to_motor_names[self.external_motor_names[i]] = self.proto_motor_names[i]

        self.current_positions = {}
        self.joint_limits = {}
        for motor_name in self.proto_motor_names:
            motor = self.robot_node.getDevice(motor_name)
            motor.enableTorqueFeedback(self.timestep)
            self.motors.append(motor)
            self.motors_dict[self.motor_names_to_external_names[motor_name]] = motor
            sensor = self.robot_node.getDevice(motor_name + self.sensor_suffix)
            sensor.enable(self.timestep)
            self.sensors.append(sensor)
            self.sensors_dict[self.motor_names_to_external_names[motor_name]] = sensor
            self.current_positions[self.motor_names_to_external_names[motor_name]] = sensor.getValue()
            # min, max and middle position (precomputed since it will be used at each step)
            self.joint_limits[self.motor_names_to_external_names[motor_name]] = (
                motor.getMinPosition(), motor.getMaxPosition(),
                0.5 * (motor.getMinPosition() + motor.getMaxPosition()))
        if(accel_name != ""):
            self.accel = self.robot_node.getDevice(accel_name)
            self.accel.enable(self.timestep)
        if(gyro_name != ""):
            self.gyro = self.robot_node.getDevice(gyro_name)
            self.gyro.enable(self.timestep)
        if(gps_name != "" and compass_name != ""):
            self.gps = self.robot_node.getDevice(gps_name)
            self.gps.enable(self.timestep)
            self.compass = self.robot_node.getDevice(compass_name)
            self.compass.enable(self.timestep)
            self.use_gps = True
        if self.is_wolfgang:
            self.accel_head = self.robot_node.getDevice("imu_head accelerometer")
            self.accel_head.enable(self.timestep)
            self.gyro_head = self.robot_node.getDevice("imu_head gyro")
            self.gyro_head.enable(self.timestep)
        self.camera = self.robot_node.getDevice(camera_name)
        self.camera_counter = 0
        if self.camera_active:
            self.camera.enable(self.timestep * CAMERA_DIVIDER)
        if self.recognize:
            self.camera.recognitionEnable(self.timestep)
            self.last_img_saved = 0.0
            self.img_save_dir = "/tmp/webots/images" + \
                                time.strftime("%Y-%m-%d-%H-%M-%S") + \
                                os.getenv('WEBOTS_ROBOT_NAME')
            if not os.path.exists(self.img_save_dir):
                os.makedirs(self.img_save_dir)

        self.ros_node.declare_parameter("imu_frame", "imu_frame")
        self.imu_frame = self.ros_node.get_parameter("imu_frame").get_parameter_value().string_value
        if self.ros_active:
            self.pub_js = self.ros_node.create_publisher(JointState, base_ns + "joint_states", 1)
            self.pub_cam = self.ros_node.create_publisher(Image, base_ns + "camera/image_proc", 1)
            self.pub_cam_info = self.ros_node.create_publisher(CameraInfo, base_ns + "camera/camera_info", 1)
            self.pub_odom = self.ros_node.create_publisher(Pose, base_ns + "odom", 1)
            self.ros_node.create_subscription(JointCommand, base_ns + "DynamixelController/command", self.command_cb, 1)

        if robot == "op3":
            # start pose
            command = JointCommand()
            command.joint_names = ["r_sho_roll", "l_sho_roll"]
            command.positions = [-math.tau / 8, math.tau / 8]
            self.command_cb(command)

        # needed to run this one time to initialize current position, otherwise velocity will be nan
        self.get_joint_values(self.external_motor_names)

    def mat_from_fov_and_resolution(self, fov, res):
        return 0.5 * res * (math.cos((fov / 2)) / math.sin((fov / 2)))

    def h_fov_to_v_fov(self, h_fov, height, width):
        return 2 * math.atan(math.tan(h_fov * 0.5) * (height / width))

    def step_sim(self):
        self.time += self.timestep / 1000
        self.robot_node.step(self.timestep)

    def step(self):
        self.step_sim()
        if self.ros_active:
            self.publish_ros()

    def publish_ros(self):
        self.publish_joint_states()
        if self.camera_active and self.camera_counter == 0:
            self.publish_camera()
        if self.recognize:
            self.save_recognition()
        if self.use_gps:
            self.publish_gps_odom()
        self.camera_counter = (self.camera_counter + 1) % CAMERA_DIVIDER

    def convert_joint_radiant_to_scaled(self, joint_name, pos):
        # helper method to convert to scaled position between [-1,1] for this joint using min max scaling
        lower_limit, upper_limit, mid_position = self.joint_limits[joint_name]
        return 2.0 * (pos - mid_position) / (upper_limit - lower_limit)

    def convert_joint_scaled_to_radiant(self, joint_name, position):
        # helper method to convert to scaled position for this joint using min max scaling
        lower_limit, upper_limit, mid_position = self.joint_limits[joint_name]
        return position * (upper_limit - lower_limit) / 2 + mid_position

    def set_joint_goal_position(self, joint_name, goal_position, goal_velocity=-1, scaled=False, relative=False):
        motor = self.motors_dict[joint_name]
        if motor is None:
            print(f"Joint {motor} not found. Can not set goal position.")
            return
        if scaled:
            goal_position = self.convert_joint_scaled_to_radiant(joint_name, goal_position)
        if relative:
            goal_position = goal_position + self.get_joint_values([joint_name])[0]
        motor.setPosition(goal_position)
        if goal_velocity == -1:
            motor.setVelocity(motor.getMaxVelocity())
        else:
            motor.setVelocity(goal_velocity)

    def set_joint_goals_position(self, joint_names, goal_positions, goal_velocities=[]):
        for i in range(len(joint_names)):
            try:
                if len(goal_velocities) != 0:
                    self.set_joint_goal_position(joint_names[i], goal_positions[i], goal_velocities[i])
                else:
                    self.set_joint_goal_position(joint_names[i], goal_positions[i])
            except ValueError:
                print(f"invalid motor specified ({joint_names[i]})")

    def command_cb(self, command: JointCommand):
        if len(command.positions) != 0:
            # position control
            # todo maybe needs to match external motor names to interal ones fist?
            self.set_joint_goals_position(command.joint_names, command.positions, command.velocities)
        else:
            # torque control
            for i, name in enumerate(command.joint_names):
                try:
                    self.motors_dict[name].setTorque(command.accelerations[i])
                except ValueError:
                    print(f"invalid motor specified ({name})")

    def set_head_tilt(self, pos):
        self.motors[-1].setPosition(pos)

    def set_arms_zero(self):
        positions = [-0.8399999308200574, 0.7200000596634105, -0.3299999109923385, 0.35999992683575216,
                     0.5099999812500172, -0.5199999789619728]
        for i in range(0, 6):
            self.motors[i].setPosition(positions[i])

    def get_joint_values(self, used_joint_names, scaled=False):
        joint_positions = []
        joint_velocities = []
        joint_torques = []
        for joint_name in used_joint_names:
            value = self.sensors_dict[joint_name].getValue()
            if scaled:
                value = self.convert_joint_radiant_to_scaled(joint_name, value)
            joint_positions.append(value)
            joint_velocities.append(self.current_positions[joint_name] - value)
            joint_torques.append(self.motors_dict[joint_name].getTorqueFeedback())
            self.current_positions[joint_name] = value
        return joint_positions, joint_velocities, joint_torques

    def get_joint_state_msg(self):
        js = JointState()
        js.name = []
        js.header.stamp = Time(seconds=int(self.time), nanoseconds=self.time % 1 * 1e9).to_msg()
        js.position = []
        js.effort = []
        for joint_name in self.external_motor_names:
            js.name.append(joint_name)
            value = self.sensors_dict[joint_name].getValue()
            js.position.append(value)
            js.velocity.append(self.current_positions[joint_name] - value)
            js.effort.append(self.motors_dict[joint_name].getTorqueFeedback())
            self.current_positions[joint_name] = value
        return js

    def publish_joint_states(self):
        self.pub_js.publish(self.get_joint_state_msg())

    def get_imu_msg(self, head=False):
        msg = Imu()
        msg.header.stamp = Time(seconds=int(self.time), nanoseconds=self.time % 1 * 1e9).to_msg()
        if head:
            msg.header.frame_id = self.head_imu_frame
        else:
            msg.header.frame_id = self.imu_frame

        # change order because webots has different axis
        if head:
            accel_vels = self.accel_head.getValues()
            msg.linear_acceleration.x = accel_vels[2]
            msg.linear_acceleration.y = -accel_vels[0]
            msg.linear_acceleration.z = -accel_vels[1]
        else:
            accel_vels = self.accel.getValues()
            msg.linear_acceleration.x = accel_vels[0]
            msg.linear_acceleration.y = accel_vels[1]
            msg.linear_acceleration.z = accel_vels[2]

        # make sure that acceleration is not completely zero or we will get error in filter.
        # Happens if robot is moved manually in the simulation
        if msg.linear_acceleration.x == 0 and msg.linear_acceleration.y == 0 and msg.linear_acceleration.z == 0:
            msg.linear_acceleration.z = 0.001

        if head:
            gyro_vels = self.gyro_head.getValues()
            msg.angular_velocity.x = gyro_vels[2]
            msg.angular_velocity.y = -gyro_vels[0]
            msg.angular_velocity.z = -gyro_vels[1]
        else:
            gyro_vels = self.gyro.getValues()
            msg.angular_velocity.x = gyro_vels[0]
            msg.angular_velocity.y = gyro_vels[1]
            if not math.isnan(gyro_vels[2]): # nao robot reports nan for yaw
                msg.angular_velocity.z = gyro_vels[2]
            else:
                msg.angular_velocity.z = 0.0
        return msg

    def publish_camera(self):
        img_msg = Image()
        img_msg.header.stamp = Time(seconds=int(self.time), nanoseconds=self.time % 1 * 1e9).to_msg()
        img_msg.header.frame_id = self.camera_optical_frame
        img_msg.height = self.camera.getHeight()
        img_msg.width = self.camera.getWidth()
        img_msg.encoding = "bgra8"
        img_msg.step = 4 * self.camera.getWidth()
        img = self.camera.getImage()
        img_msg.data = img
        self.pub_cam.publish(img_msg)

        self.cam_info = CameraInfo()
        self.cam_info.header = img_msg.header
        self.cam_info.height = self.camera.getHeight()
        self.cam_info.width = self.camera.getWidth()
        f_y = self.mat_from_fov_and_resolution(
            self.h_fov_to_v_fov(self.camera.getFov(), self.cam_info.height, self.cam_info.width),
            self.cam_info.height)
        f_x = self.mat_from_fov_and_resolution(self.camera.getFov(), self.cam_info.width)
        self.cam_info.k = [f_x, 0.0, self.cam_info.width / 2,
                           0.0, f_y, self.cam_info.height / 2,
                           0.0, 0.0, 1.0]
        self.cam_info.p = [f_x, 0.0, self.cam_info.width / 2, 0.0,
                           0.0, f_y, self.cam_info.height / 2, 0.0,
                           0.0, 0.0, 1.0, 0.0]
        self.pub_cam_info.publish(self.cam_info)

    def publish_gps_odom(self):
        position = self.gps.getValues()
        orientation = self.compass.getValues()

        odom = Pose()
        odom.position.x = position[0]
        odom.position.y = position[1]
        odom.position.z = position[2]

        roll = math.atan2(orientation[0], orientation[2])
        pitch = math.asin(-orientation[1])
        yaw =  math.atan2(orientation[1], orientation[2])

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        odom.orientation.w = cr * cp * cy + sr * sp * sy
        odom.orientation.x = sr * cp * cy - cr * sp * sy
        odom.orientation.y = cr * sp * cy + sr * cp * sy
        odom.orientation.z = cr * cp * sy - sr * sp * cy

        self.pub_odom.publish(odom)


    def save_recognition(self):
        if self.time - self.last_img_saved < 1.0:
            return
        self.last_img_saved = self.time
        annotation = ""
        img_stamp = f"{self.time:.2f}".replace(".", "_")
        img_name = f"img_{os.getenv('WEBOTS_ROBOT_NAME')}_{img_stamp}.PNG"
        recognized_objects = self.camera.getRecognitionObjects()
        # variables for saving not in image later
        found_ball = False
        found_wolfgang = False
        for e in range(self.camera.getRecognitionNumberOfObjects()):
            model = recognized_objects[e].get_model()
            position = recognized_objects[e].get_position_on_image()
            size = recognized_objects[e].get_size_on_image()
            if model == b"soccer ball":
                found_ball = True
                vector = f"""{{"x1": {position[0] - 0.5 * size[0]}, "y1": {position[1] - 0.5 * size[1]}, "x2": {position[0] + 0.5 * size[0]}, "y2": {position[1] + 0.5 * size[1]}}}"""
                annotation += f"{img_name}|"
                annotation += "ball|"
                annotation += vector
                annotation += "\n"
            if model == b"wolfgang":
                found_wolfgang = True
                vector = f"""{{"x1": {position[0] - 0.5 * size[0]}, "y1": {position[1] - 0.5 * size[1]}, "x2": {position[0] + 0.5 * size[0]}, "y2": {position[1] + 0.5 * size[1]}}}"""
                annotation += f"{img_name}|"
                annotation += "robot|"
                annotation += vector
                annotation += "\n"
        if not found_ball:
            annotation += f"{img_name}|ball|not in image\n"
        if not found_wolfgang:
            annotation += f"{img_name}|robot|not in image\n"
        with open(os.path.join(self.img_save_dir, "annotations.txt"), "a") as f:
            f.write(annotation)
        self.camera.saveImage(filename=os.path.join(self.img_save_dir, img_name), quality=100)

    def get_image(self):
        return self.camera.getImage()