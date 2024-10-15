import rclpy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header

from nao_lola_command_msgs.msg import JointRequests
from nao_lola_sensor_msgs.msg import Fsr, Imu, JointData, JointIndexes


class NaoWebotsDriver:
    DOF = 25
    sensors = {
        "Stiffness": [1.0] * DOF,
        "Position": [0.0] * DOF,
        "Temperature": [0.0] * DOF,
        "Current": [0.0] * DOF,
        "Battery": [1.0, -32708.0, 0.0, 0.0],
        "Accelerometer": [0.0, 0.0, 0.0],
        "Gyroscope": [0.0, 0.0, 0.0],
        "Angles": [0.0, 0.0],
        "Sonar": [0.0, 0.0],
        "FSR": [0.0] * 8,
        "Status": [0] * DOF,
        "Touch": [0.0] * 14,
        "RobotConfig": [
            "P0000000000000000000",
            "6.0.0",
            "P0000000000000000000",
            "6.0.0",
        ],
    }

    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        print(self.__robot.getName() + " initialized.")
        rclpy.init(args=None)
        robot_name = str(self.__robot.getName())
        self.__node = rclpy.create_node(robot_name + "_driver")

        self.findAndEnableDevices()

        self.__node.create_subscription(
            JointRequests,
            robot_name + "/effectors/joint_requests",
            self.__joint_request_callback,
            10,
        )

        self.__joint_data_publisher = self.__node.create_publisher(
            JointData, robot_name + "/sensors/joint_data", 10
        )
        self.__imu_publisher = self.__node.create_publisher(
            Imu, robot_name + "/sensors/imu", 10
        )
        self.__fsr_publisher = self.__node.create_publisher(
            Fsr, robot_name + "/sensors/fsr", 10
        )

    def __joint_request_callback(self, joint_request):
        for index, position in zip(joint_request.indexes, joint_request.positions):
            self.motors[index].setPosition(position)
            if index == JointIndexes.LHIPYAWPITCH:
                self.motors[25].setPosition(position)

    def findAndEnableDevices(self):
        self.timeStep = int(self.__robot.getBasicTimeStep())
        self.fsr = []
        self.pos = []
        self.motors = []

        self.accelerometer = self.__robot.getDevice("accelerometer")
        self.gyro = self.__robot.getDevice("gyro")
        self.inertialUnit = self.__robot.getDevice("inertial unit")

        for i in ["LFsr", "RFsr"]:
            self.fsr.append(self.__robot.getDevice(i))

        # get motors
        for j in [
            "HeadYaw",
            "HeadPitch",
            "LShoulderPitch",
            "LShoulderRoll",
            "LElbowYaw",
            "LElbowRoll",
            "LWristYaw",
            "LHipYawPitch",
            "LHipRoll",
            "LHipPitch",
            "LKneePitch",
            "LAnklePitch",
            "LAnkleRoll",
            "RHipRoll",
            "RHipPitch",
            "RKneePitch",
            "RAnklePitch",
            "RAnkleRoll",
            "RShoulderPitch",
            "RShoulderRoll",
            "RElbowYaw",
            "RElbowRoll",
            "RWristYaw",
            "LPhalanx1",
            "RPhalanx1",
            "RHipYawPitch",
        ]:
            self.pos.append(self.__robot.getDevice(j + "S"))
            self.motors.append(self.__robot.getDevice(j))

        self.accelerometer.enable(self.timeStep)
        self.gyro.enable(self.timeStep)
        self.inertialUnit.enable(self.timeStep)

        for f in self.fsr:
            f.enable(self.timeStep)

        for s in self.pos:
            s.enable(self.timeStep)

    def updateSensors(self):
        # IMU
        a = self.accelerometer.getValues()
        self.sensors["Accelerometer"] = [-a[0], a[1], a[2]]
        g = self.gyro.getValues()
        self.sensors["Gyroscope"] = [g[0], g[1], -g[2]]
        imu = self.inertialUnit.getRollPitchYaw()
        self.sensors["Angles"] = [imu[0], imu[1]]

        # motors
        for i in range(self.DOF):
            self.sensors["Position"][i] = self.pos[i].getValue()

        # FSR
        # conversion is stolen from webots nao_demo_python controller
        fsv = [self.fsr[0].getValues(), self.fsr[1].getValues()]

        a = []
        a.append(
            fsv[0][2] / 3.4 + 1.5 * fsv[0][0] + 1.15 * fsv[0][1]
        )  # Left Foot Front Left
        a.append(
            fsv[0][2] / 3.4 + 1.5 * fsv[0][0] - 1.15 * fsv[0][1]
        )  # Left Foot Front Right
        a.append(
            fsv[0][2] / 3.4 - 1.5 * fsv[0][0] - 1.15 * fsv[0][1]
        )  # Left Foot Rear Right
        a.append(
            fsv[0][2] / 3.4 - 1.5 * fsv[0][0] + 1.15 * fsv[0][1]
        )  # Left Foot Rear Left

        a.append(
            fsv[1][2] / 3.4 + 1.5 * fsv[1][0] + 1.15 * fsv[1][1]
        )  # Right Foot Front Left
        a.append(
            fsv[1][2] / 3.4 + 1.5 * fsv[1][0] - 1.15 * fsv[1][1]
        )  # Right Foot Front Right
        a.append(
            fsv[1][2] / 3.4 - 1.5 * fsv[1][0] - 1.15 * fsv[1][1]
        )  # Right Foot Rear Right
        a.append(
            fsv[1][2] / 3.4 - 1.5 * fsv[1][0] + 1.15 * fsv[1][1]
        )  # Right Foot Rear Left
        for i in range(len(a)):
            self.sensors["FSR"][i] = max(0.0, a[i] / 25.0)  # fix scaling of values

        for i in range(0, 4):
            self.sensors["Touch"][i] = 0.0

    def publish_measurements(self):
        time_stamp = self.__node.get_clock().now().to_msg()
        self.__fsr_publisher.publish(
            Fsr(
                header=Header(stamp=time_stamp),
                l_foot_front_left=self.sensors["FSR"][0],
                l_foot_front_right=self.sensors["FSR"][1],
                l_foot_back_right=self.sensors["FSR"][2],
                l_foot_back_left=self.sensors["FSR"][3],
                r_foot_front_left=self.sensors["FSR"][4],
                r_foot_front_right=self.sensors["FSR"][5],
                r_foot_back_right=self.sensors["FSR"][6],
                r_foot_back_left=self.sensors["FSR"][7],
            )
        )
        self.__imu_publisher.publish(
            Imu(
                header=Header(stamp=time_stamp),
                angle_pitch=self.sensors["Angles"][1],
                angle_roll=self.sensors["Angles"][0],
                accelerometer=Vector3(
                    x=self.sensors["Accelerometer"][0],
                    y=self.sensors["Accelerometer"][1],
                    z=self.sensors["Accelerometer"][2],
                ),
                gyroscope=Vector3(
                    x=self.sensors["Gyroscope"][0],
                    y=self.sensors["Gyroscope"][1],
                    z=self.sensors["Gyroscope"][2],
                ),
            )
        )
        self.__joint_data_publisher.publish(
            JointData(
                header=Header(stamp=time_stamp),
                positions=self.sensors["Position"],
                statuses=self.sensors["Status"],
                stiffnesses=self.sensors["Stiffness"],
                temperatures=self.sensors["Temperature"],
                currents=self.sensors["Current"],
            )
        )

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        self.updateSensors()
        self.publish_measurements()
