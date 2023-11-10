#!/usr/bin/env python3

import rospy, rospkg, sys
from typing import List

# Import ROS Messages
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
from alexa_voice_control.msg import VoiceCommand

# Import Parent Folders
sys.path.append(f'{rospkg.RosPack().get_path("alexa_voice_control")}/script/utils')

# Move Robot Utilities
from move_robot import UR10e_RTDE_Move, GRIPPER_OPEN, GRIPPER_CLOSE
from command_list import *
from object_list import HOME_POSITION, PLACE_POSITION, HOLD_POSITION, MOUNT_POSITION, get_object_positions

"""

Labelling:

    0: Empty Command
    1: Begin Experiment
    2: Provide Screw
    3: Provide Screwdriver
    4: Hold Object
    5: Take Object
    6: Move Mounting

"""

class ExperimentManager():

    # Flags
    experiment_started = False

    def __init__(self) -> None:

        # Init ROS Node
        rospy.init_node('Experiment_Manager')

        # Instance Robot Movement Class
        self.robot = UR10e_RTDE_Move()

        # Publishers
        self.ttsPub   = rospy.Publisher('/alexa/tts', String, queue_size=1)

        # Subscribers
        rospy.Subscriber('/alexa/voice_command', VoiceCommand, self.commandCallback)

        # Load Parameters
        self.gripper_enabled = rospy.get_param('/experiment/gripper_enabled', True)

        # Initialization Sleep
        rospy.sleep(1)

    def commandCallback(self, data:VoiceCommand):

        rospy.logwarn(f"Received Command: {data}")

        # Start / Stop Experiment Command -> Set Flags
        if   data.command == BEGIN_EXPERIMENT: self.experiment_started = True; return

        # Empty or Unused Commands -> Do Nothing
        elif data.command in [NULL]: return

        # `Provide Screw` Command -> Pick-And-Place Command
        elif data.command in [PROVIDE_SCREW]: self.pick_and_place('screws')

        # `Provide Screwdriver` Command -> Pick-And-Place Command
        elif data.command in [PROVIDE_SCREWDRIVER]: self.pick_and_place('screwdriver')

        # `Hold Object` Command -> Hold Object Command
        elif data.command in [HOLD_OBJECT]: self.hold_object()

        # `Take Object` Command -> Take Object Command
        elif data.command in [TAKE_OBJECT]: self.take_object()

        # `Move Mounting` Command -> Move Mounting Command
        elif data.command in [MOVE_MOUNTING]: self.move_mounting()

    def pick_and_place(self, object_name) -> bool:

        """ Handover Object """

        # Get Object Pick and Place Positions
        pick_position, place_position = get_object_positions(object_name)
        assert pick_position is not None and place_position is not None, f"Object Positions not Found for Object {object_name}"

        # Move Gripper to Starting Position
        rospy.loginfo('Open Gripper')
        if not self.robot.move_gripper(GRIPPER_OPEN, gripper_enabled=self.gripper_enabled): return False

        # Forward + Inverse Kinematic -> Increase z + 10cm
        rospy.loginfo('Forward Kinematic + Inverse Kinematic -> Increase z + 10cm')
        pick_position_cartesian: Pose = self.robot.FK(pick_position)
        pick_position_cartesian.position.z += 0.10
        pick_position_up: List[float] = self.robot.IK(pick_position_cartesian, pick_position)

        # Move 10cm Over the Object
        rospy.loginfo('Move Over the Object')
        if not self.robot.move_joint(pick_position_up): return False

        # Move to Object
        rospy.loginfo('Move To the Object')
        if not self.robot.move_joint(pick_position): return False

        # Grip Object
        rospy.loginfo('Close Gripper')
        if object_name == 'screws': 
            if not self.robot.move_gripper(GRIPPER_CLOSE, 50, 50, gripper_enabled=self.gripper_enabled): return False
        elif not self.robot.move_gripper(GRIPPER_CLOSE, gripper_enabled=self.gripper_enabled): return False
        rospy.sleep(1)

        # Move 10cm Over the Object
        rospy.loginfo('Move Over the Object')
        if not self.robot.move_joint(pick_position_up): return False

        # Forward + Inverse Kinematic -> Increase z + 10cm
        rospy.loginfo('Forward Kinematic + Inverse Kinematic -> Increase z + 10cm')
        place_position_cartesian: Pose() = self.robot.FK(place_position)
        place_position_cartesian.position.z += 0.10
        place_position_up: List[float] = self.robot.IK(place_position_cartesian, place_position)

        # Move 10cm Over the Place Position
        rospy.loginfo('Move Over the Place Position')
        if not self.robot.move_joint(place_position_up): return False

        # Move to Place Position
        rospy.loginfo('Move To the Place Position')
        if not self.robot.move_joint(place_position): return False

        # Release Object
        rospy.loginfo('Open Gripper')
        if not self.robot.move_gripper(GRIPPER_OPEN, gripper_enabled=self.gripper_enabled): return False
        rospy.sleep(1)

        # Move 10cm Over the Place Position
        rospy.loginfo('Move Over the Place Position')
        if not self.robot.move_joint(place_position_up): return False

        # Move to Home
        rospy.loginfo('Move To Home')
        if not self.robot.move_joint(HOME_POSITION): return False

        return True

    def hold_object(self) -> bool:

        """ Hold Object """

        # Move Gripper to Starting Position
        rospy.loginfo('Open Gripper')
        if not self.robot.move_gripper(GRIPPER_OPEN, gripper_enabled=self.gripper_enabled): return False

        # Move to Object Hold Position
        rospy.loginfo('Move To the Hold Position')
        if not self.robot.move_joint(HOLD_POSITION): return False

        return True

    def take_object(self) -> bool:

        """ Take Object """

        # Grip Object
        rospy.loginfo('Close Gripper')
        if not self.robot.move_gripper(GRIPPER_CLOSE, gripper_enabled=self.gripper_enabled): return False
        rospy.sleep(1)

        return True

    def move_mounting(self) -> bool:

        """ Move Mounting """

        # Move to Mount Position
        rospy.loginfo('Move To the Mount Position')
        if not self.robot.move_joint(MOUNT_POSITION): return False

        return True

    def run(self):

        """ Run the Experiment """

        # Move to Home
        rospy.loginfo('Move To Home')
        if not self.robot.move_joint(HOME_POSITION): return False

        # Wait for Experiment Start
        while not self.experiment_started and not rospy.is_shutdown(): rospy.loginfo_throttle(5, 'Waiting for Experiment Start')

        # Start Experiment -> Pick-And-Place Base
        rospy.logwarn('Start Experiment - Move to Home')
        self.pick_and_place('base')
        
        rospy.spin()

if __name__ == '__main__':

    # Initialize Experiment Manager Node
    exp = ExperimentManager()

    # Run the Experiment
    while not rospy.is_shutdown(): exp.run()
