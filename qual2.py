#!/usr/bin/env python

#  Copyright 2017 Jennifer Leaf

#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at

#      http://www.apache.org/licenses/LICENSE-2.0

#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import copy
import time
import rospy
import tf
import tf2_ros
import numpy
from numpy import append
from sensor_msgs.msg import JointState
from pandas import Series, DataFrame
import pandas as pd

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

from ihmc_msgs.msg import FootstepStatusRosMessage
from ihmc_msgs.msg import FootstepDataListRosMessage
from ihmc_msgs.msg import FootstepDataRosMessage
from ihmc_msgs.msg import GoHomeRosMessage

from ihmc_msgs.msg import ArmTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage

LEFT = 0
RIGHT = 1

ROBOT_NAME = None
LEFT_FOOT_FRAME_NAME = None
RIGHT_FOOT_FRAME_NAME = None

#PUSH_BUTTON = [0.0, 0.2, 0.0, 1.3, 0.0, 0.0, 0.0]
# ZERO_ARM = [0,0,0,0,0,0,0]
# PUSH_BUTTON = [1.75, -0.8, -1.0, 0.25, 0.0, 0.0, 0.0]

# These match with values published on the /ihmc_ros/valkyrie/output/joint_states topic.
# correspond to: 'rightShoulderPitch', 'rightShoulderRoll', 'rightShoulderYaw', 'rightElbowPitch', 'rightForearmYaw', 'rightWristRoll', 'rightWristPitch'
# 'rightForearmYaw', 'rightWristRoll', 'rightWristPitch' are non-functional.
ARM_FOLDED_IN = [0.0, 1.2, 0.5, 1.65, 0.0, 0.0, 0.0]
#ARM_FOLDED_IN = [-0.2, 1.2, 0.71, 1.51, 0.0, 0.0, 0.0]
#ARM_PUSH_BUTTON = [1.75, -0.8, -1.0, 0.3, 0.0, 0.0, 0.0]
ARM_PUSH_BUTTON = [1.75, -0.6, -0.9, 0.55, 0.0, 0.0, 0.0]

#lastJointData = DataFrame(columns=['joint', 'timestamp', 'position', 'velocity', 'effort'])
counter = 0

ARM_STATE_INITIAL = 0
ARM_STATE_EXTEND = 1
ARM_STATE_RETRACTED = 2

armState = ARM_STATE_INITIAL

def runQual():
    # set up the walking parameters
    msg = FootstepDataListRosMessage()
    msg.transfer_time = 0.5
    msg.swing_time = 0.5
    msg.execution_mode = 0
    msg.unique_id = int(time.time())

    # walk forward to the door
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.2, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.4, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [0.6, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [0.8, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [1.0, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [1.2, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [1.4, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [1.6, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [1.81, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [2.02, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [2.23, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [2.44, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [2.65, 0.0, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(RIGHT, [2.86, -0.02, 0.0]))
    msg.footstep_data_list.append(createFootStepOffset(LEFT, [2.86, 0.0, 0.0]))
    print msg

    footStepListPublisher.publish(msg)
    rospy.loginfo('walk forward...')
    waitForFootsteps(len(msg.footstep_data_list))

    time.sleep(5)

    #new_unique_id = msg.unique_id + 1
    new_unique_id = -1

    # move arm into position to push the button.
    msg2 = ArmTrajectoryRosMessage()
    msg2.robot_side = ArmTrajectoryRosMessage.RIGHT
    msg2 = appendTrajectoryPoint(msg2, 2.0, ARM_PUSH_BUTTON)
    msg2 = appendTrajectoryPoint(msg2, 3.0, ARM_FOLDED_IN)

    #msg.unique_id = int(time.time())
    msg2.unique_id = new_unique_id
    #rospy.loginfo('move right arm: {}'.format(msg2))
    armTrajectoryPublisher.publish(msg2)
    time.sleep(5)

    global armState
    while armState != ARM_STATE_RETRACTED:
        time.sleep(0.1)

    goHome()
    time.sleep(1)

    # set up the walking parameters
    msg3 = FootstepDataListRosMessage()
    msg3.transfer_time = 0.5
    msg3.swing_time = 0.5
    msg3.execution_mode = 0
    #msg3.unique_id = int(time.time())
    msg3.unique_id = -1

    # walk forward to the door
    # msg3.footstep_data_list.append(createFootStepOffset(RIGHT, [2.96, -0.02, 0.0], 3))
    # msg3.footstep_data_list.append(createFootStepOffset(LEFT, [3.06, 0.0, 0.0], 3))
    # msg3.footstep_data_list.append(createFootStepOffset(RIGHT, [3.26, -0.02, 0.0], 3))
    # msg3.footstep_data_list.append(createFootStepOffset(LEFT, [3.46, 0.0, 0.0]))
    # msg3.footstep_data_list.append(createFootStepOffset(RIGHT, [3.66, -0.02, 0.0]))
    # msg3.footstep_data_list.append(createFootStepOffset(LEFT, [3.86, 0.0, 0.0]))
    # msg3.footstep_data_list.append(createFootStepOffset(RIGHT, [4.06, -0.02, 0.0]))
    # msg3.footstep_data_list.append(createFootStepOffset(LEFT, [4.26, 0.0, 0.0]))
    # msg3.footstep_data_list.append(createFootStepOffset(RIGHT, [4.46, -0.02, 0.0]))
    # msg3.footstep_data_list.append(createFootStepOffset(LEFT, [4.66, 0.0, 0.0]))
    # msg3.footstep_data_list.append(createFootStepOffset(RIGHT, [4.86, -0.02, 0.0]))
    # msg3.footstep_data_list.append(createFootStepOffset(LEFT, [5.06, 0.0, 0.0]))
    # msg3.footstep_data_list.append(createFootStepOffset(RIGHT, [5.26, -0.02, 0.0]))
    # msg3.footstep_data_list.append(createFootStepOffset(LEFT, [5.46, 0.0, 0.0]))
    # msg3.footstep_data_list.append(createFootStepOffset(RIGHT, [5.66, -0.02, 0.0]))
    msg3.footstep_data_list.append(createFootStepOffset(LEFT, [0.2, 0.0, 0.0]))
    msg3.footstep_data_list.append(createFootStepOffset(RIGHT, [0.4, -0.02, 0.0]))
    msg3.footstep_data_list.append(createFootStepOffset(LEFT, [0.6, -0.01, 0.0]))
    msg3.footstep_data_list.append(createFootStepOffset(RIGHT, [0.8, -0.04, 0.0]))
    msg3.footstep_data_list.append(createFootStepOffset(LEFT, [1.0, -0.02, 0.0]))
    msg3.footstep_data_list.append(createFootStepOffset(RIGHT, [1.2, -0.04, 0.0]))
    msg3.footstep_data_list.append(createFootStepOffset(LEFT, [1.4, -0.02, 0.0]))
    msg3.footstep_data_list.append(createFootStepOffset(RIGHT, [1.6, -0.04, 0.0]))
    msg3.footstep_data_list.append(createFootStepOffset(LEFT, [1.81, -0.02, 0.0]))
    msg3.footstep_data_list.append(createFootStepOffset(RIGHT, [2.02, -0.04, 0.0]))

    print(msg3)

    footStepListPublisher.publish(msg3)
    rospy.loginfo('walk forward...')
    waitForFootsteps(len(msg3.footstep_data_list))

def goHome():
    rospy.loginfo('Going to home position.')
    msg = GoHomeRosMessage()
    msg.body_part = 1
    msg.trajectory_time = 1.0
    msg.unique_id = -1
    goHomePublisher.publish(msg)

    msg.body_part = 2
    goHomePublisher.publish(msg)

# Creates footstep with the current position and orientation of the foot.
def createFootStepInPlace(stepSide):
    footstep = FootstepDataRosMessage()
    footstep.robot_side = stepSide

    if stepSide == LEFT:
        foot_frame = LEFT_FOOT_FRAME_NAME
    else:
        foot_frame = RIGHT_FOOT_FRAME_NAME

    footWorld = tfBuffer.lookup_transform('world', foot_frame, rospy.Time())
    footstep.orientation = footWorld.transform.rotation
    footstep.location = footWorld.transform.translation

    return footstep

# Creates footstep offset from the current foot position. The offset is in foot frame.
def createFootStepOffset(stepSide, offset, trajectory=0):
    footstep = createFootStepInPlace(stepSide)
    footstep.trajectory_type = trajectory

    # transform the offset to world frame
    quat = footstep.orientation
    rot = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
    transformedOffset = numpy.dot(rot[0:3, 0:3], offset)

    footstep.location.x += transformedOffset[0]
    footstep.location.y += transformedOffset[1]
    footstep.location.z += transformedOffset[2]

    return footstep

def waitForFootsteps(numberOfSteps):
    global stepCounter
    stepCounter = 0
    while stepCounter < numberOfSteps:
        rate.sleep()
    rospy.loginfo('finished set of steps')

def recievedFootStepStatus(msg):
    global stepCounter
    if msg.status == 1:
        stepCounter += 1

def appendTrajectoryPoint(arm_trajectory, time, positions):
    if not arm_trajectory.joint_trajectory_messages:
        arm_trajectory.joint_trajectory_messages = [copy.deepcopy(OneDoFJointTrajectoryRosMessage()) for i in range(len(positions))]
    for i, pos in enumerate(positions):
        point = TrajectoryPoint1DRosMessage()
        point.time = time
        point.position = pos
        point.velocity = 0
        arm_trajectory.joint_trajectory_messages[i].trajectory_points.append(point)
    return arm_trajectory

def jointCallback(msgData):
    global counter
    counter = counter + 1
    #data = np.column_stack((msgData.name, msgData.position, msgData.velocity, msgData.effort))

    # The robot controller publishes updates every 2ms in simulation time. So adjust the 
    # mod value depending on how frequently you want to capture updates.
    # i.e. % 500 = once per simulation second
    # % 50 = 10Hz
    # % 5 = 100Hz
    if (counter % 20 == 0):
        currentData = DataFrame(columns=['joint', 'timestamp', 'position', 'velocity', 'effort'])
        currentData['joint'] = msgData.name
        currentData['timestamp'] = msgData.header.stamp.to_sec()
        currentData['position'] = msgData.position
        currentData['velocity'] = msgData.velocity
        currentData['effort'] = msgData.effort
        # print len(currentData.index)
        # print currentData.dtypes
        #lastJointData = currentData
        #print len(self.data.index)

        global armState
        currentRightShoulderPitch = currentData[currentData['joint'] == 'rightShoulderPitch']['position'].item()
        
        # if the current right shoulder pitch is within 5% of 1 radian of the target pitch, that's good enough.
        if armState == ARM_STATE_INITIAL:
            targetRightShoulderPitch = ARM_PUSH_BUTTON[0]
            #rospy.loginfo('Checking joints: current joint data = {}, reference value = {}'.format(currentRightShoulderPitch, targetRightShoulderPitch))
            if abs(currentRightShoulderPitch - targetRightShoulderPitch) < 0.05:
                rospy.loginfo('Arm is now extended.')
                armState = ARM_STATE_EXTEND
        elif armState == ARM_STATE_EXTEND:
            targetRightShoulderPitch = ARM_FOLDED_IN[0]
            #rospy.loginfo('Checking joints: current joint data = {}, reference value = {}'.format(currentRightShoulderPitch, targetRightShoulderPitch))
            if abs(currentRightShoulderPitch - targetRightShoulderPitch) < 0.05:
                rospy.loginfo('Arm is now retracted.')
                armState = ARM_STATE_RETRACTED


if __name__ == '__main__':
    try:
        rospy.init_node('ihmc_walk_test')

        if not rospy.has_param('/ihmc_ros/robot_name'):
            rospy.logerr("Cannot run walk_test.py, missing parameters!")
            rospy.logerr("Missing parameter '/ihmc_ros/robot_name'")

        else:
            ROBOT_NAME = rospy.get_param('/ihmc_ros/robot_name')

            armTrajectoryPublisher = rospy.Publisher("/ihmc_ros/{0}/control/arm_trajectory".format(ROBOT_NAME), ArmTrajectoryRosMessage, queue_size=1)

            right_foot_frame_parameter_name = "/ihmc_ros/{0}/right_foot_frame_name".format(ROBOT_NAME)
            left_foot_frame_parameter_name = "/ihmc_ros/{0}/left_foot_frame_name".format(ROBOT_NAME)

            if rospy.has_param(right_foot_frame_parameter_name) and rospy.has_param(left_foot_frame_parameter_name):
                RIGHT_FOOT_FRAME_NAME = rospy.get_param(right_foot_frame_parameter_name)
                LEFT_FOOT_FRAME_NAME = rospy.get_param(left_foot_frame_parameter_name)

                footStepStatusSubscriber = rospy.Subscriber("/ihmc_ros/{0}/output/footstep_status".format(ROBOT_NAME), FootstepStatusRosMessage, recievedFootStepStatus)
                footStepListPublisher = rospy.Publisher("/ihmc_ros/{0}/control/footstep_list".format(ROBOT_NAME), FootstepDataListRosMessage, queue_size=1)
                rospy.Subscriber("/ihmc_ros/valkyrie/output/joint_states", JointState, jointCallback)
                goHomePublisher = rospy.Publisher('/ihmc_ros/valkyrie/control/go_home', GoHomeRosMessage, queue_size=1)

                tfBuffer = tf2_ros.Buffer()
                tfListener = tf2_ros.TransformListener(tfBuffer)

                rate = rospy.Rate(10) # 10hz
                time.sleep(1)

                # make sure the simulation is running otherwise wait
                if footStepListPublisher.get_num_connections() == 0:
                    rospy.loginfo('waiting for subsciber...')
                    while footStepListPublisher.get_num_connections() == 0:
                        rate.sleep()

                # make sure the simulation is running otherwise wait
                if armTrajectoryPublisher.get_num_connections() == 0:
                    rospy.loginfo('waiting for subscriber...')
                    while armTrajectoryPublisher.get_num_connections() == 0:
                        rate.sleep()                        

                if not rospy.is_shutdown():
                    runQual()
            else:
                if not rospy.has_param(left_foot_frame_parameter_name):
                    rospy.logerr("Missing parameter {0}".format(left_foot_frame_parameter_name))
                if not rospy.has_param(right_foot_frame_parameter_name):
                    rospy.logerr("Missing parameter {0}".format(right_foot_frame_parameter_name))

    except rospy.ROSInterruptException:
        pass
