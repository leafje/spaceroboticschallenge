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

import datetime
import time
import os
import struct

import rospy
import tf2_ros
import numpy as np
import cv2

from image_geometry import StereoCameraModel
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import CameraInfo
from srcsim.msg import Console

# constants
COLOR_NONE = 0
COLOR_RED = 1
COLOR_BLUE = 2
COLOR_GREEN = 3

# data for color masks
redMin = np.array([0,100,100])
redMax = np.array([10,255,255])
blueMin = np.array([110,100,100])
blueMax = np.array([130,255,255])
greenMin = np.array([50,100,100])
greenMax = np.array([70,255,255])

# data for general processing
imageLogPath = "/home/nibelheim/imagelog/{}/".format(datetime.datetime.now().strftime('%d%b%Y_%H%M%S'))

class Qual1:

    def __init__(self):
        # data for image disparity
        self.f = 0
        self.T = 0
        self.disparityImageData = None

        self.cameraInfoLeft = None
        self.cameraInfoRight = None
        self.headToWorldTransform = None
        self.consolePublisher = None

        # track current program state
        self.isDisparityDataReady = False
        self.isLightOn = False

        # track current image state
        self.currentColor = COLOR_NONE
        self.centerPixelLeftImage = None
        self.lightCounter = 0

    def getDisparityData(self, disparityData):
        if not self.isDisparityDataReady:
            rospy.loginfo('DisparityImage data: height = {} width = {} step = {}'.format(disparityData.image.height, disparityData.image.width, disparityData.image.step))
            rospy.loginfo('f = {} T = {} min_disparity = {} max_disparity = {} delta_d = {} step = {}'.format(
                disparityData.f, disparityData.T, disparityData.min_disparity, disparityData.max_disparity, 
                disparityData.delta_d, disparityData.image.step))

            imageBytes = np.ndarray(shape=(disparityData.image.height, disparityData.image.width, disparityData.image.step / disparityData.image.width), 
                dtype='uint8', buffer=disparityData.image.data)
            #imageBytes = cv2.flip(imageBytes, -1)
            self.disparityImageData = imageBytes
            #self.disparityImageData = cv2.cvtColor(imageBytes, cv2.CV_32FC1)
            self.f = disparityData.f
            self.T = disparityData.T
            cv2.imwrite("{}disparityImage.png".format(imageLogPath), self.disparityImageData)
            self.isDisparityDataReady = True
            rospy.loginfo('Disparity data: f = {} T = {}'.format(self.f, self.T))
            rospy.loginfo('Disparity data saved.')

    # returns a 2-element tuple with the center x and y coordinate
    def getCenterPixel(self, litPixels):
        #print('Lit pixels: min = {} max = {} mean = {}'.format(np.amin(litPixels, axis=0), np.amax(litPixels, axis=0), np.mean(litPixels, axis=0)))
        meanPixelValue = np.mean(litPixels, axis = 0)
        heightCoord = int(round(meanPixelValue[0]))
        widthCoord = int(round(meanPixelValue[1]))

        # sanity check
        if (heightCoord < 132 or heightCoord > 390):
            rospy.loginfo('heightCoord out of range: {}'.format(heightCoord))
        if (widthCoord < 366 or widthCoord > 690):
            rospy.loginfo('widthCoord out of range: {}'.format(widthCoord))

        return (heightCoord, widthCoord)

    def watchLeftCamera(self, msgData):
        imageBytes = np.ndarray(shape=(msgData.height, msgData.width, 3), dtype='uint8', buffer=msgData.data)
        # flip image on both axes (the camera is mounted upside-down)
        #imageBytes = cv2.flip(imageBytes, -1)

        color, pixels = self.getLitPixels(imageBytes)

        if self.isLightOn:
            if color == COLOR_NONE:
                # the light turned off.
                rospy.loginfo('The light has turned off.')
                self.isLightOn = False
                self.isLightProcessed = False
                # let the right camera callback reset the current color
            else:
                # do nothing - a light was on, and is still on
                pass
        else:
            if color == COLOR_NONE:
                # do nothing - all lights were off, and are still off
                pass
            else:
                rospy.loginfo('A light has turned on.')
                self.lightCounter = self.lightCounter + 1
                self.isLightOn = True
                self.currentColor = color
                self.centerPixelLeftImage = self.getCenterPixel(pixels)
                rospy.loginfo('Light color = {}, center pixel (h,w) = {}'.format(color, self.centerPixelLeftImage))
                colorFlippedImage = cv2.cvtColor(imageBytes, cv2.COLOR_BGR2RGB)
                cv2.imwrite("{}LeftImage{}.png".format(imageLogPath, self.lightCounter), colorFlippedImage)

                hCoord = self.centerPixelLeftImage[0]
                wCoord = self.centerPixelLeftImage[1]
                #print self.disparityImageData[wCoord, hCoord]
                depthValue = struct.unpack_from('f', self.disparityImageData[hCoord, wCoord])[0]
                if depthValue < 0:
                    rospy.logerr('Got invalid depth value {} for ({},{}.)'.format(depthValue, wCoord, hCoord))
                    # rospy.logerr('Shutting down.')
                    # exit()

                rospy.loginfo('Depth data for (h, w) ({},{}) (in pixels) = {}'.format(hCoord, wCoord, depthValue))

                scm = StereoCameraModel()
                scm.fromCameraInfo(self.cameraInfoLeft, self.cameraInfoRight)
                worldCoords = scm.projectPixelTo3d((wCoord, hCoord), depthValue)
                worldCoords = -1 * np.array(worldCoords)
                #zCoord = scm.getZ(depthValue)
                #print zCoord
                rospy.loginfo('Depth data for ({},{}) (in world coordinates) = {}'.format(worldCoords[0], worldCoords[1], worldCoords[2]))

                # worldCoordsByHand = self.getWorldCoords(wCoord, hCoord, depthValue)
                # print worldCoordsByHand

                # The center pixel is not pointing at 0,0,0, so we need to adjust the world coordinates
                # calculated from the stereo image processing, by an offset. Offset found by analyzing
                # the console mesh in Blender.
                # multiply the Z (height) coordinate by -1, since by convention y points down in the head
                # frame, which is not how the world frame is set up.
                # +x - to the right
                # +y - behind the robot
                # +z - up
                adjustedWorldCoords = (worldCoords[0] + 0.122, worldCoords[2] - 0.126, -worldCoords[1] + 1.23)
                rospy.loginfo('Adjusted world coordinates: x = {} depth = {} height = {}'.format(adjustedWorldCoords[0], adjustedWorldCoords[1], adjustedWorldCoords[2]))

                # +x - behind the robot
                # +y - to the left of the robot
                # +z - up
                relativeCoordinates = (
                    self.headToWorldTransform.transform.translation.y - adjustedWorldCoords[1],
                    adjustedWorldCoords[0] - self.headToWorldTransform.transform.translation.x,
                    adjustedWorldCoords[2] - self.headToWorldTransform.transform.translation.z
                )

                rospy.loginfo('Relative to head coordinates: x = {} depth = {} height = {}'.format(relativeCoordinates[0], relativeCoordinates[1], relativeCoordinates[2]))
                
                # if self.currentColor == COLOR_RED:
                #     self.saveEntry(relativeCoordinates[0], relativeCoordinates[1], relativeCoordinates[2], 1.0, 0.0, 0.0)
                # elif self.currentColor == COLOR_BLUE:
                #     self.saveEntry(relativeCoordinates[0], relativeCoordinates[1], relativeCoordinates[2], 0.0, 0.0, 1.0)
                # elif self.currentColor == COLOR_GREEN:
                #     self.saveEntry(relativeCoordinates[0], relativeCoordinates[1], relativeCoordinates[2], 0.0, 1.0, 0.0)

                if self.currentColor == COLOR_RED:
                    self.saveEntry(-worldCoords[2], worldCoords[0], worldCoords[1], 1.0, 0.0, 0.0)
                elif self.currentColor == COLOR_BLUE:
                    self.saveEntry(-worldCoords[2], worldCoords[0], worldCoords[1], 0.0, 0.0, 1.0)
                elif self.currentColor == COLOR_GREEN:
                    self.saveEntry(-worldCoords[2], worldCoords[0], worldCoords[1], 0.0, 1.0, 0.0)

    def saveEntry(self, x, y, z, r, g, b):
        msg = Console()
        msg.x = x
        msg.y = y
        msg.z = z
        msg.r = r
        msg.g = g
        msg.b = b
        rospy.loginfo('Publishing this ROS Console message:\n{}'.format(msg))
        self.consolePublisher.publish(msg)

    def getWorldCoords(self, x, y, d):
        pX = self.cameraInfoLeft.P[2]
        pY = self.cameraInfoLeft.P[6]
        translationMatrix = np.array([[1, 0, 0, -pX], [0, 1, 0, -pY], [0, 0, 0, self.f], [0, 0, -(1/self.T), 0]])
        coordVector = np.array([x, y, d, 1])
        unscaledCoords = np.matmul(translationMatrix, coordVector)
        return unscaledCoords[0]/unscaledCoords[3], unscaledCoords[1]/unscaledCoords[3], unscaledCoords[2]/unscaledCoords[3]

    # returns an array of x/y coordinates that were within the given mask boundaries
    # returns an empty array (size == 0) if no matching pixels were found
    def getLitPixelsColor(self, image, maskMin, maskMax):
        imageHsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(imageHsv, maskMin, maskMax)
        maskedImage = cv2.bitwise_and(imageHsv, imageHsv, mask=mask)

        goodPixels = np.nonzero(maskedImage)
        if np.size(goodPixels) == 0:
            return np.empty(shape=(0,0))
        goodPixels = np.transpose(goodPixels)
        goodPixelsNoDups = np.vstack({tuple(row) for row in goodPixels[:,0:2]})
        sortedIndices = np.lexsort((goodPixelsNoDups[:,1],goodPixelsNoDups[:,0]))
        return goodPixelsNoDups[sortedIndices]

    # returns a tuple of 2 elements:
    # first element is color, second element is a numpy array of the pixels that
    # are lit by the color of the first element. if no pixels are lit (i.e., color
    # is equal to COLOR_NONE), then the returned pixel array is equal to np.empty()
    def getLitPixels(self, image):
        color = COLOR_NONE
        pixels = np.empty(shape=(0,0))

        # TODO: check whether more than 1 mask returns good pixels. If so, we need to error out.

        litPixels = self.getLitPixelsColor(image, redMin, redMax)
        if litPixels.size != 0:
            pixels = litPixels
            color = COLOR_RED
        else:
            litPixels = self.getLitPixelsColor(image, blueMin, blueMax)
            if litPixels.size != 0:
                pixels = litPixels
                color = COLOR_BLUE
            else:
                litPixels = self.getLitPixelsColor(image, greenMin, greenMax)
                if litPixels.size != 0:
                    pixels = litPixels
                    color = COLOR_GREEN

        return color, pixels

    def saveCameraInfoLeft(self, data):
        if self.cameraInfoLeft is None:
            self.cameraInfoLeft = data
            self.printCameraInfo(data, 'Left')

    def saveCameraInfoRight(self, data):
        if self.cameraInfoRight is None:
            self.cameraInfoRight = data
            self.printCameraInfo(data, 'Right')

    def printCameraInfo(self, data, side):
        print('{} camera info:'.format(side))
        print('Height = {} width = {} distortion_model = {}'.format(data.height, data.width, data.distortion_model))
        print('D = {}'.format(data.D))
        print('K = {}'.format(data.K))
        print('R = {}'.format(data.R))
        print('P = {}'.format(data.P))
        print('binning_x = {} binning_y = {}'.format(data.binning_x, data.binning_y))
        print('ROI: x_offset = {} y_offset = {} height = {} width = {} do_rectify = {}'.format(data.roi.x_offset, data.roi.y_offset, data.roi.height, data.roi.width, data.roi.do_rectify))

    def mainloop(self):
        # The qual1 simulation should be running, but do not send the signal to start the lights
        # until the program outputs the relevant message. This program collects a variety of data
        # at startup and it is not ready to process the lights until that startup processing is 
        # complete.

        rospy.init_node('conflux_qual1', anonymous=True)

        # Make a folder to save images for offline investigation. Include a timestamp in the folder name
        if not os.path.exists(imageLogPath):
            rospy.loginfo('Image path: {}'.format(imageLogPath))
            os.mkdir(imageLogPath)
        else:
            raise Exception('Image log folder already exists!')

        # Initialize transform buffer for converting light locations to be 
        # relative to head frame
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)
        
        rospy.Subscriber("/multisense/camera/disparity", DisparityImage, self.getDisparityData)
        rospy.Subscriber('/multisense/camera/left/camera_info', CameraInfo, self.saveCameraInfoLeft)
        rospy.Subscriber('/multisense/camera/right/camera_info', CameraInfo, self.saveCameraInfoRight)

        while not self.isDisparityDataReady:
            time.sleep(1)

        counter = 0
        while self.headToWorldTransform is None:
            try:
                self.headToWorldTransform = tfBuffer.lookup_transform('world', 'head', rospy.Time())
                rospy.loginfo('Head location (world frame): x = {} depth = {} height = {}'.format(
                    self.headToWorldTransform.transform.translation.x, self.headToWorldTransform.transform.translation.y, self.headToWorldTransform.transform.translation.z))
            except (tf2_ros.ExtrapolationException):
                # if we don't have a good transform after 10 seconds, give up.
                if (counter < 10):
                    counter = counter + 1
                    time.sleep(1)
                    continue
                else:
                    raise
        
        self.consolePublisher = rospy.Publisher('/srcsim/qual1/light', Console, queue_size=10)

        # All initialization data is ready, so start processing.
        rospy.Subscriber("/multisense/camera/left/image_rect_color", Image, self.watchLeftCamera)

        # start the lights. Note: you have to restart the simulation if you want to run the lights again.
        rospy.loginfo('Initialization is complete. You may now start the light sequence.')

        # Now, just run forever. spin() simply keeps python from exiting until this node is stopped.
        rospy.spin()

if __name__ == '__main__':
    q = Qual1()
    q.mainloop()