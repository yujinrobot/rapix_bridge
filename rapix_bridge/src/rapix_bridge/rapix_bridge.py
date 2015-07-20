#!/usr/bin/env python
# -*- coding:utf-8 -*-
#
# License: BSD
#

##############################################################################
# Imports
##############################################################################
import time

import sys
reload(sys)  # Reload does the trick!
sys.setdefaultencoding('UTF8')

# robosem
##############################################################################
import socket
import errno
import threading
import struct
from .cmdset import CCmdSet
import abc

# ros
##############################################################################
import rospy
import std_msgs.msg as std_msgs

class RAPIXBridge():

    __meta_class = abc.ABCMeta

    def __init__(self, name="ROCONBRIDGE", host='localhost', port=6001):
        # robosem
        self.NAME = name
        self.HOST = host 
        self.PORT = int(port)
        self.TIME_OUT_SOCKET = 5
        self.DISCRIMINATOR = 2503011588 # Defines start of rapix msg
        self.VERSION = 0x00020003
        self.is_connecting = False
        self.cmd_id = 0
        self.res_cmdset_list = []
        self.evt_cmdset_list = []
        self.loginfo("robosem connect info: [name: %s][host: %s][port: %s]" % (name, host, str(port)))

        # ros
        self._publishers = self.init_subscriber()
        self._subscribers = self.init_publisher()

    def spin(self):

        while not rospy.is_shutdown():
            self.connect()
            self.init_robot()
            self.recv()
            self.disconnet()
            rospy.sleep(0.1)

    def connect(self):
        socket.setdefaulttimeout(self.TIME_OUT_SOCKET)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        while not rospy.is_shutdown() and not self.is_connecting:
            try:
                self.loginfo('try to connect robot, Please turn on the robot')
                self.s.connect((self.HOST, self.PORT))
            except Exception, e:
                self.is_connecting = False
                rospy.sleep(5)
            else:
                self.is_connecting = True

    def disconnet(self):
        try:
            self.s.close()
        except Exception, e:
            self.logwarn('disconnet failed. Reason:%s' % str(e))
        else:
            self.is_connecting = False

    def init_robot(self):
        if self.is_connecting:
            self.cmd_id += 1
            cmdset = CCmdSet(
                "WhoIam", "REQUEST_MESSAGE", self.NAME, "RVM", 0, " ", 0, 0, " ", self.cmd_id)
            self.s.send(cmdset.getCmdSet())

    def recv(self):
        self.loginfo('Start Robot')
        while not rospy.is_shutdown() and self.is_connecting:
            try:
                data = self.s.recv(12)
                if len(data) != 12:
                    data += self.s.recv(12 - len(data))

            except socket.timeout, e:
                if not self.is_connecting:
                    self.logwarn(
                        'Exception error in receive header data. Reason:%s' % str(e))
                continue

            except socket.error, e:
                if e.errno == errno.ECONNRESET:
                    self.logwarn(
                        'Exception error in receive header data. Reason:%s' % str(e))
                    break
                else:
                    self.logwarn(
                        'Exception error in receive header data. Reason:%s' % str(e))
                    continue

            except Exception, e:
                self.logwarn(
                    'Exception error in receive header data. Reason:%s' % str(e))
                continue

            try:
                buffer = struct.unpack('I I I', data)
            except Exception, e:
                self.logwarn(
                    'Exception error in header unpack. Reason:%s' % str(e))
                break

            disc = buffer[0]
            ver = buffer[1]
            data_length = buffer[2]

            if disc != self.DISCRIMINATOR or ver != self.VERSION or data_length == 0:
                self.loginfo(
                    "[Wrong Data ][nDisc: %d][nVer: %d][nDataLen: %d]" % (disc, ver, data_length))
                continue

            try:
                data = self.s.recv(data_length)
                if len(data) != data_length:
                    self.loginfo('More Receive Data')
                    data += self.s.recv(data_length - len(data))
            except Exception, e:
                self.loginfo(
                    'Exception error in receive body data. Reason:%s' % str(e))
                continue

            try:
                buffer = struct.unpack('%ds' % data_length, data)
            except Exception, e:
                self.loginfo(
                    'Exception error in body unpack. Reason:%s' % str(e))
                rospy.sleep(5)
                continue

            data = buffer[0]
            cmdset = CCmdSet()
            cmdset.parse(data)

            print "MSGT: [", cmdset.m_msgtype, " ] Name: [", cmdset.m_cmdname, "] REC: [", cmdset.m_receiverID, "] SEND: [", cmdset.m_senderID, "] CMD: [", cmdset.m_cmdID, "]"
            if cmdset.m_msgtype == "RESPONSE_MESSAGE":
                self.res_proc(cmdset)
            elif cmdset.m_msgtype == "EVENT_MESSAGE":
                self.event_proc(cmdset)

    def event_proc(self, cmdset):
        if cmdset.m_cmdname == "TouchSensorEvent":
            button_id = 0
            status = 0
            for param in cmdset.m_params:
                if param['name'] == 'ButtonID':
                    button_id = param['value']
                if param['name'] == 'Status':
                    status = param['value']
            self.process_touch_event(button_id, status)

    def res_proc(self, cmdset):
        pass

    def loginfo(self, msg):
        rospy.loginfo('%s : %s'%(rospy.get_name(), str(msg)))

    def logwarn(self, msg):
        rospy.logwarn('%s : %s'%(rospy.get_name(), str(msg)))

    def _play_tts(self, msg):
        if not self.is_connecting:
            self.logwarn("not connected to robot")
            return
        text = msg 
        speech_type = 1
        self.cmd_id = self.cmd_id + 1
        cmdset = CCmdSet("PlayTTS", "REQUEST_MESSAGE", self.NAME, "RBCODE", 0, "", 0, 0, "", self.cmd_id)
        cmdset.setString('Text', text)
        cmdset.setInt('SpeechType', speech_type)
        self.s.send(cmdset.getCmdSet())

    def _robot_motion(self, motion_name):
        if not self.is_connecting:
            self.logwarn("not connected to robot")
            return
        motion = motion_name 
        self.cmd_id = self.cmd_id + 1
        cmdset = CCmdSet("RobotMotion", "REQUEST_MESSAGE", self.NAME, "RBCODE", 0, "", 0, 0, "", self.cmd_id)
        cmdset.setString('MotionName', motion)
        self.s.send(cmdset.getCmdSet())

    def face_expression(self, args):
        if not self.is_connecting:
            self.logwarn("not connected to robot")
            return
        print "FaceExpression: %d" % args.data
        self.cmd_id = self.cmd_id + 1
        cmdset = CCmdSet("ReqExpressFace", "REQUEST_MESSAGE",
                         self.NAME, "RBCODE", 0, "", 0, 0, "", self.cmd_id)
        cmdset.setUInt('FaceType', args.data)
        self.s.send(cmdset.getCmdSet())

    def easy_move_arm(self, args):
        if not self.is_connecting:
            self.logwarn("not connected to robot")
            return
        print "EasyMoveArm: %d" % args.data
        self.cmd_id = self.cmd_id + 1
        cmdset = CCmdSet("EasyMoveArm", "REQUEST_MESSAGE",
                         self.NAME, "RBCODE", 0, "", 0, 0, "", self.cmd_id)
        cmdset.setUInt('TestCode', args.data)
        self.s.send(cmdset.getCmdSet())

    def easy_move_head(self, args):
        if not self.is_connecting:
            self.logwarn("not connected to robot")
            return
        print "EasyMoveHead: %d" % args.data
        self.cmd_id = self.cmd_id + 1
        cmdset = CCmdSet("EasyMoveHead", "REQUEST_MESSAGE",
                         self.NAME, "RBCODE", 0, "", 0, 0, "", self.cmd_id)
        cmdset.setUInt('TestCode', args.data)
        self.s.send(cmdset.getCmdSet())


    @abc.abstractmethod
    def init_publisher(self):
        pass

    @abc.abstractmethod
    def init_subscriber(self):
        pass

    @abc.abstractmethod
    def process_button_event(self, button_id, status):
        pass




    # Sample Function
    # def PlayTTS(self, IsRunning, Return,  Text, SpeechType, SyncFlag = False):
    # Todo
    #     ResultCode = 'failure'
    #     nTextLength = len(Text)
    #     nSpeechType = int(SpeechType)

    #     if nTextLength <= 0:
    #         ResultCode = "failure"
    #         return 'failure'

    #     elif nSpeechType< 0 or nSpeechType> 10:
    #         ResultCode = "failure"

    #     else:
    # Todo plat tts
    #         TTSText = 'Text: '+Text
    #         g_cmdID = 0;
    #         g_cmdID +=1
    #         cmdID = g_cmdID

    #         cmdset = CCmdSet("PlayTTS","REQUEST_MESSAGE",self.NAME,"RBCODE",0,"",0,0,"",g_cmdID)

    #         cmdset.setString('Text',Text)
    #         cmdset.setInt('SpeechType',SpeechType)
    #         self.s.send(cmdset.getCmdSet())

    # receive
    #         waitFlag = SyncFlag
    #         while waitFlag and IsRunning[0] :
    #             for res in self.res_cmdset_list:
    #                 if int(res.m_cmdID) == cmdID:
    #                     waitFlag = False

    #                     ArgList = i.getArgList()
    #                     for idx in range(len(ArgList)):
    #                         Return[ArgList[idx]] = cmdset.getValue(ArgList[idx])
    #                     self.res_cmdset_list.remove(res)

    #         ResultCode = "success"
    #     IsRunning[0] = False
    #     print ResultCode


