#!/usr/bin/env python
# -*- coding:utf-8 -*-
#
# License: BSD
#

##############################################################################
# Imports
##############################################################################
import rospy
import time

#client program
##############################################################################
import socket
import threading
import struct

from cmdset import CCmdSet

class RobosemBridge():
    def __init__(self, name = "ROCONBRIDGE", host = '192.168.10.111', port = '6001'):
        self.NAME = "ROCONBRIDGE" 
        self.HOST = "192.168.10.111" 
        self.PORT = 6001
        self.TIME_OUT_SOCKET = 5 
        self.DISCRIMINATOR = 2503011588
        self.VERSION = 0x00020003
        self.is_connecting = False
        self.cmd_id = 0
        self.res_cmdset_list = []
        self.evt_cmdset_list = []

        self.connect()
        self.init_robosem()

    def connect(self):
        socket.setdefaulttimeout(self.TIME_OUT_SOCKET)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
               
        try:
            self.s.connect((self.HOST, self.PORT))
        except socket.timeout, e:
            rospy.logwarn('failed socket timeout. Reason: %s' % str(e))
            self.is_connecting = False
        except Exception, e:
            rospy.logwarn('failed. Reason:%s' % str(e))
            self.is_connecting = False
        else:
            self.is_connecting = True

    def init_robosem(self):
        if self.is_connecting:
            self.cmd_id += 1
            cmdset = CCmdSet("WhoIam","REQUEST_MESSAGE",self.NAME,"RVM",0," ",0,0," ",self.cmd_id)
            self.s.send(cmdset.getCmdSet())

    def recv(self):
        print "Start Recv Thread"
        while not rospy.is_shutdown() and self.is_connecting:
            try:
                data = self.s.recv(12)
                if len(data) != 12:
                    data += self.s.recv(12-len(data))
            except Exception, e:
                rospy.logwarn('failed. data = s.recv(12) Reason:%s' % str(e))
                continue
            
            try:
                buffer = struct.unpack('I I I',data)
            except Exception:
                print "Exception!!!!!!!!!!!!!!!!!!!!!buffer = struct.unpack('I I I',data)"
                continue

            disc = buffer[0]
            ver = buffer[1]
            data_length= buffer[2]
           
            if disc != self.DISCRIMINATOR or ver != self.VERSION or data_length == 0: 
                print "[Wrong Data ][nDisc: %d][nVer: %d][nDataLen: %d]"%(disc, ver, data_length)
                continue

            try:
                data = self.s.recv(data_length)
                if len(data) != data_length:
                    print 'More Receive Data'
                    data += self.s.recv(data_length-len(data))
            except Exception:
                print "Exception!!!!!!!!!!!!!!!!!!!!!data = s.recv(data_length)"
                continue
            
            try:
                buffer = struct.unpack('%ds'%data_length,data)
            except Exception:
                print "Exception!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!buffer = struct.unpack()"
                continue
            
            data = buffer[0]
            cmdset = CCmdSet()
            cmdset.parse(data)

            print "MSGT: [", cmdset.m_msgtype, " ] Name: [",cmdset.m_cmdname,"] REC: [",cmdset.m_receiverID,"] SEND: [",cmdset.m_senderID,"] CMD: [",cmdset.m_cmdID,"]"
            if cmdset.m_msgtype == "RESPONSE_MESSAGE":
                self.res_cmdset_list.append(cmdset)
            elif cmdset.m_msgtype == "EVENT_MESSAGE":
                self.event_proc(cmdset)
    
    def event_proc(self, cmdset):
        if cmdset.m_cmdname == "UserEvent":
            text = ""
            for param in cmdset.m_params:
                if param['name'] == "Param1":
                    text = param['value'] 
            
            m_IsRunning = [False]
            Ret = {}
            self.PlayTTS(m_IsRunning,Ret, text,1, False)

    def PlayTTS(self, IsRunning, Return,  Text, SpeechType, SyncFlag = False):
        #Todo
        ResultCode = 'failure'
        nTextLength = len(Text)
        nSpeechType = int(SpeechType)
        
        if nTextLength <= 0:
            ResultCode = "failure"
            return 'failure'
        
        elif nSpeechType< 0 or nSpeechType> 10:
            ResultCode = "failure"
            
        else:
            #Todo plat tts
            TTSText = 'Text: '+Text
            g_cmdID = 0;
            g_cmdID +=1
            cmdID = g_cmdID

            cmdset = CCmdSet("PlayTTS","REQUEST_MESSAGE",self.NAME,"RBCODE",0,"",0,0,"",g_cmdID)

            cmdset.setString('Text',Text)
            cmdset.setInt('SpeechType',SpeechType)
            self.s.send(cmdset.getCmdSet())

            #receive   
            waitFlag = SyncFlag
            while waitFlag and IsRunning[0] :
                for res in self.res_cmdset_list:
                    if int(res.m_cmdID) == cmdID:
                        waitFlag = False        

                        ArgList = i.getArgList()
                        for idx in range(len(ArgList)):
                            Return[ArgList[idx]] = cmdset.getValue(ArgList[idx])                
                        self.res_cmdset_list.remove(res)
            
            ResultCode = "success"
        IsRunning[0] = False
        print ResultCode


#######################################################################################################
##Main
####################################################################################################### 

# main
def main():
    print "============================Main start============================"
    
    rospy.init_node('robosem_bridge')
    rb = RobosemBridge()
    rb.recv()

    #m_IsRunning = [False]
    #Ret = {}
    #PlayTTS(m_IsRunning,Ret, u'\xeb\xb0\x94\xeb\xb3\xb4',1, False)
    #PlayTTS(m_IsRunning,Ret, "바보",1, False)
    #a = "바보"
    #a = u"\xeb\xb0\x94\xeb\xb3\xb4"
    #print a.encode('utf-8')
    
    #Connect()
    #rospy.init_node('robosem_bridge')
    #Recv()

    print "======================Main end================================="

if __name__ == '__main__':
    main()