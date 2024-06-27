#!/usr/bin/python

import sys
import time
import math

sys.path.append('../lib/python/amd64')
import robot_interface as sdk

# high cmd
TARGET_PORT = 8082
LOCAL_PORT = 8081
TARGET_IP = "192.168.123.220"   # target IP address

HIGH_CMD_LENGTH = 113
HIGH_STATE_LENGTH = 244

class discrete_control:

    def __init__(self, cmd, state) -> None:
        
        self.state = state
        self.cmd = cmd

        self.cur_pos = cmd.position
        self.cur_rpy = state.imu.rpy

    def apply_action(self, control_type, distance, degree):
        if control_type == "i":
            self.move_forward(distance)
        elif control_type == "k":
            self.move_backward(distance)
        elif control_type == "j":
            self.turn_left(degree)
        elif control_type == "l":
            self.turn_right(degree)
        else:
            raise "Wrong input. please input 'i', 'j', 'k', 'l'..."

    def guodu(self):
        self.cmd.mode = 1
        return self.cmd

    def move_forward(self, distance):
        self.cmd.mode = 3
        self.cmd.position[2] = self.cur_pos[2] + distance

        return self.cmd
    
    def move_backward(self, distance):
        return self.move_forward(-distance)
    
    def turn_left(self, degree):
        self.cmd.mode = 3
        rot = degree * 3.1415 / 180
        self.cmd.rpy[2] = self.cur_rpy - rot
        return self.cmd

    def turn_right(self, degree):
        return self.turn_left(-degree)
    
    def update_cur_pos_rpy(self, distance = 0, degree = 0):

        rot = degree * 3.1415 / 180
        self.cur_rpy = self.state.rpy
        # self.cur_rpy += rot

        self.cur_pos += distance


if __name__ == '__main__':

    HIGHLEVEL = 0x00
    LOWLEVEL  = 0xff

    # udp = sdk.UDP(8080, "192.168.123.161", 8082, 129, 1087, False, sdk.RecvEnum.nonBlock)
    # udp = sdk.UDP(HIGHLEVEL, 8080, "192.168.123.161", 8082)

    cmd = sdk.HighCmd()
    state = sdk.HighState()

    udp = sdk.UDP(LOCAL_PORT, TARGET_IP, TARGET_PORT, HIGH_CMD_LENGTH, HIGH_STATE_LENGTH, -1)

    udp.InitCmdData(cmd)
    dc = discrete_control(cmd, state)

    motiontime = 0
    while True:
        time.sleep(0.002)
        motiontime = motiontime + 1

        # print(motiontime)
        # print(state.imu.rpy[0])        
        udp.Recv()
        udp.GetRecv(state)

        if(motiontime > 0 and motiontime < 1000):
            cmd = dc.guodu()
        
        if(motiontime > 1000 and motiontime < 5000):
            cmd = dc.apply_action('i', 0.5, 0)
        dc.update_cur_pos_rpy(0.5, 0)
        
        if(motiontime > 5000 and motiontime < 10000):
            cmd = dc.apply_action('k', -0.5, 0)
        dc.update_cur_pos_rpy(-0.5, 0)
        
        if(motiontime > 10000 and motiontime < 15000):
            cmd = dc.apply_action('j', 0, 90)
        dc.update_cur_pos_rpy(0, 90)
        
        if(motiontime > 15000 and motiontime < 20000):
            cmd = dc.apply_action('l', 0, -90)
        dc.update_cur_pos_rpy(0, -90)
            
        udp.SetSend(cmd)
        udp.Send()
