#!/usr/bin/env python

"""
    ros_tcp_comm tcp_comm Node

    raw Authors: Nicholas McCullough and Joseph Neidigh

    author: Haijun Wang

    This node sends and receive messages across a lan or wireless network via a TCP connection.


    Currently this node receives location data from the turtlesim node. To
    send different data change the params inside the 'tcp_comm.launch' file.

"""

import socket
import sys
import struct
import zlib
import rospy
import rostopic
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from sensor_msgs.msg import Imu
from xunjian_nav.msg import Encoder, Ultrasound
from tf.transformations import euler_from_quaternion

HEADER=0xFA
IMU_ID=0x10
ENC_ID=0x11
ULTRA_ID=0x12
VEL_ID=0x13
RESET_ID=0x14
MAX_PACK_LEN=41
Q30=1073741824.0
connect_flag = True

class Tcp_comm():
    def __init__(self):
        rospy.init_node('tcp_comm')

        RECEIVER_IP = rospy.get_param('~ip')
        PORT = rospy.get_param('~port_number')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #socket connection
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)#reuse addr
            self.sock.bind(("", PORT))
            self.sock.listen(1)
            rospy.loginfo('waiting for connection')
            self.sock, addr = self.sock.accept()#block here and waite for connection
            rospy.loginfo("connectted")
        except Exception as e:
            self.sock.close()
            rospy.loginfo("CONNECTION ERROR")
            rospy.loginfo(e)
            sys.exit()

        rospy.Subscriber("smoother_cmd_vel", Twist, self.vel_callback, queue_size=1)
        rospy.Subscriber("encoder_reset", Empty, self.rst_callback, queue_size=1)

        enc_pub = rospy.Publisher("encoder", Encoder, queue_size=10)
        ut_pub = rospy.Publisher("range_dist", Ultrasound, queue_size=10)
        imu_pub = rospy.Publisher("imu",Imu,queue_size=10)

        enc_msg = Encoder()
        ut_msg = Ultrasound()
        imu_msg = Imu()
        global connect_flag
        while not rospy.is_shutdown():#receive data
            connect_flag = True
            try:
                self.sock.settimeout(5)#timeout 5s
                data=self.recv_msg()
                self.sock.settimeout(None)
                now = rospy.Time.now()
                if data is not None:
                    if data[0]==ENC_ID:
                        rospy.loginfo('recv enc data,%d,%d,%d,%d',data[1],data[2],data[3],data[4])
                        enc_msg.leftEncoder=data[1]
                        enc_msg.rightEncoder=data[2]
                        enc_msg.vx = data[3]
                        enc_msg.w = data[4]
                        enc_pub.publish(enc_msg)
                    elif data[0]==ULTRA_ID:
                        rospy.loginfo('recv ultra data')
                        ut_msg.ultra_1=data[1]
                        ut_msg.ultra_2=data[2]
                        ut_msg.ultra_3=data[3]
                        ut_msg.ultra_4=data[4]
                        ut_msg.ultra_5=data[5]
                        ut_msg.ultra_6=data[6]
                        ut_pub.publish(ut_msg)
                    elif data[0]==IMU_ID:
                        #rospy.loginfo('recv imu data')
                        imu_msg.header.frame_id = "imu"
                        imu_msg.header.stamp = now
                        imu_msg.angular_velocity.x=0#data[1]/65536.0*500.0/57.3
                        imu_msg.angular_velocity.y=0#data[2]/65536.0*500.0/57.3
                        imu_msg.angular_velocity.z=0#data[3]/65536.0*500.0/57.3
                        imu_msg.linear_acceleration.x=0#data[4]/65536.0*16.0*9.8
                        imu_msg.linear_acceleration.y=0#data[5]/65536.0*16.0*9.8
                        imu_msg.linear_acceleration.z=0#data[6]/65536.0*16.0*9.8
                        imu_msg.orientation.w=data[7]/Q30
                        imu_msg.orientation.x=data[8]/Q30
                        imu_msg.orientation.y=data[9]/Q30
                        imu_msg.orientation.z=data[10]/Q30
                        imu_msg.orientation_covariance=[0.25, 0, 0, 0, 0.25, 0, 0, 0, 0.25]
                        imu_msg.angular_velocity_covariance=[0.2, 0, 0, 0, 0.2, 0, 0, 0, 0.2]
                        imu_msg.linear_acceleration_covariance = [0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5]
                        imu_pub.publish(imu_msg)
                        (roll,pitch,yaw)=euler_from_quaternion([imu_msg.orientation.x,imu_msg.orientation.y,imu_msg.orientation.z,imu_msg.orientation.w])
                        rospy.loginfo("recv imu data:%d,%d,%d",(int)(roll*57.3),(int)(pitch*57.3),(int)(yaw*57.3))
                    else:
                        rospy.loginfo("error: unrecognized header,0x%x",data[0])
                else:
                    rospy.loginfo("error: data loss or connection lost")
                    connect_flag = False
                    self.sock.shutdown(2)
                    self.sock.close()
                    rospy.loginfo("waiting for reconnection")
                    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                    self.sock.bind(("", PORT))
                    self.sock.listen(1)
                    self.sock, addr = self.sock.accept()
                    rospy.loginfo("connectted")
                    connect_flag = True

            except socket.timeout:
                connect_flag = False
                self.sock.shutdown(2)
                self.sock.close()
                rospy.loginfo("socket timeout")
                rospy.loginfo("waiting for reconnection")
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                self.sock.bind(("", PORT))
                self.sock.listen(1)
                self.sock, addr = self.sock.accept()
                rospy.loginfo("connectted")
                connect_flag = True
        self.sock.close()
        rospy.spin()

    def vel_callback(self, msg):
        global connect_flag
        if connect_flag:
            data = [(int)(msg.linear.x*1000), (int)(msg.angular.z*1000)]#extract data from msg
            rospy.loginfo('set vel:%d,%d',data[0],data[1])
            pack = self.msg_pack(VEL_ID, data)#pack the data
            #print repr(pack)
            try:
                self.sock.sendall(pack)
            except Exception as e:
                connect_flag=False
                rospy.loginfo("SENDER ERROR")
                rospy.loginfo(e)

    def rst_callback(self, msg):
        global connect_flag
        if connect_flag:
            data = [0x01]#reset encoder
            pack = self.msg_pack(RESET_ID, data)
            try:
                self.sock.sendall(pack)
                rospy.loginfo('reset encoder')
            except Exception as e:
                connect_flag = False
                rospy.loginfo("SENDER ERROR")
                rospy.loginfo(e)

    def msg_pack(self, idd, data):
        pack = struct.pack('>B',HEADER)#pack header
        pack = pack+struct.pack('>B',len(data)*4+1) #pack len
        pack = pack + struct.pack('>B',idd) #pack header
        # pack data one by one
        for item in data:
            pack=pack+struct.pack('>i',item)
        pack = pack + struct.pack('>B',0xAA)
        return pack

    def recv_msg(self):
        #Read message length and unpack it into an integer
        header = struct.unpack('>B',self.recvall(1))[0]#first 1 data is the header
        if header != HEADER:
            rospy.loginfo('header error %d,%d',header,HEADER)
            return None
        raw_len= self.recvall(1)#next is len
        if not raw_len:
            rospy.loginfo('len = none')
            return None
        data_len = struct.unpack('>B',raw_len)[0]#len is unsigned byte
        # Read the message data
        #rospy.loginfo('recv pack_len:%d',data_len)
        if data_len<1 or data_len>MAX_PACK_LEN:
            aaa = self.recvall(4096)
            #bbb = struct.unpack('>B')
            rospy.loginfo('len error,%d,%d',header,data_len)
            return None
        pack = self.recvall(data_len)
        if not pack:
            rospy.loginfo('pack = none')
            return None
        data = struct.unpack('>B'+str((data_len-1)/4)+'i',pack)#a byte header + (lenbyte-1)/4 int
        chk= self.recvall(1)#last is chk
        return data

    def recvall(self, n):
        # Helper function to recv n bytes or return None if EOF is hit
        data = ''
        while len(data) < n:
            packet = self.sock.recv(n - len(data))
            if not packet:
                return None
            data += packet
        return data

if __name__ == "__main__":
    Tcp_comm()
