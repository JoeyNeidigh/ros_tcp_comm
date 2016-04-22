#!/usr/bin/env python

"""
    ros_tcp_comm Receiver Node

    Authors: Nicholas McCullough and Joseph Neidigh

    Faculty Advisor: Dr. Nathan Sprague

    Version 1.0

    This node receives messages across a wireless network via a TCP connection.
    It is intended to receive these messages from the corresponding 'Sender'
    node in this package. As an example this node simply receives the pose
    from a turtlesim_node running on the sender and publishes it to a local 
    topic on this machine. To change the message_type to receive and the topic
    to publish to, simply change the params sent to this node in 'receiver.launch'.

    The recv_msg and recvall methods were adapted from this Stackoverflow post:
    http://stackoverflow.com/questions/17667903/python-socket-receive-large-amount-of-data

"""

import socket
import pickle
import sys
import struct
import zlib
import rospy
import rostopic

class Receiver():
    def __init__(self):
        rospy.init_node('receiver')
        PACKAGE = rospy.get_param('~package')
        NAME = rospy.get_param('~message_type')
        TOPIC = rospy.get_param('~topic_name')
        PORT = rospy.get_param('~port_number')
        
        MESSAGE = getattr(__import__(PACKAGE, fromlist=[NAME]), NAME)

        receiver_pub = rospy.Publisher(TOPIC, MESSAGE, queue_size=10)

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.bind(("", PORT))
            sock.listen(1)
            sock, addr = sock.accept()
        except Exception as e:
            sock.close()
            rospy.loginfo("RECEIVER ERROR:")
            rospy.loginfo(e)
            sys.exit() 

        while True:
            compressed_message = self.recv_msg(sock)
            message = zlib.decompress(compressed_message)
            if not message is None:
                depickled_message = pickle.loads(message)
                topic_message = depickled_message
                receiver_pub.publish(topic_message)

    def recv_msg(self, sock):
        # Read message length and unpack it into an integer
        raw_msglen = self.recvall(sock, 4)
        if not raw_msglen:
            return None
        msglen = struct.unpack('>I', raw_msglen)[0]
        # Read the message data
        return self.recvall(sock, msglen)

    def recvall(self, sock, n):
        # Helper function to recv n bytes or return None if EOF is hit
        data = ''
        while len(data) < n:
            packet = sock.recv(n - len(data))
            if not packet:
                return None
            data += packet
        return data

if __name__ == "__main__":
    Receiver()
