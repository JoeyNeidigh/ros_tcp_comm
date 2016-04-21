#!/usr/bin/env python

"""
    ros_tcp_comm Receiver Node

    Authors: Nicholas McCullough and Joseph Neidigh

    Faculty Advisor: Dr. Nathan Sprague

    Version 1.0

    This node receives messages across a wireless network via a TCP connection.
    It is intended to receive these messages from the corresponding 'Sender'
    node in this package. Instructions for how to customize this node to receive
    and publish a particular topic are as follows:

        Step 1: Import the appropriate message type. This should match the message type
                that is being used by the originating topic in 'Sender'.

        Step 2: Set up a publisher to publish the message after it has been received over
                the TCP connection. The second argument should match the message type 
                that was imported in Step 1.

        Step 3: Create a message of the type imported in Step 1.

        Step 4: Assign the received data to the appropriate field of the message created
                in Step 3. In this example the information being transfered across the
                network is the 'data' field of the std_msgs/String message.
"""

import socket
import pickle
import sys
import struct
import zlib
import rospy

# Step 1 ######################
from std_msgs.msg import String
###############################


class Receiver():
    def __init__(self):
        rospy.init_node('receiver')
        # Step 2 ########################################################
        receiver_pub = rospy.Publisher('/received_message_topic', String)
        #################################################################

        # port to receive messages on
        PORT = 13000

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

        # Step 3 ###############
        topic_message = String()
        ########################

        while True:
            compressed_message = self.recv_msg(sock)
            message = zlib.decompress(compressed_message)
            if not message is None:
                depickled_message = pickle.loads(message)
                # Step 4 #############################
                topic_message.data = depickled_message
                ######################################
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
