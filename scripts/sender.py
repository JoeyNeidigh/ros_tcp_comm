#!/usr/bin/env python

# rostopic pub /topic_to_send std_msgs/String "Hello World" -r 1

import socket
import pickle
import sys
import struct
import zlib
import rospy
###############################
from std_msgs.msg import String
###############################

class Sender():
    def __init__(self):
        rospy.init_node('sender')
        ####################################################################
        rospy.Subscriber('/topic_to_send', String, self.callback, queue_size=1)
        ####################################################################

        RECEIVER_IP = "127.0.0.1"
        PORT = 13000

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((RECEIVER_IP, PORT))
        except Exception as e:
            sock.close()
            rospy.loginfo("SENDER ERROR")
            rospy.loginfo(e)
            sys.exit()
        
        rospy.spin()

    def callback(self, topic_message):
        message = pickle.dumps(topic_message.data)
        compressed_message = zlib.compress(message)
        self.send_msg(compressed_message)

    def send_msg(self, msg):
        # Prefix each message with a 4-byte length (network byte order)
        msg = struct.pack('>I', len(msg)) + msg
        try:
            self.sock.sendall(msg)
        except Exception as e:
            self.sock.close()
            rospy.loginfo("SENDER ERROR")
            rospy.loginfo(e)
            sys.exit()

if __name__ == "__main__":
    Sender()
