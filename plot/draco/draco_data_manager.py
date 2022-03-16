import zmq
import sys
import os

import time

import ruamel.yaml as yaml

cwd = os.getcwd()
sys.path.append(cwd + '/build')

from messages.draco_pb2 import *

context = zmq.Context()
socket = context.socket(zmq.SUB)

##YAML parse
with open("config/draco/pnc.yaml", "r") as stream:
    try:
        config = yaml.safe_load(stream)
        ip_address = config["ip_address"]
    except yaml.YAMLError as exc:
        print(exc)

socket.connect(ip_address)
socket.setsockopt_string(zmq.SUBSCRIBE, "")

msg = pnc_msg()

while True:
    ##receive msg trough socket
    encoded_msg = socket.recv()
    msg.ParseFromString(encoded_msg)

    ##print pnc msg
    print(msg)
