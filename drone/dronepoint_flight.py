import os
import sys
import logging
import threading

# Add parent dir to PATH to import messaging_lib and config_lib
current_dir = (os.path.dirname(os.path.realpath(__file__)))
lib_dir = os.path.realpath(os.path.join(current_dir, '../lib'))
sys.path.insert(0, lib_dir)

import messaging
import modules.client_core as client_core

logging.basicConfig(
    level=logging.DEBUG,  # INFO
    stream=sys.stdout,
    format="%(asctime)s [%(name)-7.7s] [%(threadName)-12.12s] [%(levelname)-5.5s]  %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
    ])


@messaging.message_callback("dronepoint")
def dronepoint_callback(connection, dronepoint_id=None, container_id=None):
    # this function only called after receiving "dronepoint" message from the server
    # dronepoint_id and container_id are ints
    logging.info("Dronepoint_id: {}; Container_id: {}".format(dronepoint_id, container_id))
    # your code goes here


if __name__ == "__main__":
    # logging.basicConfig(level=logging.DEBUG)
    client = client_core.Client()
    client.start()  # this function WILL block until exit! but you can put this call into thread
    print("Client exited")
