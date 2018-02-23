# producer.py
import time
import zmq

def producer():
    context = zmq.Context()
    zmq_socket = context.socket(zmq.PUSH)
    zmq_socket.bind("tcp://127.0.0.1:5557")
    # Start your result manager and workers before you start your producers
    for num in range(20000):
        # eg. Read leap motion data and create a json `work_message`
        work_message = { 'num' : num }
        zmq_socket.send_json(work_message)

producer()