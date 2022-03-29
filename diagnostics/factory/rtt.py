import socket
#import fcntl
import os
import threading
import logging
import time

HOST = "127.0.0.1"

class RTT(threading.Thread):
    def __init__(self, port):
        threading.Thread.__init__(self)

        self.port = port

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((HOST, self.port))
        #fcntl.fcntl(self.s, fcntl.F_SETFL, os.O_NONBLOCK)
        self.s.setblocking(False)
        #s.sendall(b"Hello, world")

        self.running = True

        self.received_data = b""
        self.lock = threading.Lock()

        self.start()

    def __del__(self):
        self.close()

    def close(self):
        self.running = False
        self.join()
        self.s.close()

    def write(self, data):
        try:
            self.s.send(data)
        except Exception as e:
            logging.debug("Exception when writing to RTT: " + str(e))
    
    def read(self):
        self.lock.acquire()
        data = self.received_data
        self.received_data = b""
        self.lock.release()
        return data

    def _add_received_data(self, data):
        self.lock.acquire()
        self.received_data += data
        self.lock.release()

    def run(self):
        while self.running:
            try:
                data = self.s.recv(1024)

                if len(data) > 0:
                    self._add_received_data(data)
                    logging.debug(data.decode("utf-8"))
            except Exception as e:
                time.sleep(0.01)