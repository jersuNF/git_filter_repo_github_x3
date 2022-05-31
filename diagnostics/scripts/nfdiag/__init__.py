from .commander import Commander
from .stream import BLEStream, JLinkStream

import time

class NFDiag:
	def __init__(self):
		self.stream = JLinkStream() #BLEStream("/dev/ttyACM0", serial=674)
		while not self.stream.is_connected():
			time.sleep(0.1)
			
		self.cmndr = Commander(self.stream)
		for i in range(20):
			print(i)
			self.cmndr.send_cmd(0, 0x55) #, data=b"\x00")
			time.sleep(0.1)
		
		self.cmndr.send_cmd(0x02, 0x10, b"\x01")
		time.sleep(0.1)
		while True:
			self.cmndr.send_cmd(0x02, 0x12)
			time.sleep(0.05)

	def __del__(self):
		self.cmndr.stop()
		del self.cmndr
		del self.stream