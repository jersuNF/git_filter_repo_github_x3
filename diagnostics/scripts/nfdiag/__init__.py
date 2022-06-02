from .commander import Commander
from .stream import BLEStream, JLinkStream

import time

class NFDiag:
	def __init__(self):
		self.stream = JLinkStream()
		#self.stream = BLEStream("COM4", serial=8010)
		while not self.stream.is_connected():
			time.sleep(0.1)

		self.cmndr = Commander(self.stream)
		print("Trying to ping...")
		start_time = time.time()
		while time.time() < (start_time + 5):
			resp = self.cmndr.send_cmd(0, 0x55)
			if resp:
				break
		
		self.cmndr.send_cmd(0x02, 0x10, b"\x03")

		while True:
			resp = self.cmndr.send_cmd(0x02, 0x12)
			if resp:
				if resp["data"]:
					print(resp["data"])
			else:
				time.sleep(0.01)

	def __del__(self):
		self.cmndr.stop()
		del self.cmndr
		del self.stream