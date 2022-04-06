from .commander import Commander
from .stream import BLEStream

class NFDiag:
	def __init__(self):
		self.stream = BLEStream("/dev/ttyACM0", serial=674)
		self.cmndr = Commander(self.stream)

		self.cmndr.send_cmd(1, 0, data=b"\x00")

import time
if __name__ == "__main__":
	nfdiag = NFDiag()

	while True:
		time.sleep(1)