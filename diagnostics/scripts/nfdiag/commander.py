from cobs import cobs
import struct
from queue import Queue, Empty
import crc
import threading
import time

import logging
logging.basicConfig(level=logging.INFO)

class Commander(threading.Thread):
	def __init__(self, stream):
		threading.Thread.__init__(self)

		self.stream = stream
		crc16_conf = crc.Configuration(
			width=16,
			polynomial=0x1021,
			init_value=0x0000,
			final_xor_value=0x0000,
			reverse_input=True,
			reverse_output=True,
		)
		self.crc_calc = crc.CrcCalculator(crc16_conf)

		self.resp_queue = Queue()

		self.running = True
		self.daemon = True
		self.start()

	def __del__(self):
		self.stop()

	def stop(self):
		self.running = False
		self.join()

	def send_cmd(self, group, cmd, data=None):
		struct_format = "<BBH"
		raw_cmd = struct.pack(struct_format, group, cmd, 0)
		if not (data is None or len(data) == 0):
			raw_cmd += data

		checksum = self.crc_calc.calculate_checksum(raw_cmd)

		raw_cmd = bytearray(raw_cmd)

		raw_cmd[2] = checksum&0xFF
		raw_cmd[3] = (checksum>>8)&0xFF

		logging.debug("Sending: " + str(raw_cmd))

		self.stream.write(cobs.encode(raw_cmd) + b"\x00")

		return self.get_resp(group, cmd)

	def get_resp(self, group, cmd, timeout=0.5):
		resp = None
		end_time = time.time() + timeout
		while (not resp) and (time.time() < end_time):
			try:
				resp = self.resp_queue.get(timeout=end_time - time.time())

				if resp["group"] != group or resp["cmd"] != cmd:
					resp = None
			except:
				pass
		
		return resp

	def handle_resp(self, resp):
		logging.debug("Response: " + str(resp))

		resp_struct_format = "<BBBH"
		
		size = len(resp)
		if size < struct.Struct(resp_struct_format).size:
			logging.warning("Response too short")
			return
		
		data_size = size - struct.Struct(resp_struct_format).size
		data = None
		if data_size > 0:
			resp_struct_format += str(data_size) + "s"
			group, cmd, code, checksum, data = struct.unpack(resp_struct_format, resp)
		else:
			group, cmd, code, checksum = struct.unpack(resp_struct_format, resp)
		
		# Validate checksum
		resp = bytearray(resp)
		resp[3] = 0
		resp[4] = 0
		calc_checksum = self.crc_calc.calculate_checksum(resp)
		if checksum != calc_checksum:
			logging.warning("Invalid checksum")
			return
		
		logging.debug("Got response: ")
		logging.debug("    group=" + str(group))
		logging.debug("    cmd=" + str(cmd))
		logging.debug("    code=" + str(code))
		logging.debug("    data=" + str(data))

		response = {}
		response["group"] = group
		response["cmd"] = cmd
		response["code"] = code
		response["data"] = data

		self.resp_queue.put(response)


	def run(self):
		receive_buffer = b""

		while self.running:
			try:
				data = self.stream.read(1000)

				if len(data) > 0:
					receive_buffer += data

					# Decode COBS format data
					# Parse as long as 0's are found 
					found_zero = True
					while found_zero:
						ind = receive_buffer.find(b"\x00")
						found_zero = (ind >= 0)
						if found_zero:
							# Identified a COBS encoded packet, fetch and remove from buffer
							logging.debug("Received: " + str(receive_buffer))
							enc = data[:ind]
							receive_buffer = receive_buffer[ind+1:]

							logging.debug("COBS-data: " + str(enc))
							resp = cobs.decode(enc)
							logging.debug("Decoded data: " + str(resp))

							self.handle_resp(resp)
				else:
					time.sleep(0.001)
			except Exception as e:
				time.sleep(0.001)

class TestStream:
	def __init__(self):
		self.read_queue = Queue()
	
	def inject_read_data(self, data):
		self.read_queue.put(data)

	def read(self, max_size):
		data = None
		try:
			data = self.read_queue.get(block=False)
		except Empty:
			pass
		return data
	
	def write(self, data):
		logging.debug("Writing: " + str(data))

if __name__ == "__main__":
	stream = TestStream()
	cmndr = Commander(stream)

	cmndr.send_cmd(1, 0)
	stream.inject_read_data(b'\x02\x01\x04\x13b\xb0\x00')

	while True:
		time.sleep(1)