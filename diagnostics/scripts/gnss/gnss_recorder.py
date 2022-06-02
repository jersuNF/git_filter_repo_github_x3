import sys
import os
base_path = os.path.dirname(os.path.abspath(__file__))
lib_path = os.path.join(base_path, "..")
sys.path.append(lib_path)

import nfdiag
import signal
import time
import argparse

# Parse input arguments
parser = argparse.ArgumentParser(description='Nofence GNSS recorder')
parser.add_argument('--file', help='Base of filename to log to. Name will be postfixed with [time].gnss')
parser.add_argument('--ble', help='Serial number of device in advertised name of device for BLE communication')
parser.add_argument('--rtt', help='Serial number of Segger J-Link to use for RTT communication')
args = parser.parse_args()

# Prepare log file
base_name = args.file
if not base_name:
	base_name = "gnss"
gnss_file = open(base_name + str(int(time.time())) + ".gnss", "wb")

# Build connection
if (args.ble and args.rtt):
	raise Exception("Can't connect to both BLE and RTT. Choose one!")
elif (not args.ble) and (not args.rtt):
	print("Didn't specify a connection. RTT will be used on any available J-Link devices.")

stream = None
if args.ble:
	stream = nfdiag.BLEStream("COM4", serial=args.ble)
else:
	stream = nfdiag.JLinkStream(serial=args.rtt)

while not stream.is_connected():
	time.sleep(0.1)

cmndr = nfdiag.Commander(stream)

# Signal handler for stopping with CTRL+C
def signal_handler(sig, frame):
	global cmndr
	global stream
	global gnss_file

	cmndr.stop()

	del cmndr
	del stream
	del gnss_file

	sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

# Sending ping to verify connection
print("Trying to ping...")
start_time = time.time()
got_ping = False
while time.time() < (start_time + 5) and (not got_ping):
	resp = cmndr.send_cmd(0, 0x55)
	if resp:
		got_ping = True
		print("Got ping")
if not got_ping:
	raise Exception("Did not get ping...")

# Setting sniffer mode in GNSS hub
cmndr.send_cmd(0x02, 0x10, b"\x01")

# Loop for collecting and writing data for GNSS
try:
	while True:
		resp = cmndr.send_cmd(0x02, 0x12)
		if resp:
			if resp["data"]:
				gnss_file.write(resp["data"])
				print(resp["data"])
		else:
			time.sleep(0.01)
except:
	pass