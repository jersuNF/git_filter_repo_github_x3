import subprocess
import time

import sys

from cobs import cobs
from rtt import RTT

SPAWN_JLINK = True

# TODO - Make this smarter with regards to existing jlink instances running
jlink_proc = None
if SPAWN_JLINK:
    JLINK_EXE = "JLinkExe"
    JLINK_EXE = "C:\\Program Files\\SEGGER\\JLink\\JLink.exe"
    #JLINK_EXE = "C:\\Program Files (x86)\\SEGGER\\JLink\\JLink.exe"

    cmd = [JLINK_EXE, '-device', 'nRF52840_xxAA', '-if', 'swd', '-speed', '20000', '-autoconnect', '1']
    jlink_proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

time.sleep(1)

rtt = RTT(19021)
rtt.write(b"$$SEGGER_TELNET_ConfigStr=RTTCh;2$$")

time.sleep(1)
rtt.read()

import signal

def signal_handler(sig, frame):
    if jlink_proc:
        jlink_proc.communicate(input=b'exit\n')
    rtt.close()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def send_diag_cmd(rtt, cmd, expect_data):
    rtt.write(cobs.encode(cmd) + b"\x00")

    start_time = time.time()
    got_resp = False
    data = b""

    while (not got_resp) and ((time.time()-start_time) < 1):
        recv = rtt.read()
        if len(recv) > 0:
            data += recv
            print("Raw-data: " + str(data))
            if len(data) >= 5:
                ind = data.find(b"\x00")
                if ind >= 0:
                    enc = data[:ind]
                    data = data[ind+1:]

                    print("COBS-data: " + str(enc))
                    resp = cobs.decode(enc)

                    print("Decoded data: " + str(resp))
                    got_resp = True

                    if expect_data:
                        return resp[4:]

    return got_resp

import struct

def write_serial(rtt, serial):
    cmd = b"N\x70\x00\x04" + struct.pack("<I", serial)
    print(cmd)
    if not send_diag_cmd(rtt, cmd, False):
        raise Exception("Failed writing serial")

def read_serial(rtt):
    resp = send_diag_cmd(rtt, b"N\x70\x00", True)
    print(resp)
    if not resp:
        raise Exception("Failed reading serial")
    return struct.unpack("<I", resp)[0]

write_serial(rtt, 11500)
print(read_serial(rtt))

rtt.close()
if jlink_proc:
    jlink_proc.communicate(input=b'exit\n')
sys.exit(0)