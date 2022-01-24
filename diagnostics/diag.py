import subprocess
import time


# TODO - Make this smarter with regards to existing jlink instances running
JLINK_EXE = "JLinkExe"
JLINK_EXE = "C:\\Program Files\\SEGGER\\JLink\\JLink.exe"
JLINK_EXE = "C:\\Program Files (x86)\\SEGGER\\JLink\\JLink.exe"

cmd = [JLINK_EXE, '-device', 'nRF52840_xxAA', '-if', 'swd', '-speed', '20000', '-autoconnect', '1']
jlink_proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

time.sleep(1)

from telnetlib import Telnet
tn = Telnet('127.0.0.1', 19021)
tn.write(b"$$SEGGER_TELNET_ConfigStr=RTTCh;2$$")

time.sleep(1)
tn.read_eager()
tn.read_eager()
tn.read_eager()
tn.read_eager()
# TODO - We just ignored the header, should we check header info? E.g. 
"""
SEGGER J-Link V7.58e - Real time terminal output
J-Link OB-SAM3U128-V2-NordicSemi compiled Dec  3 2021 15:41:28 V1.0, SN=683785781
Process: JLinkExe
"""

import msvcrt
import signal
import sys

def signal_handler(sig, frame):
    jlink_proc.communicate(input=b'exit\n')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

while True:
    if msvcrt.kbhit():
        down_data = msvcrt.getch()
        if down_data == b'\r':
            down_data = b'\n'
            
        print(down_data.decode("ascii"), end='', flush=True)

        tn.write(down_data)

    data = tn.read_eager()
    if len(data) > 0:
        print(data.decode('utf-8', 'ignore'), end='', flush=True)
    else:
        time.sleep(0.01)
