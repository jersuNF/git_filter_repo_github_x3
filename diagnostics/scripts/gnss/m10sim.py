from pyubx2 import UBXReader, UBXMessage, GET, SET, POLL
from queue import Queue, Empty
import threading
import struct
import time

UBX_SYNC_CHAR_1 = 0xB5
UBX_SYNC_CHAR_2 = 0x62

UBX_MIN_PACKET = 8

UBX_CFG_UART1_BAUDRATE = 0x40520001
UBX_CFG_UART1OUTPRO_NMEA = 0x10740002
UBX_CFG_MSGOUT_UBX_NAV_PVT_UART1 = 0x20910007
UBX_CFG_MSGOUT_UBX_NAV_DOP_UART1 = 0x20910039
UBX_CFG_MSGOUT_UBX_NAV_STATUS_UART1 = 0x2091001b
UBX_CFG_MSGOUT_UBX_NAV_SAT_UART1 = 0x20910016
UBX_CFG_NAVSPG_ACKAIDING = 0x10110025

UBX_ACK = 0x05
UBX_ACK_ACK = 0x01
UBX_ACK_NAK = 0x00

UBX_CFG = 0x06
UBX_CFG_RST = 0x04
UBX_CFG_VALDEL = 0x8C
UBX_CFG_VALGET = 0x8B
UBX_CFG_VALSET = 0x8A

UBX_MGA = 0x13
UBX_MGA_ANO = 0x20
UBX_MGA_ACK = 0x60

UBX_NAV = 0x01
UBX_NAV_DOP = 0x04
UBX_NAV_PVT = 0x07
UBX_NAV_STATUS = 0x03

class M10Simulator(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

        self.parse_buffer = b""

        self.out_data = Queue()

        self._ubx_cfg_reset()
        
        self.running = True
        self.daemon = True
        self.start()

    def __del__(self):
        self.stop()

    def stop(self):
        self.running = False
        self.join()

    def run(self):
        while self.running:
            try:
                time.sleep(1)
            except:
                pass
    
    def send_data(self, data):
        self.out_data.put(data)

    def get_data(self):
        try:
            data = self.out_data.get(block=False)
            return data
        except Empty:
            pass
        
        return None


    def parse_command(self, data):
        if len(data) == 0:
            return

        self.parse_buffer += data

        is_parsing = True
        while is_parsing:
            if len(self.parse_buffer) >= UBX_MIN_PACKET:
                if self.parse_buffer[0] == UBX_SYNC_CHAR_1:
                    if self.parse_buffer[1] == UBX_SYNC_CHAR_2:
                        length = self.parse_buffer[4] + (self.parse_buffer[5]<<8)

                        if len(self.parse_buffer) < (UBX_MIN_PACKET + length):
                            # Need more data
                            is_parsing = False
                        else:
                            (ck_a, ck_b) = M10Simulator._calc_chk(self.parse_buffer[2:2+4+length])
                            
                            if (self.parse_buffer[6+length] == ck_a) and (self.parse_buffer[6+length+1] == ck_b):
                                self._ubx_handler(self.parse_buffer[:UBX_MIN_PACKET+length])

                                self.parse_buffer = self.parse_buffer[UBX_MIN_PACKET + length:]
                            else:
                                # Failed checksum
                                self.parse_buffer = self.parse_buffer[2:]
                    else:
                        # Invalid sync char
                        self.parse_buffer = self.parse_buffer[2:]
                else:
                    # Invalid sync char
                    self.parse_buffer = self.parse_buffer[1:]
            else:
                # Need more data
                is_parsing = False
    
    def _ubx_handler(self, packet):
        parsed_message = None

        try:
            msg = UBXReader.parse(packet, msgmode=SET)
            parsed_message = msg
        except:
            pass

        if not parsed_message:
            try:
                msg = UBXReader.parse(packet, msgmode=POLL)
                parsed_message = msg
            except:
                pass
        
        if parsed_message.identity == "CFG-RST":
            self._ubx_cfg_reset()
        elif parsed_message.identity == "CFG-VALGET":
            self._ubx_cfg_valget(parsed_message.keys_01, parsed_message.layer, parsed_message.position)
        elif parser_message.identity == "CFG-VALSET":
            print(parsed_message)
        else:
            print(parsed_message)

    def _ubx_cfg_reset(self):
        print("RESET")
        # Reset internal values
        self.baudrate = 38400
        self.rate = 1000
        # No response expected
        return
    
    def _ubx_cfg_valget(self, key, layer, position):
        print("VALGET")

        binary_resp = b""
        
        found_key = False
        if key == UBX_CFG_UART1_BAUDRATE:
            data = UBXMessage('CFG','CFG-VALGET', GET, version=1, layer=layer, position=position, payload=struct.pack("<I", self.baudrate))
            binary_resp += data.serialize()
            
            found_key = True

        data = UBXMessage('ACK', 'ACK-ACK' if found_key else 'ACK-NAK', GET, clsID=UBX_CFG, msgID=UBX_CFG_VALGET)
        binary_resp += data.serialize()

        self.send_data(binary_resp)

    def _calc_chk(data):
        ck_a = 0
        ck_b = 0
        for i in range(len(data)):
            ck_a = (ck_a + data[i]) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        
        return (ck_a, ck_b)
