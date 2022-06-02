import sys
import time
import logging
from queue import Queue, Empty
from pc_ble_driver_py.observers import *

import logging
logging.basicConfig(level=logging.CRITICAL)

from cobs import cobs


class NFDiagnostics(BLEDriverObserver, BLEAdapterObserver):
    def __init__(self, adapter, dev_name="NF", wildcard=True):
        super(NFDiagnostics, self).__init__()
        self.adapter = adapter
        self.conn_q = Queue()
        self.rsp_q = Queue()
        self.adapter.observer_register(self)
        self.adapter.driver.observer_register(self)
        self.adapter.default_mtu = 250
        self.BASE_UUID = BLEUUIDBase([0x6e, 0x40, 0x00, 0x00, 0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc, 0xca, 0x9e])
        self.TX_UUID = BLEUUID(0x0003, self.BASE_UUID)
        self.RX_UUID = BLEUUID(0x0002, self.BASE_UUID)
        self.dev_name = dev_name
        self.wildcard = wildcard

    def open(self):
        self.adapter.driver.open()
        if config.__conn_ic_id__.upper() == "NRF51":
            self.adapter.driver.ble_enable(
                BLEEnableParams(
                    vs_uuid_count=1,
                    service_changed=0,
                    periph_conn_count=0,
                    central_conn_count=1,
                    central_sec_count=0,
                )
            )
        elif config.__conn_ic_id__.upper() == "NRF52":
            gatt_cfg = BLEConfigConnGatt()
            gatt_cfg.att_mtu = self.adapter.default_mtu
            gatt_cfg.tag = 1
            self.adapter.driver.ble_cfg_set(BLEConfig.conn_gatt, gatt_cfg)

            self.adapter.driver.ble_enable()

    def close(self):
        logging.debug("Closing")
        self.adapter.close()

    def connect_and_discover(self):
        scan_duration = 5
        params = BLEGapScanParams(interval_ms=200, window_ms=150, timeout_s=scan_duration)

        self.adapter.driver.ble_gap_scan_start(scan_params=params)

        try:
            new_conn = self.conn_q.get(timeout=scan_duration)
            self.adapter.service_discovery(new_conn)

            self.adapter.enable_notification(
                new_conn, self.TX_UUID
            )

            #self.adapter.enable_notification(new_conn, BLEUUID(BLEUUID.Standard.heart_rate))
            return new_conn
        except Exception as e:
            logging.debug(f"No nrfuart advertising with name {self.dev_name} found." + str(e))
            return None

    def send_data(self, conn, data):
        self.adapter.write_req(
            conn, self.RX_UUID, data
        )
    
    def get_data(self):
        return self.rsp_q.get(block=False)

    def on_gap_evt_connected(
        self, ble_driver, conn_handle, peer_addr, role, conn_params
    ):
        logging.debug("New connection: {}".format(conn_handle))
        self.conn_q.put(conn_handle)

    def on_gap_evt_disconnected(self, ble_driver, conn_handle, reason):
        logging.debug("Disconnected: {} {}".format(conn_handle, reason))

    def on_gap_evt_adv_report(
        self, ble_driver, conn_handle, peer_addr, rssi, adv_type, adv_data
    ):
        if BLEAdvData.Types.complete_local_name in adv_data.records:
            dev_name_list = adv_data.records[BLEAdvData.Types.complete_local_name]

        elif BLEAdvData.Types.short_local_name in adv_data.records:
            dev_name_list = adv_data.records[BLEAdvData.Types.short_local_name]

        else:
            return

        dev_name = "".join(chr(e) for e in dev_name_list)
        address_string = "".join("{0:02X}".format(b) for b in peer_addr.addr)
        logging.debug(
            "Received advertisment report, address: 0x{}, device_name: {}".format(
                address_string, dev_name
            )
        )

        if (dev_name == self.dev_name) or (self.wildcard and dev_name.startswith(self.dev_name)):
            self.adapter.connect(peer_addr, tag=1)

    def on_notification(self, ble_adapter, conn_handle, uuid, data):
        #if len(data) > 32:
        #    data = "({}...)".format(data[0:10])
        logging.debug("Connection: {}, {} = {}".format(conn_handle, uuid, data))

        self.rsp_q.put(bytes(data))

class BLEStream:
    def __init__(self, port, serial=None):
        global config, BLEDriver, BLEAdvData, BLEEvtID, BLEAdapter, BLEEnableParams, BLEGapTimeoutSrc, BLEUUID, BLEUUIDBase, BLEConfigCommon, BLEConfig, BLEConfigConnGatt, BLEGapScanParams
        from pc_ble_driver_py import config

        config.__conn_ic_id__ = "NRF52"
        # noinspection PyUnresolvedReferences
        from pc_ble_driver_py.ble_driver import (
            BLEDriver,
            BLEAdvData,
            BLEEvtID,
            BLEEnableParams,
            BLEGapTimeoutSrc,
            BLEUUID,
            BLEUUIDBase,
            BLEGapScanParams,
            BLEConfigCommon,
            BLEConfig,
            BLEConfigConnGatt,
        )

        # noinspection PyUnresolvedReferences
        from pc_ble_driver_py.ble_adapter import BLEAdapter

        global nrf_sd_ble_api_ver
        nrf_sd_ble_api_ver = config.sd_api_ver_get()

        logging.debug("Serial port used: {}".format(port))
        self.driver = BLEDriver(
            serial_port=port, auto_flash=False, baud_rate=1000000, log_severity_level="info"
        )

        self.adapter = BLEAdapter(self.driver)
        if serial is None:
            self.nfdiag = NFDiagnostics(self.adapter)
        else:
            self.nfdiag = NFDiagnostics(self.adapter, dev_name="NF" + str(serial).zfill(6), wildcard=False)
        self.nfdiag.open()
    
        self.conn = self.nfdiag.connect_and_discover()
        if self.conn is None:
            raise Exception("Failed connecting to device")
        else:
            self.connected = True

    def __del__(self):
        self.close()

    def close(self):
        self.nfdiag.close()

    def is_connected(self):
        return self.connected

    def read(self, size=0):
        data = None
        try:
            data = self.nfdiag.get_data()
        except:
            pass
        return data

    def write(self, data):
        self.nfdiag.send_data(self.conn, data)

import subprocess

import socket
#import fcntl
import os
import threading
import logging
import time


HOST = "127.0.0.1"
# TODO - Make this smarter with regards to existing jlink instances running
JLINK_EXE = "JLinkExe"
JLINK_EXE = "C:\\Program Files\\SEGGER\\JLink\\JLink.exe"
#JLINK_EXE = "C:\\Program Files (x86)\\SEGGER\\JLink\\JLink.exe"

class JLinkStream(threading.Thread):
    def __init__(self, serial=None):
        threading.Thread.__init__(self)

        self.jlink_proc = None
        if JLINK_EXE:
            cmd = [JLINK_EXE, '-device', 'nRF52840_xxAA', '-if', 'swd', '-speed', '4000', '-autoconnect', '1']
            if serial:
                cmd.append('-USB')
                cmd.append(serial)
            self.jlink_proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

        self.running = True
        self.daemon = True

        self.received_data = b""
        self.lock = threading.Lock()

        self.connected = False

        self.start()

    def __del__(self):
        self.close()

    def close(self):
        self.running = False
        self.join()
        self.s.close()
        if self.jlink_proc:
            self.jlink_proc.communicate(input=b'exit\n')

    def is_connected(self):
        return self.connected

    def write(self, data):
        try:
            self.s.send(data)
        except Exception as e:
            logging.debug("Exception when writing to RTT: " + str(e))
    
    def read(self, size=0):
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
        print("Connecting to JLink RTT")
        
        while not self.connected:
            try:
                self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.s.connect((HOST, 19021))
                self.s.setblocking(False)
                
                # Need to send enabling string immediately, and wait for prompt
                self.write(b"$$SEGGER_TELNET_ConfigStr=RTTCh;2$$")
                prompt = b""
                timeout = time.time() + 2
                while not ((b"Process: " in prompt) and prompt.endswith(b"\r\n")):
                    try:
                        data = self.s.recv(1024)
                        if len(data) > 0:
                            prompt += data
                    except:
                        pass
                    
                    if time.time() > timeout:
                        raise Exception("Timed out..")

                self.connected = True
            except:
                time.sleep(0.1)

        print("Connected")

        while self.running:
            try:
                data = self.s.recv(1024)

                if len(data) > 0:
                    self._add_received_data(data)
                    logging.debug(data.decode("utf-8"))
            except Exception as e:
                time.sleep(0.001)