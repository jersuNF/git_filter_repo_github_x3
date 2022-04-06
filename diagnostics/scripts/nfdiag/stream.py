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
        print("Closing")
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
            print(f"No heart rate collector advertising with name {self.dev_name} found." + str(e))
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
        print("New connection: {}".format(conn_handle))
        self.conn_q.put(conn_handle)

    def on_gap_evt_disconnected(self, ble_driver, conn_handle, reason):
        print("Disconnected: {} {}".format(conn_handle, reason))

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
        print(
            "Received advertisment report, address: 0x{}, device_name: {}".format(
                address_string, dev_name
            )
        )

        if (dev_name == self.dev_name) or (self.wildcard and dev_name.startswith(self.dev_name)):
            self.adapter.connect(peer_addr, tag=1)

    def on_notification(self, ble_adapter, conn_handle, uuid, data):
        #if len(data) > 32:
        #    data = "({}...)".format(data[0:10])
        print("Connection: {}, {} = {}".format(conn_handle, uuid, data))

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

        print("Serial port used: {}".format(port))
        self.driver = BLEDriver(
            serial_port=port, auto_flash=False, baud_rate=1000000, log_severity_level="info"
        )

        self.adapter = BLEAdapter(self.driver)
        self.nfdiag = NFDiagnostics(self.adapter, "NF000674", wildcard=False)
        self.nfdiag.open()
    
        self.conn = self.nfdiag.connect_and_discover()
        if self.conn is None:
            raise Exception("Failed connecting to device")

    def __del__(self):
        self.close()

    def close(self):
        self.nfdiag.close()

    def read(self, size):
        data = None
        try:
            data = self.nfdiag.get_data()
        except:
            pass
        return data

    def write(self, data):
        self.nfdiag.send_data(self.conn, data)
