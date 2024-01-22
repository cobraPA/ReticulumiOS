# MIT License
#
# Copyright (c) 2016-2022 Mark Qvist / unsigned.io
# Copyright (c) 2023 Kevin Brosius - iOS additions from Android version
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

## from RNS.Interfaces.Interface import Interface
#from RNS.Interfaces import Interface

print('start RNODE module')
dir()

# builds, doesn't recognize Interface type
#import RNS.Interfaces.Interface
from RNS.Interfaces.Interface import Interface
#from RNS.Interfaces.Interface import RNSInterface
# fails .. wrong directory
#from .Interface import Interface

# from kivy_swift11.kivy_swift10_sideband_src.sideband_markqvist.sbapp.RNS.Interfaces import Interface
#from RNS.Interfaces import Interface
from time import sleep
#import sys
import threading
import time

## updateRNS
import math

import RNS
# failing to load RNodeInterface , Interface not found 
## from RNS.Interfaces.Interface import Interface
# App ends when core.py also has this one,
# else ends when core has it and we comment this out
# import RNS.Interfaces.Interface as Interface
###import RNS.Interfaces.Interface
# from RNS.Interfaces.Interface import Interface as RNSIface
# App ended
# from kivy_swift11.kivy_swift10_sideband_src.sideband_markqvist.sbapp.RNS.Interfaces.Interface import Interface
# app ends
# from RNS.Interfaces import Interface
# app ends
# from kivy_swift11.kivy_swift10_sideband_src.sideband_markqvist.sbapp.RNS.Interfaces import Interface

##from PyCoreBluetooth.wrappers.corebluetooth import CBCentralManager, CBPeripheral, CBService, CBUUID
from corebluetooth import CBCentralManager, CBPeripheral, CBService, CBUUID
from corebluetooth import PyCBPeripheralDelegate, CBL2CAPChannel, CBCharacteristic, CBDescriptor

from typing import Optional

#import dispatch_ios
from dispatch_wrap import DispatchSerialSwift

#from kivy.event import EventDispatcher,Observable
#from kivy.event import EventDispatcher
#from kivy.properties import ListProperty, NumericProperty, StringProperty, ObjectProperty

#import corebluetooth
#import gooblegook

class KISS():
    FEND            = 0xC0
    FESC            = 0xDB
    TFEND           = 0xDC
    TFESC           = 0xDD
    
    CMD_UNKNOWN     = 0xFE
    CMD_DATA        = 0x00
    CMD_FREQUENCY   = 0x01
    CMD_BANDWIDTH   = 0x02
    CMD_TXPOWER     = 0x03
    CMD_SF          = 0x04
    CMD_CR          = 0x05
    CMD_RADIO_STATE = 0x06
    CMD_RADIO_LOCK  = 0x07
    CMD_ST_ALOCK    = 0x0B
    CMD_LT_ALOCK    = 0x0C
    CMD_DETECT      = 0x08
    CMD_LEAVE       = 0x0A
    CMD_READY       = 0x0F
    CMD_STAT_RX     = 0x21
    CMD_STAT_TX     = 0x22
    CMD_STAT_RSSI   = 0x23
    CMD_STAT_SNR    = 0x24
    CMD_STAT_CHTM   = 0x25
    CMD_STAT_PHYPRM = 0x26
    CMD_BLINK       = 0x30
    CMD_RANDOM      = 0x40
    CMD_FB_EXT      = 0x41
    CMD_FB_READ     = 0x42
    CMD_FB_WRITE    = 0x43
    CMD_BT_CTRL     = 0x46
    CMD_PLATFORM    = 0x48
    CMD_MCU         = 0x49
    CMD_FW_VERSION  = 0x50
    CMD_ROM_READ    = 0x51
    CMD_RESET       = 0x55

    DETECT_REQ      = 0x73
    DETECT_RESP     = 0x46
    
    RADIO_STATE_OFF = 0x00
    RADIO_STATE_ON  = 0x01
    RADIO_STATE_ASK = 0xFF
    
    CMD_ERROR           = 0x90
    ERROR_INITRADIO     = 0x01
    ERROR_TXFAILED      = 0x02
    ERROR_EEPROM_LOCKED = 0x03
    ERROR_INVALID_FIRMWARE = 0x10

    PLATFORM_AVR   = 0x90
    PLATFORM_ESP32 = 0x80

    @staticmethod
    def escape(data):
        data = data.replace(bytes([0xdb]), bytes([0xdb, 0xdd]))
        data = data.replace(bytes([0xc0]), bytes([0xdb, 0xdc]))
        return data



###################
#class CBDeviceDataModel(EventDispatcher):
class CBDeviceDataModel():
    
    #delegate: PyCBPeripheralDelegate # wrap of class in swift that has CBPeripheralDelegate
    delegate = None
    #current_battery_level = NumericProperty(0)
    #current_label = StringProperty('')
    #current_rssi = NumericProperty(0)
    charTX = None
    charRX = None
    interfaceReady = False
    in_queue = None

    def __init__(self) -> None:
        #super(CBDeviceDataModel, self).__init__()
        
        self.delegate = PyCBPeripheralDelegate(self)
        #self.delegate.py_callback = self

        self.in_queue = []
        
    


    def didDiscoverServices(self, peripheral: CBPeripheral, error: Optional[str]):
        print('Py CBDeviceDataModel : didDiscoverServices')
        # We transmit to the devices' RX
        self.charTX_UUID = CBUUID('BEB5483E-36E1-4688-B7F5-EA07361BD01D')
        self.charRX_UUID = CBUUID('BEB5483E-36E1-4688-B7F5-EA07361BD02D')
        for service in peripheral.services:
            uuid = service.uuid
            service_name = str(uuid) # uses the __str__ from the swift class (.description)
            print(service_name)
            if service_name == "Battery":
                print(f"didDiscoverServices: {service_name} / {uuid.uuidString}")
                peripheral.discoverCharacteristics(None,service)
            else:
                print('request discover char 1')
                # works!
                # peripheral.discoverCharacteristics( [self.charRX_UUID, self.charTX_UUID], service)
                # worked!
                #peripheral.discoverCharacteristics( [self.charRX_UUID], service)
                peripheral.discoverCharacteristics( [self.charTX_UUID, self.charRX_UUID], service)
                print('request discover char 2')

    def didDiscoverCharacteristics(self, peripheral: CBPeripheral, service: CBService, error: Optional[str]):
        print('Discover char!')
        #called when device asks for services
        characteristics = service.characteristics
        #print(f"didDiscoverCharacteristics:", characteristics)

        for characteristic in characteristics:
            #print('1')
            
            # fails, must call existing var/method like __hash__()
            #uuid1 = characteristic.uuid
            #uuid_name = str(uuid1)

            # working
            #print('Discover char -', str(characteristic))
            #print('1.1')
            #print(type(characteristic))
            #print(type(characteristic.__hash__))
            #print(str(characteristic.__hash__()))
            #print(str(self.charTX_UUID.__hash__()))
            #print('1.2')
            #print(2)

            # works!
            #peripheral.setNotifyValue(True, characteristic)
            #peripheral.readCharacteristic(characteristic)

            if characteristic.__hash__() == self.charTX_UUID.__hash__():
                # Found TX
                self.charTX = characteristic 

                # works - write data
                #print('writing')
                #peripheral.writeValue( bytes('Greetings!', 'utf-8'), characteristic, 0)
                #print('write sent')
            elif characteristic.__hash__() == self.charRX_UUID.__hash__():
                # Found RX
                self.charRX = characteristic
                
                print('Found RX char - try notify')
                peripheral.setNotifyValue(True, characteristic)
                #peripheral.readCharacteristic(characteristic)
                #peripheral.setNotifyValue(1, characteristic)
                ##peripheral.notifyTrue(characteristic)

                print('Found RX char - 2')
            else:
                # Found unknown char
                print('Unexpected iOS RNode BLE characteristic! RX exp', str(self.charRX_UUID.uuid), ' found: ', str(characteristic))
                
            if (not self.charRX == None ) and (not self.charTX == None):
                self.interfaceReady = True
                print('RNode BLE comm available')
            else:
                self.interfaceReady = False
                print('RNode BLE comm not ready')

    def didUpdateValueForCharacteristic(self, peripheral: CBPeripheral, characteristic: CBCharacteristic, error: Optional[str]):
        #print('char update', characteristic)
        #print('data', type(characteristic.value))
        #print('data', characteristic.value)
        
        ##print('incoming', characteristic, 'data len', len(characteristic.value))
        self.in_queue.append(characteristic.value )
        #print('appended!')
        
#        ...
##        value: bytes = characteristic.value
 #       if value:
 #           self.current_battery_level = value[0]
 #           self.current_label = f"{peripheral.name}'s battery level is {value[0]}%"
 #           print(self.current_label)

    def peripheralDidUpdateName(self, peripheral: CBPeripheral):
        ...

    def peripheralIsReady(self, peripheral: CBPeripheral):
        ...

    def didDiscoverDescriptors(self, peripheral: CBPeripheral, characteristic: CBCharacteristic, error: Optional[str]):
        ...

    def didDiscoverIncludedServices(self, peripheral: CBPeripheral, service: CBService, error: Optional[str]):
        print("Service!")
        ...

    def didReadRSSI(self, peripheral: CBPeripheral, RSSI: int, error: Optional[str]):
        print("didReadRSSI", peripheral.name, RSSI)
  #      self.current_rssi = RSSI
        


    def didOpenChannel(self, peripheral: CBPeripheral, channel: Optional[CBL2CAPChannel], error: Optional[str]):
        ...

    def didModifyServices(self, peripheral: CBPeripheral, invalidatedServices: list[CBService]):
        print('modify service')
        ...

    def didUpdateNotificationState(self, peripheral: CBPeripheral, characteristic: CBCharacteristic, error: Optional[str]):
        print("didUpdateNotificationState", characteristic, error)

    def didWriteValueForCharacteristic(self, peripheral: CBPeripheral, characteristic: CBCharacteristic, error: Optional[str]):
        ##print("didWriteValueForCharacteristic", characteristic, "data", characteristic.value, "error", error)
        ...

    def didWriteValueForDescriptor(self, peripheral: CBPeripheral, descriptor: CBDescriptor, error: Optional[str]):
        ...

    def didUpdateValueForDescriptor(self, peripheral: CBPeripheral, descriptor: CBDescriptor, error: Optional[str]):
        print("didUpdateValueForDescriptor", descriptor)




class iOSBluetoothManager():
    
    # kills all
    # manager: CBCentralManager
    manager = None
    # kills aoo
    # current_device_data: CBDeviceDataModel
    current_device_data = None
    # kills aoo
    # periphals: list[CBPeripheral] = ListProperty([])
    
    _devices: dict[int,any] = {}
    
    connected_peripheral = None

    ##__events__ = ["on_peripheral"]

    def __init__(self, owner, target_device_name = None, target_device_address = None):
        # corebluetooth
        manager = CBCentralManager()
        self.manager = manager
        manager.py_callback = self
        self.current_device_data = CBDeviceDataModel()
        #end corebluetooth
        
        #from jnius import autoclass
        self.owner = owner
        self.connected = False
        self.target_device_name = target_device_name
        self.target_device_address = target_device_address
        self.potential_remote_devices = []
        self.rfcomm_socket = None
        self.connected_device = None
        self.connection_failed = False
        #self.bt_adapter = autoclass('android.bluetooth.BluetoothAdapter')
        #self.bt_device  = autoclass('android.bluetooth.BluetoothDevice')
        #self.bt_socket  = autoclass('android.bluetooth.BluetoothSocket')
        #self.bt_rfcomm_service_record = autoclass('java.util.UUID').fromString("00001101-0000-1000-8000-00805F9B34FB")
        #self.buffered_input_stream    = autoclass('java.io.BufferedInputStream')

        print (' iOS BLE manager init() target: ' + (target_device_name if target_device_name else 'n/a') )

    def connect(self, device_address=None):
        #self.rfcomm_socket = self.remote_device.createRfcommSocketToServiceRecord(self.bt_rfcomm_service_record)
        pass
    
    
    ###
    # corebluetooth
    
        #callback


    def didDiscover(self, peripheral: CBPeripheral, rssi: int):
        # prints for each periph
        # print('didDiscover - rssi')
        print("per: ", str(peripheral), " name: ", str(peripheral.name), " rssi: ", str(rssi), " id ", peripheral.identifier)

####        self.dispatch("on_peripheral",True,peripheral,rssi)

        dev_id = peripheral.identifier
#        matcher = '5D3FF136'
#        matcher = 'F95C5177'
# Lilygo
#        matcher = 'DE742A6F'
# custom  RNode - Heltec      - RNode 1 from iPhone15P
# connector into display has no numbers printed

    ###    matcher = '469EA203'

# same Heltec device when seen by iPadPro6
# RNode  name:  RNode  rssi:  -62  id  178D096E-14E1-60A4-DD6E-3783E4CC5C45

# heltec dev 2
# on black tape at display 'BH210504'
# per:  RNode  name:  RNode  rssi:  -64  id  1B8BCDD0-F47A-9A3F-1684-FFE11351D8BF 
# 
# after merge, update to RNode firmware 1.69, 
# RNode  name:  RNode  rssi:  -64  id  98048F8A-153E-0871-17A3-F27EE80BD13E

        #matcher = '1B8BCDD0'
        matcher = '98048F8A'
        
        if True:
        #if state:
            if matcher in dev_id:
                print("per: ", str(peripheral), " name: ", str(peripheral.name), " rssi: ", str(rssi), " id ", peripheral.identifier)

                if not self.connected_peripheral:
                    self.manager.connect(peripheral)

                _devices = self._devices
                key = hash(peripheral)
                if key in _devices:
                    _devices[key].rssi = rssi
                else:
                    _devices[key].rssi = rssi
                    _devices[key].peripheral = peripheral
#                    _devices[key].rssi = rssi
                    
##                    self.recycleview.data.append({'label': peripheral.identifier, 'rssi': rssi, 'peripheral': peripheral})

                    # connect here?
                    #self.manager.connect(peripheral)
                                        
                    pass
                self._devices = _devices
                

        # device removed - no longer visible?
        else:
            print('Removed ' + dev_id)







    #callback
    def remove_peripheral(self, peripheral: CBPeripheral):
        print("remove_peripheral", peripheral.name)
####        self.dispatch("on_peripheral",False,peripheral, 0)
    
    #callback

    def did_connect(self, peripheral: CBPeripheral):
        print("did_connect callback")
        #peripheral.discoverServices(None)
        #peripheral.discoverServices(peripheral.CBUUID)
        self.connected_peripheral = peripheral
        peripheral.delegate = self.current_device_data.delegate
        self.serviceUUID = CBUUID('4FAFC201-1FB5-459E-8FCC-C5C9C331D00D')
        peripheral.discoverServices([self.serviceUUID])
        self.active_peripheral = peripheral

        #peripheral.discoverServices()
        #self.user_peripheral_start( peripheral)


    def connect_status(self, status: bool):
        print("connect_status",status)

    # def on_peripheral(self,wid,peripheral,rssi): ...
    def on_peripheral(self, state: bool, peripheral: CBPeripheral, rssi: int):
        print("per: ", str(peripheral), " name: ", str(peripheral.name), " rssi: ", str(rssi), " id ", peripheral.identifier)
     
    def user_peripheral_start(self, peripheral):
        print('peripheral start')
        self.active_peripheral = peripheral
####        self.active_peripheral.discoverServices([peripheral.CBUUID])
        #peripheral.discoverServices(None)
        # these crash (because of lambda, double calls?)
        #Clock.schedule_interval(lambda dt: self.peripheral.discoverServices(), 5)



    def close(self):
        if self.connected:
            # if self.rfcomm_reader != None:
            #     self.rfcomm_reader.close()
            #     self.rfcomm_reader = None
            
            # if self.rfcomm_writer != None:
            #     self.rfcomm_writer.close()
            #     self.rfcomm_writer = None

            # if self.rfcomm_socket != None:
            #     self.rfcomm_socket.close()

            self.connected = False
            self.connected_device = None
            self.potential_remote_devices = []


    # why len? overrides standard operation 'len()'  Agggh
    #def read(self, len = None):
    def read(self, readLength = None):
        #print('read 1')
        if self.connection_failed:
            #print('read 1 failed')
            raise IOError("Bluetooth connection failed")
        else:
            # if self.connected and self.rfcomm_reader != None:
            #     available = self.rfcomm_reader.available()
            #     if available > 0:
            #         if hasattr(self.rfcomm_reader, "readNBytes"):
            #             return self.rfcomm_reader.readNBytes(available)
            #         else:
            #             # Compatibility mode for older android versions lacking readNBytes
            #             rb = self.rfcomm_reader.read().to_bytes(1, "big")
            #             return rb
            #     else:
            #         return bytes([])
            # else:
            #     raise IOError("No RFcomm socket available")
            
            
            #TODO - read from buffer - BLE received
            #return bytes([])

        #    print('read 2', type(self.current_device_data))
        #    print('read 2.1', type(self.current_device_data.in_queue))
        #    print('read 2.0.1 size empty list', len([]))
        #    print('read 2.1', self.current_device_data.in_queue)
        #    print('read 2.2 - check queue', len(self.current_device_data.in_queue))
            if len(self.current_device_data.in_queue) > 0:
        #        print('read 3')
                return self.current_device_data.in_queue.pop(0)
            else:
                return bytes([])

    
    def write(self, data):
                #print('writing')
                #peripheral.writeValue( bytes('Greetings!', 'utf-8'), characteristic, 0)
                #print('write sent')
        try:
            self.active_peripheral.writeValue( data, self.current_device_data.charTX, 0)
            return len(data)
        except Exception as e:
            RNS.log("Bluetooth connection failed for "+str(self), RNS.LOG_ERROR)
            self.connection_failed = True
            return 0

    
    
    
    
    
    
    
# class RNodeInterface(RNSIface):
class RNodeInterface(Interface):
#class RNodeInterface(RNSInterface):
#class RNodeInterface(RNS.Interfaces.Interface.Interface):
    MAX_CHUNK = 32768

    FREQ_MIN = 137000000
    FREQ_MAX = 1020000000

    RSSI_OFFSET = 157

    CALLSIGN_MAX_LEN    = 32

    REQUIRED_FW_VER_MAJ = 1
    REQUIRED_FW_VER_MIN = 52

    RECONNECT_WAIT = 5
    PORT_IO_TIMEOUT = 3

    Q_SNR_MIN_BASE = -9
    Q_SNR_MAX      = 6
    Q_SNR_STEP     = 2

    # needed for dispatch?
    bt_manager = None
    bt_target_device_name = None
    bt_target_device_address = None

    #def thread_BLEManager2(self):
    #    print('thread_BL2 - 1')
    #    self.bt_manager = iOSBluetoothManager(
    #        owner = self,
    #        target_device_name = self.bt_target_device_name,
    #        target_device_address = self.bt_target_device_address
    #    )
     #   print('thread_BL2 - 2')
        # while( self.bt_manager.current_device_data.interfaceReady == False ):
        #     sleep(1.0)
        #     print('wait BLE ready')
        # self.online = True
        # print('BLE marked ready')



    def __init__(
        self, owner, name, port, frequency = None, bandwidth = None, txpower = None,
        sf = None, cr = None, flow_control = False, id_interval = None,
        allow_bluetooth = False, target_device_name = None,
        target_device_address = None, id_callsign = None, st_alock = None, lt_alock = None):
        
        print (' iOS RNode interface init() ')
#        import importlib
#        if RNS.vendor.platformutils.is_android():
#            self.on_android  = True
#            if importlib.util.find_spec('usbserial4a') != None:
#                if importlib.util.find_spec('jnius') == None:
#                    RNS.log("Could not load jnius API wrapper for Android, RNode interface cannot be created.", RNS.LOG_CRITICAL)
#                    RNS.log("This probably means you are trying to use an USB-based interface from within Termux or similar.", RNS.LOG_CRITICAL)
#                    RNS.log("This is currently not possible, due to this environment limiting access to the native Android APIs.", RNS.LOG_CRITICAL)
#                    RNS.panic()

#                from usbserial4a import serial4a as serial
#                self.parity = "N"

#  doesn't exist :( 
#        if RNS.vendor.platformutils.is_ios():
        self.bt_target_device_name = target_device_name
        self.bt_target_device_address = target_device_address
        
        self.reconnect_w = RNodeInterface.RECONNECT_WAIT

        if allow_bluetooth:

            self.bt_manager = iOSBluetoothManager(
                owner = self,
                target_device_name = self.bt_target_device_name,
                target_device_address = self.bt_target_device_address
            )
            
        #    def thread_BLEManager(self):
        #        self.bt_manager = iOSBluetoothManager(
        #            owner = self,
        #            target_device_name = self.bt_target_device_name,
        #            target_device_address = self.bt_target_device_address
        #        )
                # while( self.bt_manager.current_device_data.interfaceReady == False ):
                #     sleep(1.0)
                #     print('wait BLE ready')
                # self.online = True
                # print('BLE marked ready')

        #    print('dispatch 1')
        #    self.dispatcher = DispatchSerialSwift()
        #    print('dispatch 2')
        #    self.dispatcher.dispatch2(  self.thread_BLEManager2, self)
        #    print('dispatch 3')
            

            #thread = threading.Thread(target=thread_BLEManager(self))
            #thread.daemon = True
            #thread.start()

        #    self.online      = True
            # iOS is still bringing the RNode up, show online when done
            self.online = False
            
            # while( self.bt_manager.current_device_data.interfaceReady == False ):
            #     sleep(1.0)
            #     print('wait BLE ready')
            # self.online = True
            # print('BLE marked ready')
            
            
        else:
            self.bt_manager = None
            self.online      = False

#                self.bt_target_device_name = target_device_name
#                self.bt_target_device_address = target_device_address
#                if allow_bluetooth:
#                    self.bt_manager = AndroidBluetoothManager(
#                        owner = self,
#                        target_device_name = self.bt_target_device_name,
#                        target_device_address = self.bt_target_device_address
#                    )

#                else:
#                    self.bt_manager = None
            
#            else:
#                RNS.log("Could not load USB serial module for Android, RNode interface cannot be created.", RNS.LOG_CRITICAL)
#                RNS.log("You can install this module by issuing: pip install usbserial4a", RNS.LOG_CRITICAL)
#                RNS.panic()
#        else:
#            raise SystemError("Android-specific interface was used on non-Android OS")

## updateRNS
#        self.rxb = 0
#        self.txb = 0
        super().__init__()

        self.HW_MTU = 508
        
# kjb todo ?
#        self.pyserial    = serial
        self.serial      = None
        self.owner       = owner
        self.name        = name
        self.port        = port
        self.speed       = 115200
        self.databits    = 8
        self.stopbits    = 1
        self.timeout     = 150
# moved up
#        self.online      = False
        self.hw_errors   = []
        self.allow_bluetooth = allow_bluetooth

        self.frequency   = frequency
        self.bandwidth   = bandwidth
        self.txpower     = txpower
        self.sf          = sf
        self.cr          = cr
        self.state       = KISS.RADIO_STATE_OFF
        self.bitrate     = 0
        self.st_alock    = st_alock
        self.lt_alock    = lt_alock
        self.platform    = None
        self.display     = None
        self.mcu         = None
        self.detected    = False
        self.firmware_ok = False
        self.maj_version = 0
        self.min_version = 0

        self.last_id     = 0
        self.first_tx    = None
        self.reconnect_w = RNodeInterface.RECONNECT_WAIT

        self.r_frequency = None
        self.r_bandwidth = None
        self.r_txpower   = None
        self.r_sf        = None
        self.r_cr        = None
        self.r_state     = None
        self.r_lock      = None
        self.r_stat_rx   = None
        self.r_stat_tx   = None
        self.r_stat_rssi = None
        self.r_stat_snr  = None
        self.r_st_alock  = None
        self.r_lt_alock  = None
        self.r_random    = None
        self.r_airtime_short      = 0.0
        self.r_airtime_long       = 0.0
        self.r_channel_load_short = 0.0
        self.r_channel_load_long  = 0.0
        self.r_symbol_time_ms = None
        self.r_symbol_rate = None
        self.r_preamble_symbols = None
        self.r_premable_time_ms = None

        self.packet_queue    = []
        self.flow_control    = flow_control
        self.interface_ready = False
        self.announce_rate_target = None
        self.last_port_io = 0
        self.port_io_timeout = RNodeInterface.PORT_IO_TIMEOUT
        self.last_imagedata = None

        self.validcfg  = True
        if (self.frequency == None or self.frequency < RNodeInterface.FREQ_MIN or self.frequency > RNodeInterface.FREQ_MAX):
            RNS.log("Invalid frequency configured for "+str(self), RNS.LOG_ERROR)
            self.validcfg = False

        if (self.txpower == None or self.txpower < 0 or self.txpower > 17):
            RNS.log("Invalid TX power configured for "+str(self), RNS.LOG_ERROR)
            self.validcfg = False

        if (self.bandwidth == None or self.bandwidth < 7800 or self.bandwidth > 500000):
            RNS.log("Invalid bandwidth configured for "+str(self), RNS.LOG_ERROR)
            self.validcfg = False

        if (self.sf == None or self.sf < 7 or self.sf > 12):
            RNS.log("Invalid spreading factor configured for "+str(self), RNS.LOG_ERROR)
            self.validcfg = False

        if (self.cr == None or self.cr < 5 or self.cr > 8):
            RNS.log("Invalid coding rate configured for "+str(self), RNS.LOG_ERROR)
            self.validcfg = False

        if (self.st_alock and (self.st_alock < 0.0 or self.st_alock > 100.0)):
            RNS.log("Invalid short-term airtime limit configured for "+str(self), RNS.LOG_ERROR)
            self.validcfg = False

        if (self.lt_alock and (self.lt_alock < 0.0 or self.lt_alock > 100.0)):
            RNS.log("Invalid long-term airtime limit configured for "+str(self), RNS.LOG_ERROR)
            self.validcfg = False

        if id_interval != None and id_callsign != None:
            if (len(id_callsign.encode("utf-8")) <= RNodeInterface.CALLSIGN_MAX_LEN):
                self.should_id = True
                self.id_callsign = id_callsign.encode("utf-8")
                self.id_interval = id_interval
            else:
                RNS.log("The encoded ID callsign for "+str(self)+" exceeds the max length of "+str(RNodeInterface.CALLSIGN_MAX_LEN)+" bytes.", RNS.LOG_ERROR)
                self.validcfg = False
        else:
            self.id_interval = None
            self.id_callsign = None

        if (not self.validcfg):
            raise ValueError("The configuration for "+str(self)+" contains errors, interface is offline")


        try:
            print('init do open_port')
            self.open_port()

#            if self.serial != None:
#                if self.serial.is_open:
#                    self.configure_device()
#                else:
#                    raise IOError("Could not open serial port")
#            elif self.bt_manager != None:

            print('RNode bt_manager.connected', self.bt_manager.connected)
            if self.bt_manager != None:
                print('after open port 1')
                if self.bt_manager.connected:
                    print('RNode configure_device - init')
                    self.configure_device()
                else:
                    print('after open port 2')
                    raise IOError("Could not connect to any Bluetooth devices")
            else:
                print('after open port 3')
                raise IOError("Neither serial port nor Bluetooth devices available")

        except Exception as e:
            print('after open port 4 - except')
            RNS.log("Could not open serial port for interface "+str(self), RNS.LOG_ERROR)
            RNS.log("The contained exception was: "+str(e), RNS.LOG_ERROR)
            print('after open port 4.1 - except')
            if len(self.hw_errors) == 0:
                print('after open port 4.2 - except')
                RNS.log("Reticulum will attempt to bring up this interface periodically", RNS.LOG_ERROR)
                thread = threading.Thread(target=self.reconnect_port)
                thread.daemon = True
                thread.start()
                print('after open port 4.3 - except')




    def read_mux(self, len=None):
#        if self.serial != None:
#            return self.serial.read()
#        elif self.bt_manager != None:
        if self.bt_manager != None:
            #print('read_mux')
            return self.bt_manager.read()
        else:
            raise IOError("No ports available for reading")



    def write_mux(self, data):
#        if self.serial != None:
#            written = self.serial.write(data)
#            self.last_port_io = time.time()
#            return written
#        elif self.bt_manager != None:
        if self.bt_manager != None:
            written = self.bt_manager.write(data)
            if (written == len(data)):
                self.last_port_io = time.time()
            return written
        else:
            raise IOError("No ports available for writing")



    def open_port(self):
        # if self.port != None:
        #     RNS.log("Opening serial port "+self.port+"...")
        #     # Get device parameters
        #     from usb4a import usb
        #     device = usb.get_usb_device(self.port)
        #     if device:
        #         vid = device.getVendorId()
        #         pid = device.getProductId()

        #         # Driver overrides for speficic chips
        #         proxy = self.pyserial.get_serial_port
        #         if vid == 0x1A86 and pid == 0x55D4:
        #             # Force CDC driver for Qinheng CH34x
        #             RNS.log(str(self)+" using CDC driver for "+RNS.hexrep(vid)+":"+RNS.hexrep(pid), RNS.LOG_DEBUG)
        #             from usbserial4a.cdcacmserial4a import CdcAcmSerial
        #             proxy = CdcAcmSerial

        #         self.serial = proxy(
        #             self.port,
        #             baudrate = self.speed,
        #             bytesize = self.databits,
        #             parity = self.parity,
        #             stopbits = self.stopbits,
        #             xonxoff = False,
        #             rtscts = False,
        #             timeout = None,
        #             inter_byte_timeout = None,
        #             # write_timeout = wtimeout,
        #             dsrdtr = False,
        #         )

        #         if vid == 0x0403:
        #             # Hardware parameters for FTDI devices @ 115200 baud
        #             self.serial.DEFAULT_READ_BUFFER_SIZE = 16 * 1024
        #             self.serial.USB_READ_TIMEOUT_MILLIS = 100
        #             self.serial.timeout = 0.1
        #         elif vid == 0x10C4:
        #             # Hardware parameters for SiLabs CP210x @ 115200 baud
        #             self.serial.DEFAULT_READ_BUFFER_SIZE = 64 
        #             self.serial.USB_READ_TIMEOUT_MILLIS = 12
        #             self.serial.timeout = 0.012
        #         elif vid == 0x1A86 and pid == 0x55D4:
        #             # Hardware parameters for Qinheng CH34x @ 115200 baud
        #             self.serial.DEFAULT_READ_BUFFER_SIZE = 64
        #             self.serial.USB_READ_TIMEOUT_MILLIS = 12
        #             self.serial.timeout = 0.1
        #         else:
        #             # Default values
        #             self.serial.DEFAULT_READ_BUFFER_SIZE = 1 * 1024
        #             self.serial.USB_READ_TIMEOUT_MILLIS = 100
        #             self.serial.timeout = 0.1

        #         RNS.log(str(self)+" USB read buffer size set to "+RNS.prettysize(self.serial.DEFAULT_READ_BUFFER_SIZE), RNS.LOG_DEBUG)
        #         RNS.log(str(self)+" USB read timeout set to "+str(self.serial.USB_READ_TIMEOUT_MILLIS)+"ms", RNS.LOG_DEBUG)
        #         RNS.log(str(self)+" USB write timeout set to "+str(self.serial.USB_WRITE_TIMEOUT_MILLIS)+"ms", RNS.LOG_DEBUG)

        # elif self.allow_bluetooth:
        
        # TODO - use this instead of earlier bring up of manager?
        # if self.allow_bluetooth:
        #     if self.bt_manager == None:
        #         self.bt_manager = AndroidBluetoothManager(
        #             owner = self,
        #             target_device_name = self.bt_target_device_name,
        #             target_device_address = self.bt_target_device_address
        #         )

        #     if self.bt_manager != None:
        #         self.bt_manager.connect_any_device()
        
        # TODO - since we connected on startup...  set this for now as test
        # happens in connect_any_device() in Android

        # TODO - timeout?        
#        while not self.bt_manager.current_device_data.interfaceReady:
#            sleep(0.5)
#        self.bt_manager.connected = True
        # set if ready, else wait for reconnect to come up
        if self.bt_manager.current_device_data.interfaceReady:
            self.bt_manager.connected = True
            # indicates radio(RNode connected), this is delayed on iOS
            # unlike Android
            self.online = True


    def configure_device(self):
        print('RNode - configure device')
        sleep(2.0)
        thread = threading.Thread(target=self.readLoop)
        thread.daemon = True
        thread.start()

        self.detect()
        sleep(0.5)

        if not self.detected:
            raise IOError("Could not detect device")
        else:
            if self.platform == KISS.PLATFORM_ESP32:
                self.display = True

        if not self.firmware_ok:
            raise IOError("Invalid device firmware")

#        if self.serial != None and self.port != None:
#            RNS.log("Serial port "+self.port+" is now open")
        if self.bt_manager != None and self.bt_manager.connected:
            ## updateeRNS
            self.timeout = 1500
            RNS.log("Bluetooth connection to RNode now open")

        RNS.log("Configuring RNode interface...", RNS.LOG_VERBOSE)
        self.initRadio()
        if (self.validateRadioState()):
            self.interface_ready = True
            RNS.log(str(self)+" is configured and powered up")
            sleep(0.3)
            self.online = True
        else:
            RNS.log("After configuring "+str(self)+", the reported radio parameters did not match your configuration.", RNS.LOG_ERROR)
            RNS.log("Make sure that your hardware actually supports the parameters specified in the configuration", RNS.LOG_ERROR)
            RNS.log("Aborting RNode startup", RNS.LOG_ERROR)
            
#            if self.serial != None:
#                self.serial.close()
            if self.bt_manager != None:
                self.bt_manager.close()

            raise IOError("RNode interface did not pass configuration validation")


    def initRadio(self):
        self.setFrequency()
        time.sleep(0.15)

        self.setBandwidth()
        time.sleep(0.15)
        
        self.setTXPower()
        time.sleep(0.15)
        
        self.setSpreadingFactor()
        time.sleep(0.15)
        
        self.setCodingRate()
        time.sleep(0.15)

        self.setSTALock()
        time.sleep(0.15)
        
        self.setLTALock()
        time.sleep(0.15)
        
        self.setRadioState(KISS.RADIO_STATE_ON)
        time.sleep(0.15)

    def detect(self):
        kiss_command = bytes([KISS.FEND, KISS.CMD_DETECT, KISS.DETECT_REQ, KISS.FEND, KISS.CMD_FW_VERSION, 0x00, KISS.FEND, KISS.CMD_PLATFORM, 0x00, KISS.FEND, KISS.CMD_MCU, 0x00, KISS.FEND])
        written = self.write_mux(kiss_command)
#        print('detect', written, 'cmd size', len(kiss_command), kiss_command)
        if written != len(kiss_command):
            raise IOError("An IO error occurred while detecting hardware for "+str(self))

## updateRNS - iOS unimplemented?
    def leave(self):
        kiss_command = bytes([KISS.FEND, KISS.CMD_LEAVE, 0xFF, KISS.FEND])
        written = self.write_mux(kiss_command)
        if written != len(kiss_command):
            raise IOError("An IO error occurred while sending host left command to device")

## TODO
# enable_bluetooth
# disable_bluetooth
# bluetooth_pair
# later in file - done - enable_external_framebuffer
# later in file - done - disable_external_framebuffer
# later in file - done - display_image
# later in file - done - write_framebuffer
# hard_reset (unused - only rnodeconf.py)

    def setFrequency(self):
        c1 = self.frequency >> 24
        c2 = self.frequency >> 16 & 0xFF
        c3 = self.frequency >> 8 & 0xFF
        c4 = self.frequency & 0xFF
        data = KISS.escape(bytes([c1])+bytes([c2])+bytes([c3])+bytes([c4]))

        kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_FREQUENCY])+data+bytes([KISS.FEND])
        written = self.write_mux(kiss_command)
        if written != len(kiss_command):
            raise IOError("An IO error occurred while configuring frequency for "+str(self))

    def setBandwidth(self):
        print(' set bandwidth ', self.bandwidth)
        c1 = self.bandwidth >> 24
        c2 = self.bandwidth >> 16 & 0xFF
        c3 = self.bandwidth >> 8 & 0xFF
        c4 = self.bandwidth & 0xFF
        data = KISS.escape(bytes([c1])+bytes([c2])+bytes([c3])+bytes([c4]))

        kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_BANDWIDTH])+data+bytes([KISS.FEND])
        written = self.write_mux(kiss_command)
        if written != len(kiss_command):
            raise IOError("An IO error occurred while configuring bandwidth for "+str(self))

    def setTXPower(self):
        txp = bytes([self.txpower])
        kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_TXPOWER])+txp+bytes([KISS.FEND])
        written = self.write_mux(kiss_command)
        if written != len(kiss_command):
            raise IOError("An IO error occurred while configuring TX power for "+str(self))

    def setSpreadingFactor(self):
        sf = bytes([self.sf])
        kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_SF])+sf+bytes([KISS.FEND])
        written = self.write_mux(kiss_command)
        if written != len(kiss_command):
            raise IOError("An IO error occurred while configuring spreading factor for "+str(self))

    def setCodingRate(self):
        cr = bytes([self.cr])
        kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_CR])+cr+bytes([KISS.FEND])
        written = self.write_mux(kiss_command)
        if written != len(kiss_command):
            raise IOError("An IO error occurred while configuring coding rate for "+str(self))

    def setSTALock(self):
        if self.st_alock != None:
            at = int(self.st_alock*100)
            c1 = at >> 8 & 0xFF
            c2 = at & 0xFF
            data = KISS.escape(bytes([c1])+bytes([c2]))

            kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_ST_ALOCK])+data+bytes([KISS.FEND])
            written = self.write_mux(kiss_command)
            if written != len(kiss_command):
                raise IOError("An IO error occurred while configuring short-term airtime limit for "+str(self))

    def setLTALock(self):
        if self.lt_alock != None:
            at = int(self.lt_alock*100)
            c1 = at >> 8 & 0xFF
            c2 = at & 0xFF
            data = KISS.escape(bytes([c1])+bytes([c2]))

            kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_LT_ALOCK])+data+bytes([KISS.FEND])
            written = self.write_mux(kiss_command)
            if written != len(kiss_command):
                raise IOError("An IO error occurred while configuring long-term airtime limit for "+str(self))

    def setRadioState(self, state):
        self.state = state
        kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_RADIO_STATE])+bytes([state])+bytes([KISS.FEND])
        written = self.write_mux(kiss_command)
        if written != len(kiss_command):
            raise IOError("An IO error occurred while configuring radio state for "+str(self))

    def validate_firmware(self):
        if (self.maj_version >= RNodeInterface.REQUIRED_FW_VER_MAJ):
            if (self.min_version >= RNodeInterface.REQUIRED_FW_VER_MIN):
                self.firmware_ok = True
        
        if self.firmware_ok:
            return

        RNS.log("The firmware version of the connected RNode is "+str(self.maj_version)+"."+str(self.min_version), RNS.LOG_ERROR)
        RNS.log("This version of Reticulum requires at least version "+str(RNodeInterface.REQUIRED_FW_VER_MAJ)+"."+str(RNodeInterface.REQUIRED_FW_VER_MIN), RNS.LOG_ERROR)
        RNS.log("Please update your RNode firmware with rnodeconf from https://github.com/markqvist/rnodeconfigutil/")
        error_description  = "The firmware version of the connected RNode is "+str(self.maj_version)+"."+str(self.min_version)+". "
        error_description += "This version of Reticulum requires at least version "+str(RNodeInterface.REQUIRED_FW_VER_MAJ)+"."+str(RNodeInterface.REQUIRED_FW_VER_MIN)+". "
        error_description += "Please update your RNode firmware with rnodeconf from: https://github.com/markqvist/rnodeconfigutil/"
        self.hw_errors.append({"error": KISS.ERROR_INVALID_FIRMWARE, "description": error_description})


    def validateRadioState(self):
        RNS.log("Wating for radio configuration validation for "+str(self)+"...", RNS.LOG_VERBOSE)
        if not self.platform == KISS.PLATFORM_ESP32:
            sleep(1.00);
        else:
            sleep(2.00);

        self.validcfg = True
        if (self.r_frequency != None and abs(self.frequency - int(self.r_frequency)) > 100):
            RNS.log("Frequency mismatch", RNS.LOG_ERROR)
            self.validcfg = False
        if (self.bandwidth != self.r_bandwidth):
            print('BW req', self.bandwidth, '  BW Rnode', self.r_bandwidth)
            RNS.log("Bandwidth mismatch", RNS.LOG_ERROR)
            self.validcfg = False
        if (self.txpower != self.r_txpower):
            RNS.log("TX power mismatch", RNS.LOG_ERROR)
            self.validcfg = False
        if (self.sf != self.r_sf):
            RNS.log("Spreading factor mismatch", RNS.LOG_ERROR)
            self.validcfg = False
        if (self.state != self.r_state):
            RNS.log("Radio state mismatch", RNS.LOG_ERROR)
            self.validcfg = False

        if (self.validcfg):
            return True
        else:
            return False




    def updateBitrate(self):
        try:
            self.bitrate = self.r_sf * ( (4.0/self.r_cr) / (math.pow(2,self.r_sf)/(self.r_bandwidth/1000)) ) * 1000
            self.bitrate_kbps = round(self.bitrate/1000.0, 2)
            RNS.log(str(self)+" On-air bitrate is now "+str(self.bitrate_kbps)+ " kbps", RNS.LOG_VERBOSE)
        except:
            self.bitrate = 0



    def processIncoming(self, data):
        self.rxb += len(data)

        print(' XXXXXX - processIncoming daemon start    XXXXXXXXX')
        def af():
            self.owner.inbound(data, self)
        threading.Thread(target=af, daemon=True).start()
        print(' XXXXXX - processIncoming daemon started    XXXXXXXXX')

        ## updateRNS
        #self.r_stat_rssi = None
        #self.r_stat_snr = None




    def processOutgoing(self,data):
        print('RNode out >', str(data),'<')
        datalen = len(data)
        
        # iOS checks
        if self.bt_manager.current_device_data.interfaceReady:
            self.interface_ready = True
        else:
            self.interface_ready = False
        # end iOS
            
        if self.online:
            if self.interface_ready:
                if self.flow_control:
                    self.interface_ready = False

                if data == self.id_callsign:
                    self.first_tx = None
                else:
                    if self.first_tx == None:
                        self.first_tx = time.time()

                data    = KISS.escape(data)
                frame   = bytes([0xc0])+bytes([0x00])+data+bytes([0xc0])

                written = self.write_mux(frame)
                self.txb += datalen

                if written != len(frame):
                    raise IOError("Serial interface only wrote "+str(written)+" bytes of "+str(len(data)))
            else:
                self.queue(data)
        print('RNode queue len ', len(self.packet_queue))

    def queue(self, data):
        self.packet_queue.append(data)

    def process_queue(self):
        if len(self.packet_queue) > 0:
            data = self.packet_queue.pop(0)
            self.interface_ready = True
            self.processOutgoing(data)
        elif len(self.packet_queue) == 0:
            self.interface_ready = True


    def readLoop(self):
        #print(' readloop 1')
        try:
            in_frame = False
            escape = False
            command = KISS.CMD_UNKNOWN
            data_buffer = b""
            command_buffer = b""
            last_read_ms = int(time.time()*1000)

        #    print(' readloop 2')
        #    print(' readloop 2.1', str(self.bt_manager.connected))

            # from Android - iOS no serial support yet
            # TODO: Ensure hotplug support for serial drivers
            # This should work now with the new time-based
            # detect polling.
#            while (self.serial != None and self.serial.is_open) or (self.bt_manager != None and self.bt_manager.connected):
            while (self.bt_manager != None and self.bt_manager.connected):
        #        print(' readloop 3')
                serial_bytes = self.read_mux()
        #        print(' readloop 3.1')
                got = len(serial_bytes)
                if got > 0:
                    self.last_port_io = time.time()

        #        print(' readloop 3.2', got)

                for byte in serial_bytes:
                    last_read_ms = int(time.time()*1000)

                    if (in_frame and byte == KISS.FEND and command == KISS.CMD_DATA):
                        print(' XXXXXXX  CMD_DATA   XXXXXXXXXX - TODO? ')
                        in_frame = False
                        self.processIncoming(data_buffer)
                        data_buffer = b""
                        command_buffer = b""
                    elif (byte == KISS.FEND):
        #                print(' 2 ')
                        in_frame = True
                        command = KISS.CMD_UNKNOWN
                        data_buffer = b""
                        command_buffer = b""
                    elif (in_frame and len(data_buffer) < self.HW_MTU):
        #                print(' 3 ')

                        if (len(data_buffer) == 0 and command == KISS.CMD_UNKNOWN):
                            command = byte
                        elif (command == KISS.CMD_DATA):
                            if (byte == KISS.FESC):
                                escape = True
                            else:
                                if (escape):
                                    if (byte == KISS.TFEND):
                                        byte = KISS.FEND
                                    if (byte == KISS.TFESC):
                                        byte = KISS.FESC
                                    escape = False
                                data_buffer = data_buffer+bytes([byte])
                        elif (command == KISS.CMD_FREQUENCY):
                            if (byte == KISS.FESC):
                                escape = True
                            else:
                                if (escape):
                                    if (byte == KISS.TFEND):
                                        byte = KISS.FEND
                                    if (byte == KISS.TFESC):
                                        byte = KISS.FESC
                                    escape = False
                                command_buffer = command_buffer+bytes([byte])
                                if (len(command_buffer) == 4):
                                    self.r_frequency = command_buffer[0] << 24 | command_buffer[1] << 16 | command_buffer[2] << 8 | command_buffer[3]
                                    RNS.log(str(self)+" Radio reporting frequency is "+str(self.r_frequency/1000000.0)+" MHz", RNS.LOG_DEBUG)
                                    self.updateBitrate()

                        elif (command == KISS.CMD_BANDWIDTH):
                            #print('cmd BW')
                            if (byte == KISS.FESC):
                                escape = True
                            else:
                                #print('BW else')
                                if (escape):
                                    if (byte == KISS.TFEND):
                                        byte = KISS.FEND
                                    if (byte == KISS.TFESC):
                                        byte = KISS.FESC
                                    escape = False
                                command_buffer = command_buffer+bytes([byte])
                                #print('buf', len(command_buffer))
                                if (len(command_buffer) == 4):
                                    #print('BW len 4')
                                    self.r_bandwidth = command_buffer[0] << 24 | command_buffer[1] << 16 | command_buffer[2] << 8 | command_buffer[3]
                                    RNS.log(str(self)+" Radio reporting bandwidth is "+str(self.r_bandwidth/1000.0)+" KHz", RNS.LOG_DEBUG)
                                    self.updateBitrate()

                        elif (command == KISS.CMD_TXPOWER):
                            self.r_txpower = byte
                            RNS.log(str(self)+" Radio reporting TX power is "+str(self.r_txpower)+" dBm", RNS.LOG_DEBUG)
                        elif (command == KISS.CMD_SF):
                            self.r_sf = byte
                            RNS.log(str(self)+" Radio reporting spreading factor is "+str(self.r_sf), RNS.LOG_DEBUG)
                            self.updateBitrate()
                        elif (command == KISS.CMD_CR):
                            self.r_cr = byte
                            RNS.log(str(self)+" Radio reporting coding rate is "+str(self.r_cr), RNS.LOG_DEBUG)
                            self.updateBitrate()
                        elif (command == KISS.CMD_RADIO_STATE):
                            #print('radio state', byte)
                            self.r_state = byte
                            if self.r_state:
                                RNS.log(str(self)+" Radio reporting state is online", RNS.LOG_DEBUG)
                            else:
                                RNS.log(str(self)+" Radio reporting state is offline", RNS.LOG_DEBUG)

                        elif (command == KISS.CMD_RADIO_LOCK):
                            self.r_lock = byte
                        elif (command == KISS.CMD_FW_VERSION):
                            if (byte == KISS.FESC):
                                escape = True
                            else:
                                if (escape):
                                    if (byte == KISS.TFEND):
                                        byte = KISS.FEND
                                    if (byte == KISS.TFESC):
                                        byte = KISS.FESC
                                    escape = False
                                command_buffer = command_buffer+bytes([byte])
                                if (len(command_buffer) == 2):
                                    self.maj_version = int(command_buffer[0])
                                    self.min_version = int(command_buffer[1])
                                    self.validate_firmware()

                        elif (command == KISS.CMD_STAT_RX):
                            if (byte == KISS.FESC):
                                escape = True
                            else:
                                if (escape):
                                    if (byte == KISS.TFEND):
                                        byte = KISS.FEND
                                    if (byte == KISS.TFESC):
                                        byte = KISS.FESC
                                    escape = False
                                command_buffer = command_buffer+bytes([byte])
                                if (len(command_buffer) == 4):
                                    self.r_stat_rx = ord(command_buffer[0]) << 24 | ord(command_buffer[1]) << 16 | ord(command_buffer[2]) << 8 | ord(command_buffer[3])

                        elif (command == KISS.CMD_STAT_TX):
                            if (byte == KISS.FESC):
                                escape = True
                            else:
                                if (escape):
                                    if (byte == KISS.TFEND):
                                        byte = KISS.FEND
                                    if (byte == KISS.TFESC):
                                        byte = KISS.FESC
                                    escape = False
                                command_buffer = command_buffer+bytes([byte])
                                if (len(command_buffer) == 4):
                                    self.r_stat_tx = ord(command_buffer[0]) << 24 | ord(command_buffer[1]) << 16 | ord(command_buffer[2]) << 8 | ord(command_buffer[3])

                        elif (command == KISS.CMD_STAT_RSSI):
                            self.r_stat_rssi = byte-RNodeInterface.RSSI_OFFSET
                        elif (command == KISS.CMD_STAT_SNR):
                            self.r_stat_snr = int.from_bytes(bytes([byte]), byteorder="big", signed=True) * 0.25
                            try:
                                sfs = self.r_sf-7
                                snr = self.r_stat_snr
                                q_snr_min = RNodeInterface.Q_SNR_MIN_BASE-sfs*RNodeInterface.Q_SNR_STEP
                                q_snr_max = RNodeInterface.Q_SNR_MAX
                                q_snr_span = q_snr_max-q_snr_min
                                quality = round(((snr-q_snr_min)/(q_snr_span))*100,1)
                                if quality > 100.0: quality = 100.0
                                if quality < 0.0: quality = 0.0
                                self.r_stat_q = quality
                            except:
                                pass

                        elif (command == KISS.CMD_ST_ALOCK):
                            if (byte == KISS.FESC):
                                escape = True
                            else:
                                if (escape):
                                    if (byte == KISS.TFEND):
                                        byte = KISS.FEND
                                    if (byte == KISS.TFESC):
                                        byte = KISS.FESC
                                    escape = False
                                command_buffer = command_buffer+bytes([byte])
                                if (len(command_buffer) == 2):
                                    at = command_buffer[0] << 8 | command_buffer[1]
                                    self.r_st_alock = at/100.0
                                    RNS.log(str(self)+" Radio reporting short-term airtime limit is "+str(self.r_st_alock)+"%", RNS.LOG_DEBUG)
                        elif (command == KISS.CMD_LT_ALOCK):
                            if (byte == KISS.FESC):
                                escape = True
                            else:
                                if (escape):
                                    if (byte == KISS.TFEND):
                                        byte = KISS.FEND
                                    if (byte == KISS.TFESC):
                                        byte = KISS.FESC
                                    escape = False
                                command_buffer = command_buffer+bytes([byte])
                                if (len(command_buffer) == 2):
                                    at = command_buffer[0] << 8 | command_buffer[1]
                                    self.r_lt_alock = at/100.0
                                    RNS.log(str(self)+" Radio reporting long-term airtime limit is "+str(self.r_lt_alock)+"%", RNS.LOG_DEBUG)
                        elif (command == KISS.CMD_STAT_CHTM):
                            if (byte == KISS.FESC):
                                escape = True
                            else:
                                if (escape):
                                    if (byte == KISS.TFEND):
                                        byte = KISS.FEND
                                    if (byte == KISS.TFESC):
                                        byte = KISS.FESC
                                    escape = False
                                command_buffer = command_buffer+bytes([byte])
                                if (len(command_buffer) == 8):
                                    ats = command_buffer[0] << 8 | command_buffer[1]
                                    atl = command_buffer[2] << 8 | command_buffer[3]
                                    cus = command_buffer[4] << 8 | command_buffer[5]
                                    cul = command_buffer[6] << 8 | command_buffer[7]
                                    
                                    self.r_airtime_short      = ats/100.0
                                    self.r_airtime_long       = atl/100.0
                                    self.r_channel_load_short = cus/100.0
                                    self.r_channel_load_long  = cul/100.0
                        elif (command == KISS.CMD_STAT_PHYPRM):
                            if (byte == KISS.FESC):
                                escape = True
                            else:
                                if (escape):
                                    if (byte == KISS.TFEND):
                                        byte = KISS.FEND
                                    if (byte == KISS.TFESC):
                                        byte = KISS.FESC
                                    escape = False
                                command_buffer = command_buffer+bytes([byte])
                                if (len(command_buffer) == 10):
                                    lst = (command_buffer[0] << 8 | command_buffer[1])/1000.0
                                    lsr = command_buffer[2] << 8 | command_buffer[3]
                                    prs = command_buffer[4] << 8 | command_buffer[5]
                                    prt = command_buffer[6] << 8 | command_buffer[7]
                                    cst = command_buffer[8] << 8 | command_buffer[9]

                                    if lst != self.r_symbol_time_ms or lsr != self.r_symbol_rate or prs != self.r_preamble_symbols or prt != self.r_premable_time_ms or cst != self.r_csma_slot_time_ms:
                                        self.r_symbol_time_ms    = lst
                                        self.r_symbol_rate       = lsr
                                        self.r_preamble_symbols  = prs
                                        self.r_premable_time_ms  = prt
                                        self.r_csma_slot_time_ms = cst
                                        RNS.log(str(self)+" Radio reporting symbol time is "+str(round(self.r_symbol_time_ms,2))+"ms (at "+str(self.r_symbol_rate)+" baud)", RNS.LOG_DEBUG)
                                        RNS.log(str(self)+" Radio reporting preamble is "+str(self.r_preamble_symbols)+" symbols ("+str(self.r_premable_time_ms)+"ms)", RNS.LOG_DEBUG)
                                        RNS.log(str(self)+" Radio reporting CSMA slot time is "+str(self.r_csma_slot_time_ms)+"ms", RNS.LOG_DEBUG)
                        elif (command == KISS.CMD_RANDOM):
                            self.r_random = byte
                        elif (command == KISS.CMD_PLATFORM):
                            self.platform = byte
                        elif (command == KISS.CMD_MCU):
                            self.mcu = byte
                        elif (command == KISS.CMD_ERROR):
                            if (byte == KISS.ERROR_INITRADIO):
                                RNS.log(str(self)+" hardware initialisation error (code "+RNS.hexrep(byte)+")", RNS.LOG_ERROR)
                                raise IOError("Radio initialisation failure")
                            elif (byte == KISS.ERROR_TXFAILED):
                                RNS.log(str(self)+" hardware TX error (code "+RNS.hexrep(byte)+")", RNS.LOG_ERROR)
                                raise IOError("Hardware transmit failure")
                            else:
                                RNS.log(str(self)+" hardware error (code "+RNS.hexrep(byte)+")", RNS.LOG_ERROR)
                                raise IOError("Unknown hardware failure")
                        elif (command == KISS.CMD_RESET):
                            if (byte == 0xF8):
                                if self.platform == KISS.PLATFORM_ESP32:
                                    if self.online:
                                        RNS.log("Detected reset while device was online, reinitialising device...", RNS.LOG_ERROR)
                                        raise IOError("ESP32 reset")
                        elif (command == KISS.CMD_READY):
                            self.process_queue()
                        elif (command == KISS.CMD_DETECT):
                #            print('kiss detect')
                            if byte == KISS.DETECT_RESP:
                                self.detected = True
                #                print('kiss detect TRUE')
                            else:
                                self.detected = False
                #                print('kiss detect fail')

                if got == 0:
                    time_since_last = int(time.time()*1000) - last_read_ms
                    if len(data_buffer) > 0 and time_since_last > self.timeout:
                        RNS.log(str(self)+" serial read timeout in command "+str(command), RNS.LOG_WARNING)
                        data_buffer = b""
                        in_frame = False
                        command = KISS.CMD_UNKNOWN
                        escape = False

                    if self.id_interval != None and self.id_callsign != None:
                        if self.first_tx != None:
                            if time.time() > self.first_tx + self.id_interval:
                                RNS.log("Interface "+str(self)+" is transmitting beacon data: "+str(self.id_callsign.decode("utf-8")), RNS.LOG_DEBUG)
                                self.processOutgoing(self.id_callsign)
                    
                    if (time.time() - self.last_port_io > self.port_io_timeout):
                        self.detect()
                    
                    if (time.time() - self.last_port_io > self.port_io_timeout*3):
                        raise IOError("Connected port for "+str(self)+" became unresponsive")

                    if self.bt_manager != None:
                        sleep(0.08)

        except Exception as e:
            self.online = False
# Android
#            RNS.log("A serial port occurred, the contained exception was: "+str(e), RNS.LOG_ERROR)
#            RNS.log("The interface "+str(self)+" experienced an unrecoverable error and is now offline.", RNS.LOG_ERROR)
            RNS.log("A BLE error occurred, the contained exception was: "+str(e), RNS.LOG_ERROR)
            RNS.log("The interface "+str(self)+" experienced an error and is now offline.", RNS.LOG_ERROR)

            if RNS.Reticulum.panic_on_interface_error:
                RNS.panic()

            RNS.log("Reticulum will attempt to reconnect the interface periodically.", RNS.LOG_ERROR)

        self.online = False

#        if self.serial != None:
#            self.serial.close()

        if self.bt_manager != None:
            self.bt_manager.close()

        self.reconnect_port()




    def reconnect_port(self):
        print('reconnect port 1')
        while not self.online and len(self.hw_errors) == 0:
            print('reconnect port - while - 2')
            try:
                print('reconnect port 3.1')
                time.sleep(self.reconnect_w)
                print('reconnect port 3.2')
#                if self.serial != None and self.port != None:
#                    RNS.log("Attempting to reconnect serial port "+str(self.port)+" for "+str(self)+"...", RNS.LOG_EXTREME)

                if self.bt_manager != None:
                    RNS.log("Attempting to reconnect Bluetooth device for "+str(self)+"...", RNS.LOG_EXTREME)

                print('reconnect port 4')
                self.open_port()
                print('reconnect port 5')

                # if hasattr(self, "serial") and self.serial != None and self.serial.is_open:
                #     self.configure_device()
                #     if self.online:
                #         if self.last_imagedata != None:
                #             self.display_image(self.last_imagedata)
                #             self.enable_external_framebuffer()
                
                #elif hasattr(self, "bt_manager") and self.bt_manager != None and self.bt_manager.connected:
                print('RNode do config device?')
                if hasattr(self, "bt_manager") and self.bt_manager != None and self.bt_manager.connected:
                    print('RNode configure_device - reconnect')
                    self.configure_device()
                    if self.online:
                        if self.last_imagedata != None:
                            self.display_image(self.last_imagedata)
                            self.enable_external_framebuffer()

            except Exception as e:
                RNS.log("Error while reconnecting RNode, the contained exception was: "+str(e), RNS.LOG_ERROR)

        if self.online:
            RNS.log("Reconnected serial port for "+str(self))





    def enable_external_framebuffer(self):
        if self.display != None:
            kiss_command = bytes([KISS.FEND, KISS.CMD_FB_EXT, 0x01, KISS.FEND])
            written = self.write_mux(kiss_command)
            if written != len(kiss_command):
                raise IOError("An IO error occurred while enabling external framebuffer on device")

    def disable_external_framebuffer(self):
        if self.display != None:
            kiss_command = bytes([KISS.FEND, KISS.CMD_FB_EXT, 0x00, KISS.FEND])
            written = self.write_mux(kiss_command)
            if written != len(kiss_command):
                raise IOError("An IO error occurred while disabling external framebuffer on device")


    # if online, rnode needs to respond to:
#    display_image(sideband_fb_data)
#    enable_external_framebuffer()
#    self.last_imagedata = sideband_fb_data

    FB_PIXEL_WIDTH     = 64
    FB_BITS_PER_PIXEL  = 1
    FB_PIXELS_PER_BYTE = 8//FB_BITS_PER_PIXEL
    FB_BYTES_PER_LINE  = FB_PIXEL_WIDTH//FB_PIXELS_PER_BYTE
    def display_image(self, imagedata):
        self.last_imagedata = imagedata
        if self.display != None:
            lines = len(imagedata)//8
            for line in range(lines):
                line_start = line*RNodeInterface.FB_BYTES_PER_LINE
                line_end   = line_start+RNodeInterface.FB_BYTES_PER_LINE
                line_data = bytes(imagedata[line_start:line_end])
                self.write_framebuffer(line, line_data)

    def write_framebuffer(self, line, line_data):
        if self.display != None:
            line_byte = line.to_bytes(1, byteorder="big", signed=False)
            data = line_byte+line_data
            escaped_data = KISS.escape(data)
            kiss_command = bytes([KISS.FEND])+bytes([KISS.CMD_FB_WRITE])+escaped_data+bytes([KISS.FEND])
            
            written = self.write_mux(kiss_command)
            if written != len(kiss_command):
                raise IOError("An IO error occurred while writing framebuffer data device")

## updateRNS
    def detach(self):
        self.disable_external_framebuffer()
        self.setRadioState(KISS.RADIO_STATE_OFF)
        self.leave()

## updateRNS
    def should_ingress_limit(self):
        return False

    def __str__(self):
        return "RNodeInterface["+str(self.name)+"]"

