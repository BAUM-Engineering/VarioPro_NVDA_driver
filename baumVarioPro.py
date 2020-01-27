# -*- coding: UTF-8 -*-
#brailleDisplayDrivers/baumVarioPro.py
#Description:
#	BAUM VarioPro driver for NVDA.
#	This file is covered by the GNU General Public License.
#Copyright: (C) 2020 BAUM Engineering SRL
#Author: 
#	Florin Trutiu (ft@baum.ro)
#Versions:
#	2017.03.17 - initial (alpha) release
#	2017.03.24 - using the hwIo.Serial class
#			   - no longer calling the InputGesture with keys data that is 0
#			   - some gesture mappings similar to the ones from the BAUM Cobra screenreader
#	2017.04.05 - added support for the TASO module
#			   - all connected modules information is stored in a dictionary 
#			   - added some gesture mappings for TASO, similar to the ones from the BAUM Cobra screenreader 
#	2017.04.06 - optimization for main module Braille output
#			   - fix crash in display when unknown modules are connected 
#	2017.09.15 - added support for Status and Telephone modules (Braille output not implemented)
#			   - some gesture mappings were added or changed based on the suggestions provided by the testers			
#   2017.10.13 - changed the code to adhere to a better object oriented architecture:
#                   - created a base class VarioProModule which is inherited by the classes created for each VarioPro module 
#                   - the code might not be as clear as it was in the previous flat implementation 
#              - added some code improvements suggested by Leonard de Ruijter:
#                   - using hwIo.Serial's waitForRead method instead of time.sleep (which locks the main thread of NVDA)
#                   - using xrange objects instead of the range objects
#                   - added description to some functions
#   2020.01.22 - converted the code to work with Python3                     
#Note: Added the possibility to output on the Status and Telephone modules. 
#      But we still need to find a method to make NVDA send text specifically for the Status and Telephone Braille modules.

import time
from collections import OrderedDict
import hwPortUtils
import braille
import inputCore
from logHandler import log
import brailleInput
import hwIo
import threading
from struct import unpack

TIMEOUT = 0.2
BAUD_RATE = 19200

# VARIO PRO MODULE TYPE IDS
VP_MAIN_80 = (0x80, 0x41)
VP_MAIN_64 = (0x81, 0x41)
TASO = (0x95, 0x41)
STATUS = (0x90, 0x41)
TELEPHONE = (0x91, 0x41)

# REAL INFO TYPE IDS
BAUM_VP_DEVICE_DETECTION = "\x50"
BAUM_VP_DYNAMIC_DATA_BLOCK = "\x51"

# FAKE IDS for keeping the dictionary based structure
BAUM_ROUTING_KEYS = "VP_RK"
BAUM_DISPLAY_KEYS = "VP_DK"
BAUM_WHEELS_UP = "VP_WU"
BAUM_WHEELS_DOWN = "VP_WD"
BAUM_WHEELS_PUSH = "VP_WP"

BAUM_TASO_NC_KEYS = "TASO_NCK" #TASO NUMPAD and CONTROL KEYS
BAUM_TASO_SW_KEYS = "TASO_SWK" #TASO SLIDER and WHEEL KEYS
BAUM_TASO_SW_RELEASES = "TASO_SWR" #TASO SLIDER and WHEEL keys releases
BAUM_TASO_HORIZONTAL_SLIDER = "TASO_HS"
BAUM_TASO_VERTICAL_SLIDER = "TASO_VS"
BAUM_TASO_WHEEL = "TASO_W"

BAUM_TELEPHONE_KCW_KEYS = "TM_KCWK" #TELEPHONE MODULE KEYPAD, CONTROL and WHEEL KEYS
BAUM_TELEPHONE_ROUTING_KEYS = "TM_RK"
BAUM_TELEPHONE_WHEEL = "TM_W"
BAUM_TELEPHONE_CELLS_NUMBER = 12

BAUM_STATUS_ROUTING_KEYS = "SM_RK"
BAUM_STATUS_C_KEYS = "SM_CK"
BAUM_STATUS_CELLS_NUMBER = 4

KEY_NAMES = {
    BAUM_ROUTING_KEYS: None,
    BAUM_DISPLAY_KEYS: ("d1", "d2", "d3", "d4", "d5", "d6"),
    BAUM_WHEELS_UP: ("wu1", "wu2", "wu3", "wu4"),
    BAUM_WHEELS_DOWN: ("wd1", "wd2", "wd3", "wd4"),
    BAUM_WHEELS_PUSH: ("wp1", "wp2", "wp3", "wp4"),
    BAUM_TASO_NC_KEYS: ("tn1", "tn2", "tn3", "tn4", "tn5", "tn6", "tn7", "tn8", "tn9", "tn*", "tn0", "tn#", "tc1", "tc2", "tc3"), # numpad and control keys
    BAUM_TASO_SW_KEYS: ("thsp", "tvsp", "twp"), # sliders and wheel buttons
    BAUM_TASO_SW_RELEASES: ("tswr",), # send a key release for the slider and wheel keys
    BAUM_TASO_VERTICAL_SLIDER: ("tvsd", "tvsu"), 
    BAUM_TASO_HORIZONTAL_SLIDER: ("thsl", "thsr"),
    BAUM_TASO_WHEEL: ("twl", "twr"),
    BAUM_TELEPHONE_KCW_KEYS: ("tmk1", "tmk2", "tmk3", "tmkA", "tmk4", "tmk5", "tmk6", "tmkB", "tmk7", "tmk8", "tmk9", "tmkC", "tmk*", "tmk0", "tmk#", "tmkD", "tmc1", "tmc2", "tmc3", "tmc4", "tmwp"), # kepad, control and wheel keys
    BAUM_TELEPHONE_ROUTING_KEYS: None,
    BAUM_TELEPHONE_WHEEL: ("tmwd", "tmwu"), # wheel down and up directions
    BAUM_STATUS_C_KEYS: ("smc1", "smc2", "smc3", "smc4"), # control keys
    BAUM_STATUS_ROUTING_KEYS: None
}

USB_IDS_SER = {
    "VID_0403&PID_FE76", # VarioPro 80
    "VID_0403&PID_FE77", # VarioPro 64
}

class VarioProModule(object):
    """ A class to abstract all Vario Pro modules, including the main module """
    
    def __init__(self, driver, module_id): 
        self.module_type = module_id[:2]
        self.module_id = module_id
        self.has_braille = False
        self.number_cells = 0
        self.input_handler = None
        self.output_handler = driver.braille_out_send # None for modules without Braille output

class VarioProMainModule80(VarioProModule):
    """ A class that implements the methods to process the input from the main module keys and operators"""
    def __init__(self, driver, module_id):
        super(type(self), self).__init__(driver, module_id)
        self.number_cells = 80
        driver.numCells = self.number_cells  
        self.input_handler = self.process_main_data_packet # None for output only modules
        driver.mainModule = self

        self.cumul_d_keys = 0

        log.info("VarioPro80 main module initialized")

    def process_main_display_keys(self, d_keys):
        if d_keys != 0x00:
            # cumulate keys and exit 
            self.cumul_d_keys |= d_keys	
            return

        # send all previously cumulated keys
        kd = {BAUM_DISPLAY_KEYS : self.cumul_d_keys}	
        try:
            ig = InputGesture(kd)
            inputCore.manager.executeGesture(ig)
        except inputCore.NoInputGestureAction:
            pass

        self.cumul_d_keys = 0;

    def process_main_routing_keys(self, r_keys):
        """Process routing keys as a bitmap of keys"""
        if not r_keys:
            return
        kd = {BAUM_ROUTING_KEYS : r_keys}
        try:
            ig = InputGesture(kd)
            inputCore.manager.executeGesture(ig)
        except inputCore.NoInputGestureAction:
            pass

    def process_main_wheel_rotation(self, wheels):
        for wi in range(len(wheels)):
            w = wheels[wi]
            ws = unpack("b", bytes([w]))[0]
            if ws != 0:
                sign = lambda x: -1 if x < 0 else 1
                for i in range(0, ws, sign(ws)):
                    if ws > 0:
                        wd = {BAUM_WHEELS_UP : 1 << wi}
                    else:
                        wd = {BAUM_WHEELS_DOWN : 1 << wi}
                try:
                    ig = InputGesture(wd)
                    inputCore.manager.executeGesture(ig)
                except inputCore.NoInputGestureAction:
                    pass

    def process_main_wheel_buttons(self, wp):
        # Should we send all previously cumulated keys like we do for display keys (?!?)
        if not wp:
            return 
        kd = {BAUM_WHEELS_PUSH : wp}	
        try:
            ig = InputGesture(kd)
            inputCore.manager.executeGesture(ig)
        except inputCore.NoInputGestureAction:
            pass

    def process_main_data_packet(self, pkt):
        stat = pkt[0]
        if stat & 0x08: # routing keys changed
            # Put R keys bitmap in a looong word (this is anyway what NVDA expects, and easier to compare and assign)
            r_keys = 0
            klen = int(self.number_cells/8) 
            for i in range(klen): 
                if self.module_type == VP_MAIN_80:
                    r_keys |= pkt[i + 8] << (i * 8)
                else:
                    r_keys |= pkt[i + 7] << (i * 8)
            self.process_main_routing_keys(r_keys)
        elif stat & 0x04: # D keys changed
            self.process_main_display_keys(pkt[7] if self.module_type == VP_MAIN_80 else pkt[6])
        elif stat & 0x02: # wheel buttons changed
            self.process_main_wheel_buttons(pkt[6] if self.module_type == VP_MAIN_80 else pkt[5])
        elif stat & 0x01: # wheel rotation
            self.process_main_wheel_rotation(pkt[2:6] if self.module_type == VP_MAIN_80 else pkt[2:5])
        else:
            pass

class VarioProMainModule64(VarioProMainModule80):
    def __init__(self, driver, module_id):
        super(VarioProMainModule80, self).__init__(driver, module_id)
        self.number_cells = 64
        driver.numCells = self.number_cells  
        driver.mainModule = self	
        log.info("VarioPro64 main module initialized")

class VarioProTasoModule(VarioProModule):
    """ A class that implements the methods to process the input from the TASO module keys and operators"""
    def __init__(self, driver, module_id):
        super(type(self), self).__init__(driver, module_id)
        self.prev_ths = 0
        self.prev_tvs = 0
        self.prev_tnckeys = 0
        self.prev_tswkeys = 0		
        self.input_handler = self.process_taso_data_packet
        log.info("TASO module initialized")

    def process_taso_wheel_rotation(self, wd):
        ws = unpack("b", bytes([wd]))[0]
        sign = lambda x: -1 if x < 0 else 1  
        for i in range(0, ws, sign(ws)):
            if ws > 0:
                wd = {BAUM_TASO_WHEEL: (1 << 1)} 
            else:
                wd = {BAUM_TASO_WHEEL: 1}
            try:
                ig = InputGesture(wd)
                inputCore.manager.executeGesture(ig)
            except inputCore.NoInputGestureAction:
                pass

    def process_taso_vertical_slider_position(self, val):
        vs = abs(self.prev_tvs - val)
        if vs:
            for i in range (0, vs):
                vsd = {BAUM_TASO_VERTICAL_SLIDER: (1 << 1) if val < self.prev_tvs else 1}
                try:
                    ig = InputGesture(vsd)
                    inputCore.manager.executeGesture(ig)
                except inputCore.NoInputGestureAction:
                    pass
        self.prev_tvs = val	

    def process_taso_horizontal_slider_position(self, val):
        hs = abs(self.prev_ths - val)
        if hs:
            for i in range (0, hs):
                hsd = {BAUM_TASO_HORIZONTAL_SLIDER: (1 << 1) if val > self.prev_ths else 1}
                try:
                    ig = InputGesture(hsd)
                    inputCore.manager.executeGesture(ig)
                except inputCore.NoInputGestureAction:
                    pass
        self.prev_ths = val		

    def process_taso_keys(self, tk):
        #all TASO keys are sent as pressed, no cumulation.
        #also we can use the the # and * keys as shift keys
        tnckeys	= tk[0] | (tk[1] << 8) | ((tk[2] & 0x07) << 12)	
        tswkeys	= (tk[2] >> 5) & 0x07
        
        if self.prev_tnckeys != tnckeys:
            if tnckeys: # only key presses
                tnckeysd = {BAUM_TASO_NC_KEYS:  tnckeys}
                try:
                    ig = InputGesture(tnckeysd)
                    inputCore.manager.executeGesture(ig)
                except inputCore.NoInputGestureAction:
                    pass

        if self.prev_tswkeys != tswkeys:
            if tswkeys:
                td = {BAUM_TASO_SW_KEYS : tswkeys} 
            else:
                td = {BAUM_TASO_SW_RELEASES: 1}
            try:
                ig = InputGesture(td)
                inputCore.manager.executeGesture(ig)
            except inputCore.NoInputGestureAction:
                pass

        self.prev_tnckeys = tnckeys
        self.prev_tswkeys = tswkeys

    def process_taso_data_packet(self, pkt):
        stat = pkt[0]
        if stat & 0x08: #keys status (general) change
            self.process_taso_keys(pkt[5:8])
        elif stat & 0x04: #horizontal slider position change
            self.process_taso_horizontal_slider_position(pkt[4]) 
        elif stat & 0x02: #vertical slider change
            self.process_taso_vertical_slider_position(pkt[3]) 
        elif stat & 0x01: #wheel position change
            self.process_taso_wheel_rotation(pkt[2])

class VarioProStatusModule(VarioProModule):
    """ A class that implements the methods to process the input from the STATUS module keys and operators"""
    def __init__(self, driver, module_id):
        super(type(self), self).__init__(driver, module_id)
        self.prev_sm_c_keys = 0
        self.number_cells = 4
        self.has_braille = True	
        self.input_handler = self.process_status_module_data_packet
        driver.statusModule = self
        log.info("Status module initialized")

    def process_status_module_keys(self, sk):
        #all STATUS keys are sent as pressed, no cumulation.
        sm_c_keys	= sk & 0x0F		

        if self.prev_sm_c_keys != sm_c_keys:
            if sm_c_keys: # only key presses
                sm_c_keysd = {BAUM_STATUS_C_KEYS:  sm_c_keys} 		
                try:
                    ig = InputGesture(sm_c_keysd)
                    inputCore.manager.executeGesture(ig)
                except inputCore.NoInputGestureAction:
                    pass
        self.prev_sm_c_keys = sm_c_keys

    def process_status_module_data_packet(self, pkt):
        stat = pkt[0]
        if stat & 0x02: #keys status (general) change
            self.process_status_module_keys(pkt[2])

class VarioProTelephoneModule(VarioProModule):
    """ A class that implements the methods to process the input from the TELEPHONE module keys and operators"""
    def __init__(self, driver, module_id):
        super(type(self), self).__init__(driver, module_id)
        self.prev_tm_kcw_keys = 0
        self.number_cells = 12
        self.has_braille = True		
        self.input_handler = self.process_telephone_module_data_packet
        driver.telephoneModule = self
        log.info("Telephone module initialized")

    def process_telephone_module_keys(self, tk):
        #all TELEPHONE keys are sent as pressed, no cumulation.
        tm_kcw_keys	= tk[1] | (tk[2] << 8) | ((tk[0] & 0x0F) << 16) | ((tk[0] >> 7) << 20)		

        if self.prev_tm_kcw_keys != tm_kcw_keys:
            if tm_kcw_keys: # only key presses
                tm_kcw_keysd = {BAUM_TELEPHONE_KCW_KEYS:  tm_kcw_keys} 		
                try:
                    ig = InputGesture(tm_kcw_keysd)
                    inputCore.manager.executeGesture(ig)
                except inputCore.NoInputGestureAction:
                    pass
        self.prev_tm_kcw_keys = tm_kcw_keys

    def process_telephone_module_wheel_rotation(self, wd):
        ws = unpack("b", bytes([wd]))[0]
        sign = lambda x: -1 if x < 0 else 1  
        for i in range(0, ws, sign(ws)):
            if ws > 0:
                wd = {BAUM_TELEPHONE_WHEEL: (1 << 1)} 
            else:
                wd = {BAUM_TELEPHONE_WHEEL: 1}
            try:
                ig = InputGesture(wd)
                inputCore.manager.executeGesture(ig)
            except inputCore.NoInputGestureAction:
                pass		

    def process_telephone_module_data_packet(self, pkt):
        stat = pkt[0]
        if stat & 0x02: #keys status (general) change
            self.process_telephone_module_keys(pkt[3:8])
        elif stat & 0x01: #wheel position change
            self.process_telephone_module_wheel_rotation(pkt[2])


## MAP with VarioProModule classes based on the module type
VP_MODULE_CLASSES_MAP = {
    VP_MAIN_80 : VarioProMainModule80,
    VP_MAIN_64 : VarioProMainModule64,
    TASO : VarioProTasoModule,
    TELEPHONE : VarioProTelephoneModule,
    STATUS : VarioProStatusModule,
}

class BrailleDisplayDriver(braille.BrailleDisplayDriver):
    name = "baumVarioPro"
    # Translators: Names of braille displays.
    description = _("Baum VarioPro braille displays")
    isThreadSafe = True

    @classmethod
    def check(cls):
        return True

    @classmethod
    def getPossiblePorts(cls):
        ports = OrderedDict()
        comPorts = list(hwPortUtils.listComPorts(onlyAvailable=True))
        try:
            next(cls._getAutoPorts(comPorts))
            ports.update((cls.AUTOMATIC_PORT,))
        except StopIteration:
            pass
        for portInfo in comPorts:
            # Translators: Name of a serial communications port.
            ports[portInfo["port"]] = _("Serial: {portName}").format(portName=portInfo["friendlyName"])
        return ports

    @classmethod
    def _getAutoPorts(cls, comPorts):
        for portInfo in sorted(comPorts, key=lambda item: "bluetoothName" in item):
            port = portInfo["port"]
            hwID = portInfo["hardwareID"]
            if hwID.startswith(r"FTDIBUS\COMPORT"):
                # USB.
                portType = "USB serial"
                try:
                    usbID = hwID.split("&", 1)[1]
                except IndexError:
                    continue
                if usbID not in USB_IDS_SER:
                    continue
            else:
                continue
            yield port, portType

    def __init__(self, port="Auto"):
        log.info("BAUM VarioPro Init")
        super(BrailleDisplayDriver, self).__init__()
        self.numCells = 0

        self.VPS_IDLE = 0
        self.VPS_W4_INFOTYPE = 1
        self.VPS_W4_LENGTH = 2
        self.VPS_GET_PAYLOAD = 3
        self.bp_sm_state = self.VPS_IDLE
        self.vp_pkt = []
        self.vp_payload_len = 0

        self._dev = None
        self.bp_trans_prev_byte = 0 #BAUM protocol transport layer (ESC dedoubling)

        self.mainModule = None
        self.statusModule = None
        self.telephoneModule = None
        self.connected_modules = {}

        try:		
            if port == "auto":
                tryPorts = self._getAutoPorts(hwPortUtils.listComPorts(onlyAvailable=True))
            else:
                tryPorts = ((port, "serial"),)
            for port, portType in tryPorts:
                if self._dev:
                    if self._dev.is_open(): self._dev.close()
                self._dev = hwIo.Serial(port, baudrate=BAUD_RATE, timeout=TIMEOUT, writeTimeout=TIMEOUT, onReceive=self._onReceive)
                self.vp_query_modules()
                for i in range(10): # wait for dev arrival
                    self._dev.waitForRead(TIMEOUT)
                    if self.numCells:
                        break
                    else:
                        log.error("Device arrival timeout")
        except Exception as e:
            log.error(e)
            raise RuntimeError("No BAUM VarioPro display found")

    def terminate(self):
        try: 
            super(BrailleDisplayDriver, self).terminate()
        except Exception as e:
            log.error(e)
        finally:
            if self._dev:
                self._dev.close()

    def _onReceive(self, data):
        try:
            self.decode_escape_transport(data)
        except Exception as e: # IOError?
            #break #???
            log.error(e)

    def send_packet(self, cmd, payload):
        dep = bytearray()
        for b in payload:
            dep.append(b)
            if b == 0x1B: dep.append(b)
        depl = len(payload)
        fpkt = [0x1B, cmd, depl]
        if depl == 0x1B: 
            fpkt.append(depl)
        fpkt.extend(dep)
        self._dev.write(fpkt)        

    def vp_query_modules(self):
        #For query the Device-ID and Serial Number must be 0 
        payload = bytearray(b"\x00\x00\x00\x00\x04")
        self.send_packet(ord(BAUM_VP_DEVICE_DETECTION), payload) 		

    def acknowledge_device_arrival(self, module_info):
        payload = bytearray() 
        payload.extend(module_info)
        payload.append(0x01)
        self.send_packet(ord(BAUM_VP_DEVICE_DETECTION), payload)

    def process_packet(self, pkt):
        #NOTE: pkt is ESC dedoubled
        if pkt[0] == ord(BAUM_VP_DEVICE_DETECTION):			
            if len(pkt) == 11: 
                if pkt[10] == 0x01:
                    try:
                        id = tuple(pkt[2:6])
                        type = tuple(pkt[2:4])
                        module = VP_MODULE_CLASSES_MAP[type](self, id) # instantiate the module from the appropriate class based on the module id
                        self.connected_modules[module.module_id] = module
                        self.acknowledge_device_arrival(id)
                    except Exception as e:
                        log.error(e)
                elif pkt[10] == 0x02: #device removal 
                    try:
                        del self.connected_modules[tuple(pkt[2:6])]
                    except Exception as e:
                        log.error(e)
                elif pkt[10] == 0x03: #device rejected 
                    pass #TODO: Inform the user that another module with the same I2C address was connected, and this causes a communication conflict
        elif pkt[0] == ord(BAUM_VP_DYNAMIC_DATA_BLOCK):
            m = self.connected_modules[tuple(pkt[2:6])] 
            try:
                m.input_handler(pkt[6:]) # call the appropriate packet handler for module
            except Exception:
                log.error(f"module not connected or not supported")  
        else:
            log.info("invalid VP pkt")

    def baumprotocol_receive_statemachine(self, b):	# packetizing state machine
        if self.bp_sm_state == self.VPS_IDLE:
            if b == 0x1B:
                self.vp_pkt = bytearray()
            self.bp_sm_state = self.VPS_W4_INFOTYPE
        elif self.bp_sm_state == self.VPS_W4_INFOTYPE:
            self.vp_pkt.append(b)
            if b == 0x50 or b == 0x51:
                self.bp_sm_state = self.VPS_W4_LENGTH
            else:
                self.bp_sm_state = self.VPS_IDLE

        elif self.bp_sm_state == self.VPS_W4_LENGTH:
            self.vp_pkt.append(b)
            self.vp_payload_len = b
            self.bp_sm_state = self.VPS_GET_PAYLOAD

        elif self.bp_sm_state == self.VPS_GET_PAYLOAD:
            self.vp_pkt.append(b)
            self.vp_payload_len -= 1
            if self.vp_payload_len == 0:
                self.process_packet(bytearray(self.vp_pkt))
                self.bp_sm_state = self.VPS_IDLE

    def decode_escape_transport(self, data): # ESC transport decoding state machine
        b = ord(data) #convert character to byte
        if b == 0x1B and self.bp_trans_prev_byte == 0x1B:
            #discard the second 0x1B
            self.bp_trans_prev_byte = 0 # anything different from 0x1B
            return
        self.baumprotocol_receive_statemachine(b)
        self.bp_trans_prev_byte = b

    def braille_out_send(self, cells, moduleId):
        if cells and moduleId:
            payload = bytearray()
            payload.extend(moduleId) 
            payload.append(0x00) #command wr
            payload.append(0x00) #first dots register
            payload.append(len(cells))  
            payload.extend(cells)
            self.send_packet(ord(BAUM_VP_DYNAMIC_DATA_BLOCK), payload) 	

    def display(self, cells): 
        if self.mainModule and self.mainModule.output_handler:
            self.mainModule.output_handler(cells[:self.mainModule.number_cells], self.mainModule.module_id)
            if len(cells) == self.mainModule.number_cells:
                return 
        if self.statusModule and self.statusModule.output_handler:
            self.statusModule.output_handler(cells[self.mainModule.number_cells:(self.mainModule.number_cells + BAUM_STATUS_CELLS_NUMBER)], self.statusModule.module_id)
        if self.telephoneModule and self.telephoneModule.output_handler:
            self.telephoneModule.output_handler(cells[self.mainModule.number_cells + BAUM_STATUS_CELLS_NUMBER: self.mainModule.number_cells + BAUM_STATUS_CELLS_NUMBER + BAUM_TELEPHONE_CELLS_NUMBER], self.telephoneModule.module_id)

    gestureMap = inputCore.GlobalGestureMap({
        "globalCommands.GlobalCommands": {
            "activateBrailleSettingsDialog": ("br(baumVarioPro):d1+d2+d4+d5", "br(baumVarioPro):smc1", "br(baumVarioPro):tmc1"),
            "announceDropbox": ("br(baumVarioPro):d1+d4+d5",),
            "braille_cycleCursorShape": ("br(baumVarioPro):d2+d3+d5+d6", "br(baumVarioPro):smc4", "br(baumVarioPro):tmc4"),
            "braille_scrollBack": ("br(baumVarioPro):d2",),
            "braille_scrollForward": ("br(baumVarioPro):d5",),
            "braille_previousLine": ("br(baumVarioPro):d1", "br(baumVarioPro):wu1"),
            "braille_nextLine": ("br(baumVarioPro):d3", "br(baumVarioPro):wd1"),
            "braille_routeTo": ("br(baumVarioPro):routing",),
            "braille_toFocus": ("br(baumVarioPro):d5+d6", "br(baumVarioPro):wp1"),
            "braille_toggleShowCursor": ("br(baumVarioPro):d2+d4+d5", "br(baumVarioPro):smc3", "br(baumVarioPro):tmc3"),
            "braille_toggleTether": ("br(baumVarioPro):d1+d5", "br(baumVarioPro):smc2", "br(baumVarioPro):tmc2"),
            "createList": ("br(baumVarioPro):d1+d4+d5+d6",),
            "dateTime": ("br(baumVarioPro):d1+d3+d4+d6",),
            "kb:escape": ("br(baumVarioPro):wp3",),
            "kb:delete": ("br(baumVarioPro):d1+d6",),
            "kb:control+shift+escape": ("br(baumVarioPro):d1+d2+d3+d4",),	
            "kb:enter": ("br(baumVarioPro):d1+d2+d3+d5", "br(baumVarioPro):d2+d3+d5", "br(baumVarioPro):wp2", "br(baumVarioPro):wp4"),
            "kb:control+home": ("br(baumVarioPro):d1+d4",),
            "kb:downArrow": ("br(baumVarioPro):d2+d3", "br(baumVarioPro):wd2"),
            "kb:upArrow": ("br(baumVarioPro):d1+d2", "br(baumVarioPro):wu2"),
            "kb:alt+F4": ("br(baumVarioPro):d3+d4",),
            "kb:control+end": ("br(baumVarioPro):d3+d6",),
            "kb:tab": ("br(baumVarioPro):d6", "br(baumVarioPro):wd3"),
            "kb:shift+tab": ("br(baumVarioPro):d4", "br(baumVarioPro):wu3"),
            "kb:shift+f10": ("br(baumVarioPro):d3+d5",),
            "kb:windows+tab": ("br(baumVarioPro):wd4",),
            "kb:shift+windows+tab": ("br(baumVarioPro):wu4",),
            "kb:mediaprevtrack": ("br(baumVarioPro):d1+d4+d6",),
            "kb:medianexttrack": ("br(baumVarioPro):d3+d4+d6",),
            "kb:mediaplaypause": ("br(baumVarioPro):d2+d4",),
            "kb:mediastop": ("br(baumVarioPro):d2+d5+d6",),
            "kb:volumeUp": ("br(baumVarioPro):d3+d4+d5",),
            "kb:volumeDown": ("br(baumVarioPro):d1+d2+d6",),
            "kb:control": ("br(baumVarioPro):tn0",),
            "kb:numpad1": ("br(baumVarioPro):tmk1",),
            "kb:numpad2": ("br(baumVarioPro):tmk2",),
            "kb:numpad3": ("br(baumVarioPro):tmk3",),
            "kb:numpad4": ("br(baumVarioPro):tmk4",),
            "kb:numpad5": ("br(baumVarioPro):tmk5",),
            "kb:numpad6": ("br(baumVarioPro):tmk6",),
            "kb:numpad7": ("br(baumVarioPro):tmk7",),
            "kb:numpad8": ("br(baumVarioPro):tmk8",),
            "kb:numpad9": ("br(baumVarioPro):tmk9",),
            "kb:numpad0": ("br(baumVarioPro):tmk0",),
            "kb:numpadMultiply": ("br(baumVarioPro):tmk*",),
            "kb:numpadDivide": ("br(baumVarioPro):tmk#",),
            "sayAll": ("br(baumVarioPro):d1+d2+d3",),
            "showGui": ("br(baumVarioPro):d1+d3+d4", "br(baumVarioPro):d1+d2+d3+d4+d5", "br(baumVarioPro):tn*+tn0"),
            "speakForeground": ("br(baumVarioPro):d1+d2+d4",),
            "reportFormatting": ("br(baumVarioPro):d2+d4+d5+d6",),
            "speechMode": ("br(baumVarioPro):d2+d5",),	
            "toggleInputHelp": ("br(baumVarioPro):d1+d2+d5",),
            "toggleShowCursor": ("br(baumVarioPro):d2+d3+d6",),
            "reportStatusLine": ("br(baumVarioPro):d3+d5+d6", "br(baumVarioPro):tn*+tn9"),	
            "reportCurrentSelection": ("br(baumVarioPro):d1+d3+d6",),	
            "review_nextCharacter": ("br(baumVarioPro):twr", "br(baumVarioPro):thsr", "br(baumVarioPro):tn6"),
            "review_previousCharacter": ("br(baumVarioPro):twl", "br(baumVarioPro):thsl", "br(baumVarioPro):tn4"),
            "review_currentWord": ("br(baumVarioPro):twp", "br(baumVarioPro):thsp", "br(baumVarioPro):tn5"),
            "review_nextWord": ("br(baumVarioPro):tn#+tn6", "br(baumVarioPro):tn3"),
            "review_previousWord": ("br(baumVarioPro):tn9", "br(baumVarioPro):tn#+tn4"),
            "review_startOfLine": ("br(baumVarioPro):tn1",),
            "review_currentLine": ("br(baumVarioPro):tvsp", "br(baumVarioPro):tn0"),
            "review_nextLine": ("br(baumVarioPro):d4+d6", "br(baumVarioPro):tvsd", "br(baumVarioPro):tn2"),
            "review_previousLine": ("br(baumVarioPro):d1+d3", "br(baumVarioPro):tvsu", "br(baumVarioPro):tn8"),
            "review_top": ("br(baumVarioPro):d1+d4", "br(baumVarioPro):tn*+tn8"),
            "review_bottom": ("br(baumVarioPro):d3+d6", "br(baumVarioPro):tn#+tn2", "br(baumVarioPro):tn*+tn2"),
            "review_sayAll": ("br(baumVarioPro):tn*+tn3",),
            "reviewMode_next": ("br(baumVarioPro):d2+d3+d4+d6",),
            "reviewMode_previous": ("br(baumVarioPro):d1+d3+d5+d6",),
            "moveMouseToNavigatorObject": ("br(baumVarioPro):tn#+tn1",),
            "moveNavigatorObjectToMouse": ("br(baumVarioPro):d4+d5", "br(baumVarioPro):tn#+tn1",  "br(baumVarioPro):tn#+tn0"),
            "navigatorObject_current": ("br(baumVarioPro):tn#+tn5",),
            "navigatorObject_toFocus": ("br(baumVarioPro):tn7",),
            "navigatorObject_next": ("br(baumVarioPro):d1+d3+d5",),
            "navigatorObject_previous": ("br(baumVarioPro):d2+d4+d6",),
            "navigatorObject_firstChild": ("br(baumVarioPro):d2+d3+d4",),
            "navigatorObject_parent": ("br(baumVarioPro):d1+d5+d6",),	
            "leftMouseClick": ("br(baumVarioPro):tn#+tn7",),
            "toggleLeftMouseButton": ("br(baumVarioPro):tn#+tn8",),
            "rightMouseClick": ("br(baumVarioPro):tn#+tn9",),
            "say_battery_status": ("br(baumVarioPro):d3+d4+d5+d6",),
            "title": ("br(baumVarioPro):tn*+tn7", "br(baumVarioPro):d2+d3+d4+d5"),
        },
    })

class InputGesture(braille.BrailleDisplayGesture, brailleInput.BrailleInputGesture):

    source = BrailleDisplayDriver.name

    def __init__(self, keysDown):
        super(InputGesture, self).__init__()

        self.keysDown = dict(keysDown)

        self.keyNames = names = set()
        for group, groupKeysDown in keysDown.items():
            if group == BAUM_ROUTING_KEYS:
                for index in range(braille.handler.display.numCells):
                    if groupKeysDown & (1 << index):
                        self.routingIndex = index
                        names.add("routing")
                        break
            else:
               for index, name in enumerate(KEY_NAMES[group]):
                    if groupKeysDown & (1 << index):
                        names.add(name)
        self.id = "+".join(names)
