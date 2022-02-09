

import smbus, struct


a_speedl = 0
a_speedr = 1
a_encl1  = 2
a_encr1  = 6
a_batt   = 10
a_curl   = 11
a_curr   = 12
a_vers   = 13
a_cmd    = 16

v_rst_enc = 0x20


def read_int(bus, slv_addr, reg_addr):
    v = [bus.read_byte_data(slv_addr, reg_addr+i) for i in range(0,4)]
    a = int(struct.unpack('>l', bytearray(v))[0])
    return a


class MD25:
    def __init__(self):
        self.bus_id, self.addr = 1, 0x58
        self.bus = smbus.SMBus(self.bus_id)
        self.bat_cal = 0.1
        self.reset()

    def reset(self):
        self.bus.write_byte_data(self.addr, a_cmd, v_rst_enc)

    def read(self):
        self.bat = self.bat_cal*self.bus.read_byte_data(self.addr, a_batt)
        self.enc_l = read_int(self.bus, self.addr, a_encl1)
        self.enc_r = read_int(self.bus, self.addr, a_encr1)

    def write(self, vl, vr):
        for a, v in [(a_speedl,vl), (a_speedr, vr)]:
            self.bus.write_byte_data(self.addr, a, v+0x80)
        
