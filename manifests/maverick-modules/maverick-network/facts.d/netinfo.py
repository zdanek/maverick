#!/usr/bin/env python
# This fact extracts network info for network interfaces

import os, re, sys, subprocess
sys.path.insert(0, '/usr/local/examples')
try:
    import iwconfig
    import netifaces
    from udevnet import Udevnet
    netinfo = {}
except:
    print "netinfo_present=no"
    sys.exit(1)

class Netinfo(object):
    def __init__(self, _if):
        self.data = {}
        self._if = _if
        self.udevnet = Udevnet()
        self.udevnet.runall()


    def getinfo(self):
        self.data['macaddress'] = netifaces.ifaddresses(self._if)[netifaces.AF_LINK][0]['addr']
        try:
            self.data['ipaddress'] = netifaces.ifaddresses(self._if)[netifaces.AF_INET][0]['addr']
        except:
            self.data['ipaddress'] = None
        try:
            self.data['vendorstr'] = self.udevnet.data[self._if+"_id_vendor_from_database"]
        except:
            self.data['vendorstr'] = None
        try:
            self.data['vendoroui'] = self.udevnet.data[self._if+"_id_oui_from_database"]
        except:
            self.data['vendoroui'] = None
        try:
            self.data['vendor'] = self.udevnet.data[self._if+"_id_vendor"]
        except:
            self.data['vendor'] = None
        # Hack for onboard raspberry devices
        if type(self.data['vendoroui']) is str:
            if re.search("^Raspberry", self.data['vendoroui']):
                self.data['vendor'] = "RaspberryPi"

        try:
            self.data['driver'] = self.udevnet.data[self._if+"_id_net_driver"]
        except:
            try:
                self.data['driver'] = self.udevnet.data[self._if+"_id_usb_driver"]
            except:
                self.data['driver'] = None
        try:
            self.data['model'] = self.udevnet.data[self._if+"_id_model_id"]
        except:
            self.data['model'] = None
        try:
            self.data['modelstr'] = self.udevnet.data[self._if+"_id_model_from_database"]
        except:
            self.data['modelstr'] = None
        try:
            self.data['netname'] = self.udevnet.data[self._if+"_id_net_name_from_database"]
        except:
            try:
                self.data['netname'] = self.udevnet.data[self._if+"_id_net_name_onboard"]
            except:
                try:
                    self.data['netname'] = self.udevnet.data[self._if+"_id_net_name_slot"]
                except:
                    try:
                        self.data['netname'] = self.udevnet.data[self._if+"_id_net_name_path"]
                    except:
                        try:
                            self.data['netname'] = self.udevnet.data[self._if+"_id_net_name_mac"]
                        except:
                            self.data['netname'] = None
        try:
            self.data['type'] = self.udevnet.data[self._if+"_devtype"]
            if self.data['type'] == "wlan": self.data['type'] = "Wireless"
        except:
            try:
                if re.search("^en", self.data['netname']):
                    self.data['type'] = "Ethernet"
                elif re.search("^wl", self.data['netname']):
                    self.data['type'] = "Wireless"
                else:
                    self.data['type'] = None
            except:
                self.data['type'] = None
        try:
            _wifi = iwconfig.Wireless(self._if)
            self.data['mode'] = _wifi.getMode()
            self.data['bitrate'] = _wifi.getBitrate()
        except:
            self.data['mode'] = None
            self.data['bitrate'] = None

    def runall(self):
        pass
    
#If we're being called as a command, instantiate and report
if __name__ == '__main__':
    ifs = netifaces.interfaces()
    for _if in ifs:
        _netinfo = Netinfo(_if)
        _netinfo.getinfo()
        for key,val in sorted(_netinfo.data.items()):
           print "netinfo_"+_if+"_%s=%s" % (key, val)
