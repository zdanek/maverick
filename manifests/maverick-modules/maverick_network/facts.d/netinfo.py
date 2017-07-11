#!/usr/bin/env python
# This fact extracts network info for network interfaces

import os, re, sys, subprocess
sys.dont_write_bytecode = True # This is to prevent .pyc files in facts.d directory
sys.path.insert(0, '/usr/local/examples')
try:
    import netifaces
    import pyric             # pyric errors
    import pyric.pyw as pyw  # iw functionality
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
        try:
            self.data['macaddress'] = netifaces.ifaddresses(self._if)[netifaces.AF_LINK][0]['addr']
        except:
            self.data['macaddress'] = None
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

        # Stop here if we don't have a wireless card
        if self.data['type'] != "Wireless":
            return

        # Retrieve wireless info
        try:
            _ifobj = pyw.getcard(self._if)
            _ifinfo = pyw.ifinfo(_ifobj)
            _devinfo = pyw.devinfo(_ifobj)
            _physinfo = pyw.phyinfo(_ifobj)
            _linkinfo = pyw.link(_ifobj)
        except:
            pass
        try:
            self.data['isup'] = pyw.isup(_ifobj)
        except:
            self.data['isup'] = None
        try:
            self.data['blocked'] = pyw.isblocked(_ifobj)
        except:
            self.data['blocked'] = None
        try:
            self.data['mode'] = _devinfo['mode']
        except:
            self.data['mode'] = None
        try:
            self.data['modes'] = _physinfo['modes']
        except:
            self.data['modes'] = None
        try:
            self.data['bands'] = _physinfo['bands']
        except:
            self.data['bands'] = None
        try:
            self.data['standards'] = pyw.devstds(_ifobj)
        except:
            self.data['standards'] = None
        try:
            self.data['freqs'] = pyw.devfreqs(_ifobj)
        except:
            self.data['freqs'] = None
        try:
            self.data['txpower'] = pyw.txget(_ifobj)
        except:
            self.data['txpower'] = None
        try:
            self.data['chans'] = pyw.devchs(_ifobj)
        except:
            self.data['chans'] = None
        try:
            self.data['reg'] = pyw.regget(_ifobj)
        except:
            self.data['reg'] = None
        try:
            self.data['chipset'] = _ifinfo['chipset']
        except:
            self.data['chipset'] = None
        try:
            self.data['state'] = _linkinfo['stat']
        except:
            self.data['state'] = None
        try:
            self.data['ssid'] = _linkinfo['ssid']
        except:
            self.data['ssid'] = None
        try:
            self.data['chw'] = _devinfo['CHW']
        except:
            self.data['chw'] = None
        try:
            self.data['frequency'] = _devinfo['RF']
        except:
            self.data['frequency'] = None
        try:
            self.data['rss'] = _linkinfo['rss']
        except:
            self.data['rss'] = None
        try:
            self.data['wtx'] = _linkinfo['tx']
        except:
            self.data['wtx'] = None
        try:
            self.data['wrx'] = _linkinfo['rx']
        except:
            self.data['wrx'] = None

    def runall(self):
        pass
    
#If we're being called as a command, instantiate and report
if __name__ == '__main__':
    try:
        ifs = pyw.interfaces()
    except pyric.error as e:
        print "Error running netinfo, pyric not available"
        sys.exit(1)
    
    print "netinfo_present=yes"
    with open ("/etc/hostname", "r") as etc_hostname:
        data=etc_hostname.readlines()
    if data:
        print "netinfo_etchostname="+str(data[0].rstrip())
    
    print "netinfo_interfaces="+",".join(ifs)
    for _if in ifs:
        _netinfo = Netinfo(_if)
        _netinfo.getinfo()
        for key,val in sorted(_netinfo.data.items()):
           print "netinfo_"+_if+"_%s=%s" % (key, val)
