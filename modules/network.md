# Network Module

Networking is a complex subject and the setup and configuration of networking is particularly complex, not least because of the varieties of hardware and network types.  There are many ways of configuring networking, but for an embedded UAV Companion Computer that is inherently headless (without display or keyboard), reliability and connectivity is paramount.  Therefore Maverick uses a deliberately simple networking setup.  Modern Debian/Ubuntu OS use managers such as *Connection Manager* or *Network Manager*, however these tend to be complex in nature and really intended for human/desktop use where multiple transient connections (such as multiple wifi networks) are required to be configured.

Maverick, by default, disables these Managers and uses the very simple but reliable Debian/Ubuntu networking model of configuring a single file - */etc/network/interfaces*.  This has proved to be simpler and more reliable to manage, particularly from the automated manifests that Maverick uses.

## Network Interfaces
Any number of network interfaces can be defined in localconf, with a variety of parameters that change how each interface is configured.  The chosen model is to specify the MAC address and an interface name for each interface, so they are easy to identify.  For example, a simple wifi interface for normal connectivity might be defined:  
```
"maverick_network::interfaces": {
        "wman0": {
            "type":	"wireless",
            "macaddress": "00:c2:c6:e2:45:xx", // put your wifi mac address here, can be obtained from 'maverick netinfo'
            "ssid":    "MySSID",
            "psk":    "66ab1efd34a940390e024872739778958c3350cbexxxxx"
        }
}
```
This uses the MAC address specified to fix the interface name 'wman0', and set the wifi SSID and PSK to authenticate to the AP/router.  There are different interface 'modes', by default the 'managed' mode is used, however wireless type interfaces can also use 'ap' (access point) and 'monitor' mode (used for wifibroadcast).

A second interface/wifi adapter could be used as an access point (AP), by simply defining:  
```
"maverick_network::interfaces": {
        "wman0": {
            "type":	"wireless",
            "macaddress": "00:c2:c6:e2:45:xx",
            "ssid":    "MySSID",
            "psk":    "66ab1efd34a940390e024872739778958c3350cbexxxxx"
        },
        "wap0": {
            "type": "wireless",
            "mode": "ap",
            "macaddress": "00:c2:c2:ff:32:xx"
        }
}
```
This would automatically setup the second wifi card/adapter as an AP, with the default SSID 'Maverick' and the default passphrase 'ifeeltheneed'.  It sets up the necessary software and configuration to handle authentication and IP address allocation, so any computer can immediately connect to it.

A third interface could be configured for wifibroadcast:
```
"maverick_network::interfaces": {
        "wman0": {
            "type":	"wireless",
            "macaddress": "00:c2:c6:e2:45:xx",
            "ssid":    "MySSID",
            "psk":    "66ab1efd34a940390e024872739778958c3350cbexxxxx"
        },
        "wap0": {
            "type": "wireless",
            "mode": "ap",
            "macaddress": "00:c2:c2:ff:32:xx"
        },
        "wbcast0": {
            "type": "wireless",
            "mode": "monitor",
            "macaddress": "00:b3:a9:ee:10:xx"
        }
}
```

### Managed Interface
There are additional parameters to the 'managed' mode interface.  Here is an interface definition that uses all of them:  
```
"wman0": {
    "type":	"wireless",
    "macaddress": "00:c2:c6:e2:45:xx",
    "addressing": "static",
    "ipaddress": "192.168.1.10",
    "netmask": "255.255.255.0",
    "gateway": "192.168.1.254",
    "dns_nameservers": "192.168.1.254 192.168.1.1",
    "ssid":    "MySSID",
    "psk":    "66ab1efd34a940390e024872739778958c3350cbexxxxx"
}
```
For ethernet interfaces, just change 'type' to 'ethernet' and ommit ssid/psk.

### AP Interface
The example above to setup an AP interface is quite simple.  There are additional parameters that can be set however:
```
"wap0": {
    "type": "wireless",
    "mode": "ap",
    "macaddress": "00:c2:c2:ff:32:xx",
    "ssid": "Maverick",
    "driver": "nl80211",
    "channel": 1,
    "hw_mode": "g",
    "disable_broadcast_ssid": false,
    "dhcp_range": "192.168.10.10,192.168.10.50",
    "dhcp_leasetime": "24h"
}
```

## Network components
### Avahi
Avahi is automatically installed and configured, in order to provide 'zeroconf' multicast DNS.  Under normal networking circumstances, the Maverick system should respond to any requests for <hostname>.local, eg. maverick-raspberry.local.  Avahi 'publish-addresses' is disabled by default but can be enabled by setting localconf parameter:  
`"maverick_network::avahi::explicit_naming"`

### IPv6
By default, Maverick leaves ipv6 configuration as is.  However, ipv6 can sometimes cause problems and so support can be deliberately disabled by setting localconf parameter:  
`"maverick_network::ipv6": false`  

### Predictable Naming
'Predictable Naming' is a feature of newer Linux kernels that enumerates network devices into names that are 'predictable' based on the type of hardware and bus/address location.  These names can be useful in certain circumstances, but for embedded systems where the most common network interfaces are USB wireless interfaces, these names are useless and awkward.  So this feature is disabled by default, and traditional simple naming like 'eth0' and 'wlan0' is used instead.  To restore predictable naming, set localconf parameter:  
`"maverick_network::predictable": true`  
