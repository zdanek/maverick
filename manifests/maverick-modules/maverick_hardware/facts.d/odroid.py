#!/usr/bin/env python3
# This fact extracts as much hardware information out of an odroid as possible

import os, re, sys, subprocess

class Odroid(object):
    def __init__(self):
        self.data = {'present': 'no'}

    def cpudata(self):
        count = 0
        # Define main data container
        f = open('/proc/cpuinfo', 'r')
        for line in f:
            r = re.search('^(.*)\s+:\s+(.*)', line)
            if not r or not r.groups(0) or not r.groups(0)[0] or not r.groups(0)[1]: continue
            (key,val) = r.groups(0)[0],r.groups(0)[1]
            if key == "Hardware":
                self.data['model'] = val
                if re.search('ODROID', val) or re.search('EXYNOS', val):
                    self.data['present'] = 'yes'
                if re.search('SAMSUNG EXYNOS', val):
                    self.data['model'] = "Odroid XU4"
            elif key == "Revision":
                self.data['revision'] = val
            elif key == "Serial":
                self.data['serial'] = val
            elif key == "processor":
                count += 1
        self.data['cpucores'] = count
        f.close()
        f = open('/proc/meminfo', 'r')
        for line in f:
            r = re.search('^(.*):\s+(.*)', line)
            if not r or not r.groups(0) or not r.groups(0)[0] or not r.groups(0)[1]: continue
            (key,val) = r.groups(0)[0],r.groups(0)[1]
            if key == "MemTotal":
                self.data['memory'] = val
            elif key == "SwapTotal":
                self.data['swap'] = val
        f.close()

    def storagedata(self):
        # Obtain the SD card size from proc
        f = open('/proc/partitions', 'r')
        for line in f:
            if re.search("mmcblk[01]$", line):
                self.data['sdsize'] = int(line.split()[2]) / 1024
        f.close()

    def kernel(self):
        try:
            if os.path.isdir('/srv/maverick/var/build/linux'):
                self.data['kernel4x_dir'] = "yes"
            else:
                self.data['kernel4x_dir'] = "no"
            with open ("/srv/maverick/var/build/linux/.kernelrelease", "r") as kernelrelease:
                kr=kernelrelease.readlines()
            if kr:
                self.data['kernel4x_release'] = kr[0].rstrip()
        except:
            self.data['kernel4x_dir'] = "no"
            self.data['kernel4x_release'] = "no"

        try:
            if os.path.exists("/media/boot/boot.ini-k3bak") and os.path.exists("/media/boot/config-k3bak") and os.path.exists("/media/boot/exynos5422-odroidxu3.dtb-k3bak") and os.path.exists("/media/boot/uInitrd-k3bak") and os.path.exists("/media/boot/zImage-k3bak"):
                self.data['kernel3x_backups'] = "yes"
            else:
                self.data['kernel3x_backups'] = "no"
        except:
            self.data['kernel3x_backups'] = "no"

        self.data['kernel_current'] = "no"
        try:
            try:
                klines = subprocess.check_output(["/usr/bin/mkimage", "-l", "/media/boot/uInitrd"]).decode("utf-8").split("\n")
            except subprocess.CalledProcessError as e:
                klines = None
            for kline in klines:
                if re.search("Image Name", kline):
                    kver = re.split('.*initrd.img-', kline)[1]
                    self.data['kernel_current'] = kver
        except:
            pass

        if os.path.exists("/srv/maverick/var/build/linux/.install_flag"):
            self.data['kernel_install_flag'] = "yes"
        else:
            self.data['kernel_install_flag'] = "no"

    def runall(self):
        self.cpudata()
        self.storagedata()
        self.kernel()

#If we're being called as a command, instantiate and report
if __name__ == '__main__':
    odroid = Odroid()
    odroid.cpudata()
    if odroid.data['present'] == "no":
        print("odroid_present=no")
        sys.exit(1)
    odroid.storagedata()
    odroid.kernel()

    # Finally, print the data out in the format expected of a fact provider
    if odroid.data:
        for key,val in odroid.data.items():
            print("odroid_%s=%s" % (key, val))
