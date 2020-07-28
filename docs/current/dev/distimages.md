# Distribution Images

Although Maverick can be used by bootstrapping without any need for distribution images, as the project gets larger and the build times get longer, it is much more convenient to create and provide distribution images.  These images can be downloaded and flashed for immediate use.  This is particularly useful for slower platforms such as Raspberry Pi, which can take upwards of a day to complete a full developer build.  It is expected that the large majority of users will download and use pre-built images for common supported platforms.

Creating distribution images takes place in two stages:
 - _Prepare_ the donor OS, that is a running instance of Maverick that will be the base for the distribution image.
 - _Create_ the distribution image from the prepared donor OS.

---
## Preparation
There is a single script that should be executed on the running Maverick instance that will be used as the donor.  Although it lives inside the Maverick base module, it has nothing to do with Maverick itself other than preparation.

!> Warning:  Running the preparation script _will delete ALL data on the donor OS_.  All logs, data, configuration, customizations etc will be removed.

Run the preparation script:  
```
sudo ~/software/maverick/manifests/maverick-modules/base/files/preprelease.sh
```

The output should look something like this:  
```
[dev] [mav@maverick-raspberry ~]$ sudo ~/software/maverick/manifests/maverick-modules/base/files/preprelease.sh
Warning: Unit file of maverick-visiond.service changed on disk, 'systemctl daemon-reload' recommended.
Cleaning dpkg and apt
Removing logs, config and data
Recreating gstreamer cache

Running maverick to regenerate any removed files or config

Maverick - UAV Companion Computer System

Environment marker set and is being used to set maverick environment: dev
Maverick Environment:    dev
Proceeding to update system configuration - please be patient, this can take a while..

Notice: Compiled catalog for maverick-raspberry.home in environment dev in 27.23 seconds
Notice: /Stage[bootstrap]/Base::Maverick/File[/srv/maverick/.gitconfig]/ensure: defined content as '{md5}87800b72eb2d142639bf903e8a989b42'
Notice: /Stage[bootstrap]/Base::Maverick/File[/srv/maverick/config/maverick/localconf.json]/owner: owner changed 'root' to 'mav'
Notice: /Stage[bootstrap]/Base::Maverick/File[/srv/maverick/config/maverick/localconf.json]/group: group changed 'root' to 'mav'
Notice: /Stage[bootstrap]/Base::Maverick/File[/srv/maverick/.config]/ensure: created
Notice: /Stage[bootstrap]/Base::Maverick/File[/srv/maverick/.config/user-dirs.conf]/ensure: defined content as '{md5}354f0930fe7abfb33682b057ef000909'
Notice: /Stage[main]/Maverick_vision::Visiond/File[/srv/maverick/config/vision/maverick-visiond.conf]/ensure: defined content as '{md5}23c0fb15f6047dcdf236ff9dffd540d7'
Notice: /Stage[main]/Maverick_vision::Vision_landing/File[/srv/maverick/config/vision/vision_landing.conf]/ensure: defined content as '{md5}c765fa05498030ccfe4958604b978ae4'
Notice: /Stage[main]/Maverick_vision::Vision_seek/File[/srv/maverick/config/vision/vision_seek.conf]/ensure: defined content as '{md5}72f815c7c95d355625b3e080641013d8'
...
Notice: /Stage[main]/Maverick_dev::Sitl/Service_wrapper[maverick-sitl]/Service[maverick-sitl]/ensure: ensure changed 'stopped' to 'running'
Notice: /Stage[main]/Maverick_dev::Sitl/Maverick_ros::Rosmaster[sitl]/Service_wrapper[maverick-rosmaster@sitl]/Service[maverick-rosmaster@sitl]/ensure: ensure changed 'stopped' to 'running'
Notice: /Stage[main]/Maverick_dev::Sitl/Maverick_ros::Mavros[sitl]/File[/srv/maverick/config/ros/mavros-sitl.conf]/ensure: defined content as '{md5}a8bf1f1c62d8cec64befd563a51a6b35'
Notice: /Stage[main]/Maverick_dev::Sitl/Maverick_ros::Mavros[sitl]/Service_wrapper[maverick-mavros@sitl]/Service[maverick-mavros@sitl]/ensure: ensure changed 'stopped' to 'running'
Notice: Applied catalog in 54.16 seconds

Maverick finished, happy flying :)

Maverick preparation complete, shutting down cleanly
```

Once the preparation script has been run to completion, proceed to shut the system down and the next stage of Image Creation can proceed:  
`sudo shutdown -h now` 

## Image Creation
This stage is easier when the donor system runs on an SD card, such as Raspberry Pi or Jetson Nano.  In this case, the SD card can be removed from the donor system and placed in another system, and the entire SD card is then truncated, compressed and imaged.  Any desktop or laptop Linux system is a good choice.  
The imaging system does not have to be bootstrapped or running Maverick, but like the preparation script the imaging script lives in the Maverick tree so it at least has to be cloned onto the imaging system:  
`git clone https://github.com/goodrobots/maverick.git --depth 1`  

### Creating a Raspberry Pi Image
Run the imaging script:  
`sudo maverick/manifests/maverick-modules/base/files/createimage_raspberry.sh` (from the clone above)  
 or  
`sudo ~/software/maverick/manifests/maverick-modules/base/files/createimage_raspberry.sh` (from a running Maverick system)  

When run without any arguments it will print usage and a display of block devices.  From this list, the donor system must be identified.  It is _VERY IMPORTANT_ to identify the correct block device at this point, otherwise the imaging system may be inadvertently wiped!  For this reason it is highly recommended to use a VM or other system that can be easily rebuilt or reset, to minimize against disaster.  

```
[dev] [mav@maverick-joule ~]$ sudo ~/software/maverick/manifests/maverick-modules/base/files/createimage_raspberry.sh
Missing required arguments:
 - Source Disk Device (eg. mmcblk0)
 - Destination filepath (eg. /var/tmp/raspberry-os.img)

List of available devices:
NAME         MAJ:MIN RM  SIZE RO TYPE MOUNTPOINT
mmcblk0rpmb  179:40   0    4M  0 disk
mmcblk0boot0 179:8    0    4M  1 disk
mmcblk0boot1 179:16   0    4M  1 disk
mmcblk0      179:0    0 14.7G  0 disk
├─mmcblk0p1  179:1    0  243M  0 part /boot/efi
└─mmcblk0p2  179:2    0 14.4G  0 part /
mmcblk1      179:48   0 29.3G  0 disk
├─mmcblk1p1  179:49   0   41M  0 part
└─mmcblk1p2  179:50   0 29.2G  0 part
mmcblk0gp0   179:24   0    4M  0 disk
mmcblk0gp3   179:32   0    4M  0 disk
```

A good way of identifying the correct block device for removable media such as SD card from a Raspberry Pi or Odroid, is to run the above script with and then without the card inserted.  Above is with the donor Raspberry card inserted, below is with it removed:  
```
[dev] [mav@maverick-joule ~]$ sudo ~/software/maverick/manifests/maverick-modules/base/files/createimage.sh
Missing required arguments:
 - Source Disk Device (eg. mmcblk0)
 - Destination filepath (eg. /var/tmp/raspberry-os.img)

List of available devices:
NAME         MAJ:MIN RM  SIZE RO TYPE MOUNTPOINT
mmcblk0rpmb  179:40   0    4M  0 disk
mmcblk0boot0 179:8    0    4M  1 disk
mmcblk0boot1 179:16   0    4M  1 disk
mmcblk0      179:0    0 14.7G  0 disk
├─mmcblk0p1  179:1    0  243M  0 part /boot/efi
└─mmcblk0p2  179:2    0 14.4G  0 part /
mmcblk0gp0   179:24   0    4M  0 disk
mmcblk0gp3   179:32   0    4M  0 disk
```

From this, it is very clear that the donor SD card is mmcblk1.  Another hint is _never_ to use a block device that has any active mountpoints, such as mmcblk0 above (which is the root filesystem of the running imaging system).

Once the correct block device has been identified, verified and double checked, run the script with arguments to perform the actual image creation:  
`sudo ~/software/maverick/manifests/maverick-modules/base/files/createimage.sh mmcblk1 /var/tmp/maverick-1.0.6-raspberrypi.img`  
Note that the destination image is suffixed with '.xz', as it is compressed on the fly.  
The script will ask for a partition to shrink.  Most SBC/embedded systems will have a single main partition, and this shrinking allows a much faster imaging process and a much smaller end distribution image.  It also allows any size SD/MMC storage to be used by the end user, as Maverick will expand automatically to fill any size storage.  
```
[dev] [mav@maverick-joule ~]$ sudo ~/software/maverick/manifests/maverick-modules/base/files/createimage.sh mmcblk1 /var/tmp/maverick-1.0.6-raspberrypi.img
Source Disk: mmcblk1
Source Partitions:
NAME        MAJ:MIN RM  SIZE RO TYPE MOUNTPOINT
mmcblk1     179:48   0 29.3G  0 disk
├─mmcblk1p1 179:49   0   41M  0 part
└─mmcblk1p2 179:50   0 29.2G  0 part
Specify partition to shrink:
```
In the above example (Maverick Raspberry Pi donor system), mmcblk1p2 is clearly the main partition, so enter '2' as the answer.  The imaging process will then calculate the necessary storage dimensions/geometries with some overhead and (safely) shrink the partition and filesystem, before proceeding to create the image.  In the above instance it shrinks a 29.2Gb partition (on a 32Gb SD card) down to about 8Gb (which is further compressed during the imaging process to about 2Gb):  
```
[dev] [mav@maverick-joule ~]$ sudo ~/software/maverick/manifests/maverick-modules/base/files/createimage.sh mmcblk1 /var/tmp/maverick-1.0.6-raspberrypi.img
Source Disk: mmcblk1
Source Partitions:
NAME        MAJ:MIN RM  SIZE RO TYPE MOUNTPOINT
mmcblk1     179:48   0 29.3G  0 disk
├─mmcblk1p1 179:49   0   41M  0 part
└─mmcblk1p2 179:50   0 29.2G  0 part
Specify partition to shrink: 2
Partition data (mmcblk1p2): mmcblk1p2 mmcblk1p2 179:50 ext4 b105f9a8-f450-4976-8ac8-69053f57bab4 0x83 f21688de-02 128 0 0 1 29.2G root disk brw-rw---- 0 512 0 512 512 0 cfq 128 part 3145728 4M 2.1G 0 0B 0 mmcblk1 block:mmc:mmc_host:pci
Checking partition:
fsck from util-linux 2.27.1
e2fsck 1.42.13 (17-May-2015)
/dev/mmcblk1p2: clean, 234439/1894464 files, 1747104/7665408 blocks
Total blocks: 7665408, Used blocks: 1747104, Free blocks: 5918304
New size of resized filesystem = Used blocks (1747104) + 5% = 2096524

Resizing partition filesystem
resize2fs 1.42.13 (17-May-2015)
Resizing the filesystem on /dev/mmcblk1p2 to 2096524 (4k) blocks.
Begin pass 3 (max = 234)
Scanning inode table          XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
Begin pass 4 (max = 25228)
Updating inode references     XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
The filesystem on /dev/mmcblk1p2 is now 2096524 (4k) blocks long.


Current size of partition: Start=92160s, End=61415423s
New size of resized partition = Start=92160s, End=16772192s

Resizing partition
Warning: Shrinking a partition can cause data loss, are you sure you want to continue?
Information: You may need to update /etc/fstab.

parted /dev/mmcblk1 unit s resizepart 2 16864352 yes

Creating image file /var/tmp/maverick-1.0.6-raspberrypi.img.xz from resized disk mmcblk1
  --- %         24.1 MiB / 87.4 MiB = 0.275   1.5 MiB/s       0:58
```  

Note: the imaging process is very slow as it uses a small blocksize to minimise the effect of any bad blocks in the source media.  It is possible to alter the script to use larger blocksizes, which significantly speeds up the image creation process, but at the expense of possible corrupted distribution image.  

### Non-removable media
If the donor system is on non-removeable media (eg. onboard MMC storage such as Intel Joule uses), then a different process is necessary to create the image.  Prepare the donor system as normal (run the preparation script as above).

- Download clonezilla alternate, burn to SD card/USB.  Boot using default option.
- Either drop to shell and add a DOS partition on rest of SD card, or plug in a USB stick to use as an image store.

TODO: Work in Progress
- Create image from disk - device_image -> local_dev -> savedisk
- Create image from disk - device_image -> local_dev -> recovery-iso-zip -- put 'ask_user' for restore device - choose both iso and zip

Stage 1:
 - device_image
 - local_dev
 - Choose partition to mount for image storage
 - savedisk
 - name something like maverick-1.0.6-joule
 - Choose deafult -q2 priority
 - Choose default advanced parameters
 - Use -z2p parallel bzip2 compression
 - Use 4096000000 (very large number) to force a single image file
 - fsck-y auto repair filesystem
 - Yes, check saved image
 - senc, don't encrypt image
 - -p choose reboot/shutdown

 Stage 2:
 - rerun2 (keep_image_repository_/home/partimag_mounted)
 - device_image
 - skip (Use existing /home/partimag)
 - beginner
 - recovery-iso-zip
 - replace mmcblk1 with 'ask_user' for 'device to be restored'
 - -scr no, skip Checking
 - -p choose
 - both - iso and zip
