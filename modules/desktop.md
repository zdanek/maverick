# Desktop Module

All of the OS Images that are distributed os far for Maverick contain a complete desktop environment (except for the raspberrylite image), but it is turned off by default.

- Joule (Ubuntu Desktop)
- Odroid (Ubuntu Desktop)
- Raspberry (Pixel Desktop)

The desktop module is currently very simple, with a single parameter:

### Active
To enable the desktop, set a localconf paramter:  
`"maverick_desktop::enable": true`

**Note: Enabling the desktop uses more resources and also activates various power management features which can put the computer into sleep mode.  It is recommended to only enable the desktop while doing desktop development, and to disable it for any actual flights.**

### Temporary stop/start
To start the desktop without enabling it at boot, use the systemd target:  
`sudo systemctl isolate graphical.target`

To stop the desktop without disabling it at boot, use the systemd target:  
`sudo systemctl isolate multi-user.target`
