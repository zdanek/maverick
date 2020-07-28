# Desktop Module

All of the OS Images that are distributed os far for Maverick contain a complete desktop environment (except for the raspberrylite image), but it is turned off by default.

- Joule (Ubuntu Desktop)
- Odroid (Ubuntu Desktop)
- Raspberry (Pixel Desktop)

The desktop module is currently very simple, with only two parameters:

### Install
To ensure the desktop software is installed, set localconf parameter:  
`"maverick_desktop::install": true`

To ensure the desktop software is *not* installed, set localconf parameter:  
`"maverick_desktop::install": false`

Note that this parameter is set to _undefined_ by default, which means it does nothing and leaves the desktop software state as it already is.

### Activate
To enable the desktop, set localconf parameter:  
`"maverick_desktop::enable": true`

**Note: Enabling the desktop uses more resources and also activates various power management features which can put the computer into sleep mode and is highly undesirable mid-flight.  It is recommended to only enable the desktop while doing desktop development, and to disable it for any actual flights.  All Maverick OS images come with desktop disabled for this reason.**

To disable the desktop (which is the default), set localconf parameter:
`"maverick_desktop::enable": false`

### Temporary stop/start
To start the desktop without enabling it at boot, use the systemd target:  
`sudo systemctl isolate graphical.target`

To stop the desktop without disabling it at boot, use the systemd target:  
`sudo systemctl isolate multi-user.target`
