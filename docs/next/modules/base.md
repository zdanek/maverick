# Base Module

The *base* module contains resources that setup the build system itself, and core OS configuration.  It is always run in all environments.  As well as performing some base configuration on the system, it also sets up the structure of Maverick itself, and defines stages (ordering) in which certain resources are applied.  This is particularly important in the initial bootstrap of Maverick, and most of this is pre-set in Maverick and cannot/should not be changed.

The *base* module controls the following resources:
- Setting up the core Maverick environment
- Setting up the core Puppet environment
- System locale (timezone, language)
- Core system software/packages
- Core system services
- Console setup (banners, prompts)

## Desktop
All of the OS Images that are distributed os far for Maverick contain a complete desktop environment (except for the raspberrylite image), but it is turned off by default except for the Desktop VM image (desktopvm).

### Install
To ensure the desktop software is installed, set localconf parameter:  
`"maverick_desktop::install": true`

To ensure the desktop software is *not* installed, set localconf parameter:  
`"maverick_desktop::install": false`

Note that this parameter is set to _undefined_ by default, which means it does nothing and leaves the desktop software state as it already is.

### Activate
To enable the desktop, set localconf parameter:  
`"maverick_desktop::enable": true`

?> Note: Enabling the desktop uses more resources and also activates various power management features which can put the computer into sleep mode and is highly undesirable mid-flight.  It is recommended to only enable the desktop while doing desktop development, and to disable it for any actual flights.  All Maverick OS images apart from the Desktop VM come with desktop disabled for this reason.

To disable the desktop (which is the default), set localconf parameter:  
`"maverick_desktop::enable": false`

### Temporary stop/start
To start the desktop without enabling it at boot, use the systemd target:  
`sudo systemctl isolate graphical.target`

To stop the desktop without disabling it at boot, use the systemd target:  
`sudo systemctl isolate multi-user.target`

## Locale
### System Locale
The default locale is 'en_GB.UTF8' (English Unicode).  To change it, set a localconf parameter:  
`"base::locale::locale": "fr_FR.UTF8"`
If the language pack/locale data does not already exist on the system, it will be installed.  Unless specified, it will do this by installing '*locales-all*' package which includes all locales and is quite large.  If you would like to specify a particular package that contains just the locale you want, you can specify it by setting localconf parameter:  
`"base::locale::language_pack": "language-pack-fr"`

### System Timezone
The default timezone is 'Europe/London'.  A list of available timezones can be displayed: `timedatectl list-timezones`.  
To change it, set a localconf parameter:  
`"base::locale::timezone": "Australia/Sydney"`

## System Passwords
### Mav User
Almost everything in the Maverick environment is run or performed as the *mav* system user.  The default password is 'wingman'.  To change this, set a localconf parameter with a new password hash:  
`"base::users::mav_password": "$6$YuXyoBZR$cR/cNLGZV.Y/nfW6rvK//fjnr84kckI1HM0fhPnJ3MVVlsl7UxaK8vSw.bM4vTlkF4RTbOSAdi36c5d2hJ9Gj1"`
By default, the *mav* user does not need to use a password to perform root actions through sudo.  To change this, set localconf parameter:  
`"base::users::mav_sudopass": true`
Git credentials for the mav user are cached by default. To turn this off (useful if multiple people are sharing the mav user account), set the localconf parameter:  
`"base::maverick::git_credentials_cache": false`  
This sets the credentials cache in ~/.gitconfig.  Note this config file is only set once by Maverick, so for this parameter to take effect, remove the file:  
`rm ~/.gitconfig`  

### Root User
The root password is _not_ managed by default.  To manage the root password, set localconf parameter:  
`"base::users::manage_root_password": true"`  
To set the system root password, set localconf parameter:  
`"base::users::root_password": "$6$YuXyoBZR$cR/cNLGZV.Y/nfW6rvK//fjnr84kckI1HM0fhPnJ3MVVlsl7UxaK8vSw.bM4vTlkF4RTbOSAdi36c5d2hJ9Gj1"`

The password hashes can be created with the mkpasswd tool:  
`mkpasswd  -m sha-512 -s`
