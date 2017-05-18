# Base Module

The *base* module contains resources that setup the build system itself, and core OS configuration.  It is always run in all environments.  As well as performing some base configuration on the system, it also sets up the structure of Maverick itself, and defines stages (ordering) in which certain resources are applied.  This is particularly important in the initial bootstrap of Maverick, and most of this is pre-set in Maverick and cannot/should not be changed.

The *base* module controls the following resources:
- Setting up the core Maverick environment
- Setting up the core Puppet environment
- System locale (timezone, language)
- Core system software/packages
- Core system services
- Console setup (banners, prompts)

---
## Locale
### System Locale
The default locale is 'en_GU.UTF8'.  To change it, set a localconf parameter:  
`"base::locale::locale": "fr_FR.UTF8"`
If the language pack/locale data does not already exist on the system, it will be installed.  Unless specified, it will do this by installing '*locales-all*' package which includes all locales and is quite large.  If you would like to specify a particular package that contains just the locale you want, you can specify it by setting localconf parameter:  
`"base::locale::language_pack": "language-pack-fr"`

### System Timezone
The default timezone is 'Europe/London'.  A list of available timezones can be displayed: `timedatectl list-timezones`.  
To change it, set a localconf parameter:  
`"base::locale::timezone": "Australia/Sydney"`

---
## System Passwords
### Mav User
Almost everything in the Maverick environment is run or performed as the *mav* system user.  The default password is 'wingman'.  To change this, set a localconf parameter with a new password hash:  
`"base::users::mav_password": "$6$YuXyoBZR$cR/cNLGZV.Y/nfW6rvK//fjnr84kckI1HM0fhPnJ3MVVlsl7UxaK8vSw.bM4vTlkF4RTbOSAdi36c5d2hJ9Gj1"`
By default, the *mav* user does not need to use a password to perform root actions.  To change this, set localconf parameter:  
`"base::users::mav_sudopass": true`
### Root User
To set the system root password, set localconf parameter:  
`"base::users::root_password": "$6$YuXyoBZR$cR/cNLGZV.Y/nfW6rvK//fjnr84kckI1HM0fhPnJ3MVVlsl7UxaK8vSw.bM4vTlkF4RTbOSAdi36c5d2hJ9Gj1"`
The password hashes can be created with the mkpasswd tool:  
`mkpasswd  -m sha-512 -s`
