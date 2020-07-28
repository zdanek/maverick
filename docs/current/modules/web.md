# Web Module

The Web module adds a web interface and web services to Maverick.  It is currently in very early stages, and a temporary (not very pretty!) webpage index is in place.  However, there is an emerging structure in place and great things planned for the future, including a web configuration front-end for Maverick and a rich web based GCS and realtime video interface.

This module is under active construction, this page will be updated shortly.

## Cloud9

Maverick includes the brilliant [Cloud9 IDE](https://c9.io/?redirect=0) as an easy way of exploring and editing files on your companion computer.  It is very useful for altering config files, as well as full blown coding.

### Active
It is installed, configured and automatically run at boot.  If you do not want it to run at boot, set a localconf parameter:
`"maverick_cloud9::cloud9_active": false`

### Web Port
By default it runs on port 6101 in a web browser.  To change this, set a localconf parameter:
`"maverick_cloud9::webport": "1234"`

### Base Path (Document Root)
Maverick is almost entirely self contained within /srv/maverick on the OS filesystem, so that is the default base path for Cloud9.  To change this, set a localconf parameter:  
`"maverick_cloud9::basepath": "/home/joebloggs"`
