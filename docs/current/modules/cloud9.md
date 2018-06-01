# Cloud9 Module

Maverick includes the brilliant [Cloud9 IDE](https://c9.io/?redirect=0) as an easy way of exploring and editing files on your companion computer.  It is very useful for altering config files, as well as full blown coding.

### Active
It is installed, configured and automatically run at boot.  If you do not want it to run at boot, set a localconf parameter:
`"maverick_cloud9::cloud9_active": false`

### Web Port
By default it runs on port 6789 in a web browser.  To change this, set a localconf parameter:
`"maverick_cloud9::webport": "1234"`

### Base Path (Document Root)
Maverick is almost entirely self contained within /srv/maverick on the OS filesystem, so that is the default base path for Cloud9.  To change this, set a localconf parameter:  
`"maverick_cloud9::basepath": "/home/joebloggs"`
