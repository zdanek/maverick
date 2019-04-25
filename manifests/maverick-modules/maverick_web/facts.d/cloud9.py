#!/usr/bin/env python3

import os.path
if os.path.isfile("/srv/maverick/.c9/installed"):
    print("cloud9_installed=yes")
else:
    print("cloud9_installed=no")