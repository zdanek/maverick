#!/usr/bin/env python
# This fact finds the current branch of the git repo it is running from

import os,re,subprocess

# Define main data container
branch = subprocess.check_output(["/usr/bin/git", "branch"])
branch = re.sub("\* ", "", branch.strip())
print "gitbranch=" + str(branch)