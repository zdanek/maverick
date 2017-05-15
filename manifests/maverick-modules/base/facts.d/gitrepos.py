#!/usr/bin/env python
# This fact finds the paths of vcsrepos within maverick-modules, to aid conditional vcsrepo
# otherwise vcsrepo updates each repo every puppet run, which is unnecessary and very slow

import os,re,subprocess

# Define main data container
gitfiles = []
finddirs = subprocess.Popen(["find", "/srv/maverick", "/usr/local", "-type", "d", "-name", ".git"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
gitdirs = finddirs.communicate()[0].split()
for gitdir in gitdirs:
    gitfiles.append(gitdir)

# Finally, print the data out in the format expected of a fact provider
if gitfiles:
    print "gitrepos="+str(",".join(gitfiles))
else:
    print "gitrepos=false"
