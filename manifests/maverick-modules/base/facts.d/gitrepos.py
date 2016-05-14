#!/usr/bin/env python
# This fact finds the paths of vcsrepos within maverick-modules, to aid conditional vcsrepo
# otherwise vcsrepo updates each repo every puppet run, which is unnecessary and very slow

import os,re

# Define main data container
gitfiles = []
for root, dirs, files in os.walk("/srv/maverick"):
    for file in files:
        filename = os.path.join(root, file)
        if re.search(".git/HEAD", filename):
            gitfiles.append(filename)

# Finally, print the data out in the format expected of a fact provider
if gitfiles:
    #for key,val in sorted(gitfiles.items()):
    #    print "camera_%s=%s" % (key, val)
    print "gitrepos="+str(",".join(gitfiles))