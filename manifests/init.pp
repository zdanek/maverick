### Declare defines here so they can be accessed everywhere
define speak ($message = "", $level = "") {
    notify { $name:
        message     => $message,
        loglevel    => $level,
    }
}

# oncevcsrepo is a wrapper to only call vcsrepo if a clone doesn't exist at all locally.
# Otherwise, vcsrepo gets called for each define each puppet run, which can take a long time (and require internet access)
define oncevcsrepo ($gitsource, $dest, $revision = "master", $owner = "mav", $group = "mav", $submodules = true) {
    # This depends on gitfiles fact, declared in maverick-modules/base/facts.d/gitrepos.py
    $gitrepos = split($gitrepos, ',')
    if ! ("${dest}/.git/HEAD" in $gitrepos) {
        warning("oncevcsrepo: ${dest} git repo doesn't exist locally, cloning may take a while..")
        file { "${dest}":
            ensure      => directory,
            owner       => "${owner}",
            group       => "${group}",
            mode        => 755,
        } ->
        vcsrepo { "${dest}":
            ensure		=> present,
            provider 	=> git,
            source		=> "${gitsource}",
            revision	=> "${revision}",
            owner		=> "${owner}",
            group		=> "${group}",
            submodules  => $submodules,
            require     => File["${dest}"]
        }
    }
}
### End of defines

node default {
    # This is a 'catch-all' node statement.
    # Instead of declaring nodes, or using an ENC, we use hiera to assign 
    #  classes and data to nodes in a hierarchical, segregated fashion.
}

hiera_include('classes')
