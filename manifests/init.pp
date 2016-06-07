### Declare defines here so they can be accessed everywhere
define speak ($message = "", $level = "") {
    notify { $name:
        message     => $message,
        loglevel    => $level,
    }
}

# oncevcsrepo is a wrapper to only call vcsrepo if a clone doesn't exist at all locally.
# Otherwise, vcsrepo gets called for each define each puppet run, which can take a long time (and require internet access)
define oncevcsrepo ($gitsource, $dest, $revision="master", $owner="mav", $group="mav", $submodules=true) {
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

# Ensure a correct value exists for a field in a specified file
define confval ($file, $field, $value) {
    if $file and $field and $value {
        # Firstly, if the value doesn't exist, add it
        exec { "confval-add-${file}-${field}":
            command     => "/bin/echo '${field}=${value}' >> ${file}",
            unless      => "/bin/grep -e '^${field}=' ${file}"
        }
        # Otherwise, update if necessary
        exec { "confval-update-${file}-${field}":
            command     => "/bin/sed ${file} -i -r -e 's/${field}=.*/${field}=${value}/'",
            onlyif      => "/bin/grep -e '^${field}' ${file} | /bin/grep -v '${field}=${value}'"
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
