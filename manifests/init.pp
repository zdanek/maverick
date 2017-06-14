### Declare defines here so they can be accessed everywhere
define speak ($message = "", $level = "") {
    notify { $name:
        message     => $message,
        loglevel    => $level,
    }
}

# Workaround for slow pip checks: https://github.com/stankevich/puppet-python/issues/291
define install_python_module ($ensure, $pkgname=$title, $virtualenv=undef, $timeout=undef, $owner=undef, $env="global", $version=undef) {
  $module_version = $python_modules[$env][$pkgname]
  if $python_modules {
    case $ensure {
      'present': {
        unless $pkgname in $python_modules[$env] {
            notice("Installing pip: ${pkgname}")
            python::pip { $title:
                pkgname => "${pkgname}",
                ensure => 'present',
                virtualenv => $virtualenv,
                owner => $owner,
                timeout => $timeout
            }
        }
      }
      'atleast': {
        if empty($module_version) {
            notice("Installing pip: ${pkgname}")
            python::pip { $title:
                pkgname => "${pkgname}",
                ensure => 'present',
                virtualenv => $virtualenv,
                owner => $owner,
                timeout => $timeout
            }
        } elsif versioncmp($version, $module_version) > 0 {
            notice("Upgrading Pip module: ${pkgname}, installed version ${module_version} is less than requested version ${version}")
            python::pip { $title:
                pkgname => "${pkgname}",
                ensure => 'latest',
                virtualenv => $virtualenv,
                owner => $owner,
                timeout => $timeout,
                install_args => "--upgrade",
            }
        }
      }
      'absent': {
        if $pkgname in $python_modules {
          python::pip { "${pkgname}":
            pkgname => "${pkgname}",
            ensure => absent,
            owner   => $owner,
            timeout => $timeout
          }
        }
      }

    }
  }
  else {
    python::pip { $title: ensure => $ensure }
  }
}

# oncevcsrepo is a wrapper to only call vcsrepo if a clone doesn't exist at all locally.
# Otherwise, vcsrepo gets called for each define each puppet run, which can take a long time (and require internet access)
define oncevcsrepo ($gitsource, $dest, $revision="master", $owner="mav", $group="mav", $submodules=false, $depth=1, $force=false) {
    # This depends on gitfiles fact, declared in maverick-modules/base/facts.d/gitrepos.py
    $gitrepos = split($gitrepos, ',')
    if ! ("${dest}/.git" in $gitrepos) {
        notice("oncevcsrepo: ${dest} git repo doesn't exist locally, cloning may take a while..")
        file { "${dest}":
            ensure      => directory,
            owner       => "${owner}",
            group       => "${group}",
            mode        => "755",
        } ->
        vcsrepo { "${dest}":
            ensure		=> present,
            provider 	=> git,
            source		=> "${gitsource}",
            revision	=> "${revision}",
            owner		=> "${owner}",
            group		=> "${group}",
            depth       => $depth,
            submodules  => $submodules,
            require     => File["${dest}"],
            force       => $force,
        }
    }
}

# Ensure a correct value exists for a field in a specified file, where the field is on a separate line like an ini conf file
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

# This adds an entire line to a file if it doesn't already exist
define confline ($file, $line) {
    if $file and $line {
        # Firstly, if the value doesn't exist, add it
        exec { "confline-add-${file}-${line}":
            command     => "/bin/echo '${line}' >> ${file}",
            unless      => "/bin/grep -e '^${line}' ${file}"
        }
    }
}

# Ensure a correct value exists for a field in a specified file, where the field is within a line of other values, like a grub/boot line
define lineval ($file, $field, $oldvalue, $newvalue, $linesearch) {
    if $file and $field and $oldvalue and $newvalue and $linesearch {
        # Change the value if it already exists
        exec { "lineval-$file-$field-change":
            command     => "/bin/sed ${file} -i -r -e 's/${field}=${oldvalue}/${field}=${newvalue}/'",
            onlyif      => "/bin/grep '${field}=${oldvalue}' ${file}",
        }
        exec { "lineval-$file-$field-add":
            command     => "/bin/sed ${file} -i -r -e '/${linesearch}/ s/$/ ${field}=${newvalue}/'",
            unless      => "/bin/grep '${field}' ${file}",
        }
    }
}

# Define a function that only stops services if they exist
define service_stop () {
    if $operatingsystem == "Ubuntu" and ($operatingsystemrelease == "15.04" or $operatingsystemrelease == "15.10" or $operatingsystemrelease == "16.04" or $operatingsystemrelease == "16.10" or $operatingsystemrelease == "17.04" or $operatingsystemrelease == "17.10") {
        $_provider = "systemd"
    } else {
        $_provider = undef
    }
    if ! empty(grep($::installed_services, $name)) {
        service { $name:
            ensure      => stopped,
            enable      => false,
            provider    => $_provider,
        }
    }
}

# Define a function that only starts services if they exist
define service_start () {
    if $operatingsystem == "Ubuntu" and ($operatingsystemrelease == "15.04" or $operatingsystemrelease == "15.10" or $operatingsystemrelease == "16.04" or $operatingsystemrelease == "16.10" or $operatingsystemrelease == "17.04" or $operatingsystemrelease == "17.10") {
        $_provider = "systemd"
    } else {
        $_provider = undef
    }
    if ! empty(grep($::installed_services, $name)) {
        service { $name:
            ensure      => running,
            enable      => true,
            provider    => $_provider,
        }
    }
}

# Define a function that only starts services if they exist
define service_wrapper ($enable, $ensure) {
    if $operatingsystem == "Ubuntu" and ($operatingsystemrelease == "15.04" or $operatingsystemrelease == "15.10" or $operatingsystemrelease == "16.04" or $operatingsystemrelease == "16.10" or $operatingsystemrelease == "17.04" or $operatingsystemrelease == "17.10") {
        $_provider = "systemd"
    } else {
        $_provider = undef
    }
    if ! ($ensure == "stopped" and empty(grep($::installed_services, $name))) {
        service { $name:
            ensure      => $ensure,
            enable      => $enable,
            provider    => $_provider,
        }
    }
}

### End of defines

node default {
    # This is a 'catch-all' node statement.
    # Instead of declaring nodes, or using an ENC, we use hiera to assign 
    #  classes and data to nodes in a hierarchical, segregated fashion.
}

lookup('classes', {merge => unique}).include