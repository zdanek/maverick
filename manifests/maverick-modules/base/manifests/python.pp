class base::python (
    $maverick_python = true,
    $python_version = "v3.7.3"
) {

    # Install custom python 3.7
    # It is STRONGLY recommended not to disable this - Maverick and associated projects target python 3.7+
    if $maverick_python == true {
        # If ~/var/build/.install_flag_python exists, skip pulling source and compiling
        if ! ("install_flag_python" in $installflags) {
            warning("Optimized Python3 will be built and can take a long time, please be patient..")
            # Pull python from git
            ensure_packages(["libffi-dev", "libncurses5-dev", "libncursesw5-dev", "libgdbm-dev", "libreadline-dev", "tk-dev", "liblzma-dev", "libbz2-dev", "libsqlite3-dev"])
            oncevcsrepo { "git-python":
                gitsource   => "https://github.com/python/cpython.git",
                dest        => "/srv/maverick/var/build/python",
                revision    => $python_version,
            } ->
            exec { "python-configure":
                environment => ["LDFLAGS=-Wl,-rpath,/srv/maverick/software/python/lib"],
                command     => "/srv/maverick/var/build/python/configure --prefix=/srv/maverick/software/python --enable-optimizations --with-lto --enable-shared >/srv/maverick/var/log/build/python.configure.out 2>&1",
                cwd         => "/srv/maverick/var/build/python",
                creates     => "/srv/maverick/var/build/python/Makefile",
                user        => "mav",
                timeout     => 0,
                require     => [ Package["libffi-dev"], Package["tk-dev"], Package["libbz2-dev"], ],
            } ->
            exec { "python-make":
                environment => ["LDFLAGS=-Wl,-rpath,/srv/maverick/software/python/lib", "PROFILE_TASK=-m test.regrtest --pgo -j${::processorcount}"],
                command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/var/log/build/python.make.out 2>&1",
                cwd         => "/srv/maverick/var/build/python",
                creates     => "/srv/maverick/var/build/python/libpython3.so",
                user        => "mav",
                timeout     => 0,
            } ->
            exec { "python-make-install":
                environment => ["LDFLAGS=-Wl,-rpath,/srv/maverick/software/python/lib"],
                command     => "/usr/bin/make install >/srv/maverick/var/log/build/python.make.install.out 2>&1",
                cwd         => "/srv/maverick/var/build/python",
                creates     => "/srv/maverick/software/python/bin/python3",
                user        => "mav",
                timeout     => 0,
            } ->
            exec { "python-pip-upgrade":
                cwd         => "/srv/maverick/software/python",
                command     => "/srv/maverick/software/python/bin/pip3 install --upgrade pip",
                user        => "mav",
            } ->
            file { "/srv/maverick/var/build/.install_flag_python":
                ensure      => present,
            }
        }

        # This makes the custom python3 the system default, at least by path invocation
        file { "/etc/profile.d/99-maverick-python-path.sh":
            owner   => "root",
            group   => "root",
            mode    => "0644",
            content => 'NEWPATH="/srv/maverick/software/python/bin"; if [ -n "${PATH##*${NEWPATH}}" -a -n "${PATH##*${NEWPATH}:*}" ]; then export PATH=$NEWPATH:$PATH; fi',
        }

        # Remove the pip binary from custom python, it interferes with system python2
        file { "/srv/maverick/software/python/bin/pip":
            ensure  => symlink,
            target  => "/srv/maverick/software/python/bin/pip3",
        }
        
        # Create symlinks to python, to override old python2
        file { "/srv/maverick/software/python/bin/python":
            ensure  => symlink,
            target  => "/srv/maverick/software/python/bin/python3",
        }
        file { "/usr/bin/python":
            ensure  => symlink,
            target  => "/srv/maverick/software/python/bin/python3",
            force   => true,
        }
    }

    # Install python using python module
    class { "python":
        version    => 'system',
        dev        => 'present',
        virtualenv => 'present',
        gunicorn   => 'absent',
    }

    # Install basic useful python modules
    install_python_module { 'pip-numpy':
        pkgname     => 'numpy',
        ensure      => present,
    } ->
    install_python_module { 'pip-daemon':
        pkgname     => 'python-daemon',
        ensure      => present,
    } ->
    install_python_module { 'pip-lockfile':
        pkgname     => 'lockfile',
        ensure      => present,
    } ->
    install_python_module { "pytest":
        pkgname     => "pytest",
        ensure      => present,
        timeout     => 0,
    } ->
    install_python_module { "pip-yaml":
        pkgname     => "PyYAML",
        ensure      => present,
        timeout     => 0,
    } ->
    install_python_module { "pip-pylint":
        pkgname     => "pylint",
        ensure      => present,
        timeout     => 0,
    } ->
    install_python_module { "pip-virtualenvwrapper":
        pkgname     => "virtualenvwrapper",
        ensure      => present,
        timeout     => 0,
    } ->
    install_python_module { "pip-black":
        pkgname     => "black",
        ensure      => present,
        timeout     => 0,
    } ->
    install_python_module { "pip-flake8":
        pkgname     => "flake8",
        ensure      => present,
        timeout     => 0,
    }

    # Install PyRIC and netifaces, python modules necessary to run maverick --netinfo
    install_python_module { 'pip-pyric':
        pkgname     => 'PyRIC',
        ensure      => present,
    } ->
    install_python_module { 'pip-netifaces':
        pkgname     => 'netifaces',
        ensure      => present,
    } ->
    install_python_module { 'pip-future':
        pkgname     => 'future',
        ensure      => present,
    }

}
