class base::python (
    $maverick_python = true,
    $python_version = "v3.7.2"
) {

    # Install custom python 3.7
    if $maverick_python == true {
        # If ~/var/build/.install_flag_python exists, skip pulling source and compiling
        if ! ("install_flag_python" in $installflags) {
            warning("Optimized Python3 will be built and can take a long time, please be patient..")
            # Pull python from git
            ensure_packages(["libffi-dev", "libncurses5-dev", "libncursesw5-dev", "libgdbm-dev", "libgdbm-compat-dev", "libreadline-dev", "tk-dev", "liblzma-dev"])
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
                require     => [ Package["libffi-dev"], Package["tk-dev"], ],
            } ->
            exec { "python-make":
                environment => ["LDFLAGS=-Wl,-rpath,/srv/maverick/software/python/lib"],
                command     => "/usr/bin/make >/srv/maverick/var/log/build/python.make.out 2>&1",
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
            } ->
            file { "/srv/maverick/var/build/.install_flag_python":
                ensure      => present,
            }
        }

        # This makes the custom python3 the system default, at least by path invocation
        file { "/etc/profile.d/99-maverick-python-path.sh":
            content => "export PATH=/srv/maverick/software/python/bin:\$PATH",
            owner   => "root",
            group   => "root",
            mode    => "0644",
        }
    }

    # Install python using python module
    class { "python":
        version    => 'system',
        dev        => 'present',
        virtualenv => 'present',
        gunicorn   => 'absent',
        before     => Package["python-setuptools"],
    }
    ensure_packages(["python-setuptools", "virtualenvwrapper", "python-numpy", "python-lockfile", "python-daemon"])
    # Install python3 packages
    ensure_packages(["python3-yaml"])
    # Install pylint for cloud9 linting
    ensure_packages(["pylint", "pylint3"])

    # Install basic useful python modules
    install_python_module { 'pip-numpy':
        pkgname     => 'numpy',
        ensure      => present,
        pip_provider => 'pip3',
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
