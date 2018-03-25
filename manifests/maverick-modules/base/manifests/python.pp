class base::python (
) {

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
    ensure_packages(["python3-numpy", "python3-yaml"])
    # Install pylint for cloud9 linting
    ensure_packages(["pylint", "pylint3"])

    # Need to install/upgrade pip to a known version using easy_install, which is the only method that works reliably.
    exec { "upgrade-pip":
        command     => "sudo easy_install -U pip==9.0.3",
        unless      => "ls /usr/local/lib/python2.7/dist-packages/pip-9.0.3*.egg",
        path        => ["/usr/local/bin", "/usr/bin", "/bin"],
        require     => Package["python-setuptools"],
    } ->
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