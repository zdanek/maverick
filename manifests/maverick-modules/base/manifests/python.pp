class base::python (
) {

    # Install python using python module
    class { "python":
        version    => 'system',
        dev        => 'present',
        virtualenv => 'present',
        gunicorn   => 'absent',
    } ->
    # Install python packages
    package { ["python-setuptools", "virtualenvwrapper", "python-numpy", "python-lockfile", "python-daemon"]:
        ensure      => present,
    } ->
    # Install python3 packages
    package { ["python3-numpy", "python3-yaml"]:
        ensure      => present,
    } ->
    # Install pylint for cloud9 linting
    package { ["pylint", "pylint3"]:
        ensure      => present,
    }
    
    # Need to install/upgrade pip to a known version using easy_install, which is the only method that works reliably.
    exec { "upgrade-pip":
        command     => "sudo easy_install -U pip==9.0.1",
        unless      => "ls /usr/local/lib/python2.7/dist-packages/pip-9.0.1*.egg",
        path        => ["/usr/local/bin", "/usr/bin", "/bin"],
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