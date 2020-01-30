# @summary
#   Maverick_mavlink::Dronekit class
#   This class installs Dronekit and associated software.
#
# @example Declaring the class
#   This class is included from maverick_mavlink class and should not be included from elsewhere
#
class maverick_mavlink::dronekit {
    
    # Install dronekit globally (not in virtualenv) from pip
    install_python_module { "pip-dronekit-global":
        pkgname     => "dronekit",
        ensure      => atleast,
        version     => "2.9.2",
        timeout     => 0,
    }

}
