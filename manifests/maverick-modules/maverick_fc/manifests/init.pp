# @summary
#   Maverick_fc class
#   This class manages the flight controller environment, in particular mavlink proxy and services that connect to the flight controller.
#
# @example Declaring the class
#   This class is included from the environment manifests and is not usually included elsewhere.
#   It could be included selectively from eg. minimal environment.
#
# @param mavlink_proxy
#   Type of mavlink proxy to use, eg. mavlink-router, mavproxy or cmavnode
# @param mavlink_active
#   If true, the mavlink proxy will be activated and enabled at boot time
# @param mavlink_logging
#   If true and supported by the mavlink proxy software, then mavlink data will be logged to file
# @param mavlink_input
#   Path to serial device that connects to the FC.  This is different for each board, and is usually set in the board sample manifests (~/software/maverick/conf/sample-nodes)
# @param mavlink_inbaud
#   Baud rate for serial FC connection
# @param mavlink_inflow
#   If set to true, turns on hardware flow control on the FC serial connection.
# @param mavlink_startingtcp
#   Start TCP port - if more than one port then each one will be incremented from this starting value
# @param mavlink_tcpports
#   Number of TCP ports that the mavlink proxy will listen on
# @param mavlink_startingudp
#   Start UDP port - if more than one port then each one will be incremented from this starting value
# @param mavlink_udpports
#   Number of UDP ports to listen on
# @param mavlink_udpinports
#   Number of UDP In ports to listen on
# @param mavlink_serialout
#   If set, proxy mavlink data out on this serial port
# @param mavlink_outbaud
#   Baud rate of mavlink_serialout port
# @param mavlink_outflow
#   If mavlink_serialout port should use hardware flow control
# @param mavlink_replaceconfig
#   If set to true, this will overwrite the mavlink proxy config file.  If false, edits can be made directly to the config file without being managed/overwritten
# @param ros_instance
#   If true, create a separate ROS instance for this SITL instance
# @param rosmaster_active
#   If true, set this separate ROS instance active and enabled at boot time
# @param rosmaster_port
#   Define the port number that the ROS master will listen on.  This must be unique across all ROS instances.
# @param mavros_active
#   If true, the separate MAVROS instance will be activated and enabled at boot time
# @param mavros_startup_delay
#   This delay causes Mavros to wait before starting, to give ROS and SITL time to boot fully first.  Should be increased on slower boards/environments.
# @param api_instance
#   If true, create a separate maverick-api instance
# @param api_name
#   Descriptive name of this -api instance
# @param api_active
#   If true, this maverick-api instance will be activated and enabled at boot time
# @param virtualenv
#   If true, this will create a python virtual environment for FC/production
#
class maverick_fc (
    String $mavlink_proxy = "mavlink-router",
    Boolean $mavlink_active = true,
    Boolean $mavlink_logging = true,
    String $mavlink_input = "/dev/ttyAMA0",
    Integer $mavlink_inbaud = 115200,
    Boolean $mavlink_inflow = false,
    Boolean $mavlink_flow = false,
    Integer $mavlink_startingtcp = 5770,
    Integer $mavlink_tcpports = 3,
    Integer $mavlink_startingudp = 14570,
    Integer $mavlink_udpports = 3,
    Integer $mavlink_udpinports = 3,
    Optional[String] $mavlink_serialout = undef,
    Integer $mavlink_outbaud = 115200,
    Boolean $mavlink_outflow = false,
    Boolean $mavlink_replaceconfig = true,
    Boolean $ros_instance = true,
    Boolean $rosmaster_active = true,
    Integer $rosmaster_port = 11311,
    Boolean $mavros_active = true,
    Integer $mavros_startup_delay = 10,
    Boolean $api_instance = true,
    String $api_name = "Flight Controller",
    Boolean $api_active = false,
    Boolean $virtualenv = false,
) {

    if $virtualenv == true {
        # Install a virtual environment for dronekit fc
        file { "/srv/maverick/code/fc":
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => "755",
        } ->
        python::virtualenv { '/srv/maverick/code/fc':
            ensure       => present,
            version      => 'system',
            systempkgs   => false,
            distribute   => true,
            venv_dir     => '/srv/maverick/.virtualenvs/fc',
            owner        => 'mav',
            group        => 'mav',
            cwd          => '/srv/maverick/code/fc',
            timeout      => 0,
        } ->
        file { "/srv/maverick/.virtualenvs/fc/lib/python2.7/no-global-site-packages.txt":
            ensure  => absent
        }
        file { "/srv/maverick/var/log/mavlink-fc":
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
            mode        => "755",
        }
    }

    # Install default params config
    file { "/srv/maverick/config/mavlink/mavlink_params-fc.json":
        ensure      => absent,
    }

    ### Setup mavlink proxy
    if $mavlink_proxy == "mavproxy" {
        maverick_mavlink::cmavnode { "fc":
            inputaddress => $mavlink_input,
            inputbaud   => $mavlink_inbaud,
            inputflow   => $mavlink_inflow,
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
        } ->
        maverick_mavlink::mavlink_router { "fc":
            inputtype   => "serial",
            inputbaud   => $mavlink_inbaud,
            inputflow   => $mavlink_inflow,
            inputaddress => $mavlink_input,
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            logging     => $mavlink_logging,
        } ->
        maverick_mavlink::mavproxy { "fc":
            inputaddress => $mavlink_input,
            inputbaud   => $mavlink_inbaud,
            inputflow   => $mavlink_inflow,
            instance    => 2,
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            active      => $mavlink_active,
        }
    } elsif $mavlink_proxy == "cmavnode" {
        maverick_mavlink::mavproxy { "fc":
            inputaddress => $mavlink_input,
            inputbaud   => $mavlink_inbaud,
            inputflow   => $mavlink_inflow,
            instance    => 2,
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
        } ->
        maverick_mavlink::mavlink_router { "fc":
            inputtype   => "serial",
            inputaddress => $mavlink_input,
            inputbaud   => $mavlink_inbaud,
            inputflow   => $mavlink_inflow,
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            logging     => $mavlink_logging,
        } ->
        maverick_mavlink::cmavnode { "fc":
            inputaddress => $mavlink_input,
            inputbaud   => $mavlink_inbaud,
            inputflow   => $mavlink_inflow,
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            active      => $mavlink_active,
        }
    } elsif $mavlink_proxy == "mavlink-router" {
        maverick_mavlink::cmavnode { "fc":
            inputaddress => $mavlink_input,
            inputbaud   => $mavlink_inbaud,
            inputflow   => $mavlink_inflow,
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
        } ->
        maverick_mavlink::mavproxy { "fc":
            inputaddress => $mavlink_input,
            inputbaud   => $mavlink_inbaud,
            inputflow   => $mavlink_inflow,
            instance    => 2,
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
        } ->
        maverick_mavlink::mavlink_router { "fc":
            inputtype   => "serial",
            inputaddress => $mavlink_input,
            inputbaud   => $mavlink_inbaud,
            inputflow   => $mavlink_inflow,
            startingudp => $mavlink_startingudp,
            udpports    => $mavlink_udpports,
            udpinports  => $mavlink_udpinports,
            startingtcp => $mavlink_startingtcp,
            tcpports    => $mavlink_tcpports,
            serialout   => $mavlink_serialout,
            outbaud     => $mavlink_outbaud,
            outflow     => $mavlink_outflow,
            active      => $mavlink_active,
            logging     => $mavlink_logging,
        }
    }

    # Create status.d directory for maverick status`
    file { "/srv/maverick/software/maverick/bin/status.d/150.fc":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    } ->
    file { "/srv/maverick/software/maverick/bin/status.d/150.fc/__init__":
        owner       => "mav",
        content     => "Flight Controller",
    }
    file { "/srv/maverick/software/maverick/bin/status.d/150.fc/101.mavlink.status":
        owner   => "mav",
        content => "mavlink@fc,Mavlink (FC)\n",
    }
    
    # maverick_fc::ros_instance allows ros to be completely optional
    if $ros_instance == true {
        # Add a ROS master for FC
        maverick_ros::rosmaster { "fc":
            active  => $rosmaster_active,
            port    => $rosmaster_port,
        } ->
        maverick_ros::mavros { "fc":
            active              => $mavros_active,
            rosmaster_port      => $rosmaster_port,
            mavlink_port        => $mavlink_startingtcp,
            mavros_startup_delay => $mavros_startup_delay,
        }
        file { "/srv/maverick/software/maverick/bin/status.d/150.fc/102.rosmaster.status":
            owner   => "mav",
            content => "rosmaster@fc,Rosmaster (FC)\n",
        }
        file { "/srv/maverick/software/maverick/bin/status.d/150.fc/103.mavros.status":
            owner   => "mav",
            content => "mavros@fc,MAVROS (FC)\n",
        }
    }
    
    if $api_instance == true {
        # Create an API instance
        maverick_web::api { "api-fc":
            instance    => "fc",
            api_name    => $api_name,
            active      => $api_active,
            apiport     => 6800,
            rosport     => $rosmaster_port,
        }
        file { "/srv/maverick/software/maverick/bin/status.d/150.fc/104.api.status":
            owner   => "mav",
            content => "api@fc,MavAPI (FC)\n",
        }
    }
}
