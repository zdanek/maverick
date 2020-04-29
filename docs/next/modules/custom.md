# Custom Modules
Maverick provides locations in the Maverick system that are excluded from upstream commits to place custom modules.  Custom modules can be easily created to extend functionality as an alternative to modifying core Maverick modules, or else by creating additional functionality

## Module Layout
Creating a module is very simple.  A basic module layout would like like:
```
  files/
  manifests/
    index.pp
  templates/
```
*manifests/index.pp* is the 'auto executed' manifest for the module, ie. the default entry point.  Any number of additional manifests can be added, including in subdirectories of *manifests/*, but must be either directly referenced or referenced from init.pp in order to be used.  
In fact, neither *files/* nor *templates/* need to actually exist but they are commonly used to store and provide files and content.  The *files/* directory holds static files that can then be placed anywhere on the system, whereas *templates/* can contain ruby templates (.erb files) that can be placed anywhere on the system but are variable interpolated before being placed in the system.  For example, *files/* could be used to store and provide a shell script or a systemd service manifest, whereas *templates/* is commonly used to store config files that are 'filled in' by the manifests using variables and conditionals before being placed in the system destination.


## Custom Module Locations
Custom modules can be created by adding directories in one of two places:
 - *~/code/maverick/custom-modules* (Recommended location for custom modules)
 - *~/software/maverick/manifests/custom-modules* (This directory is excluded from git commits)

## Adding Custom Modules to Maverick :id=add-custom-module
?> To activate the custom modules, add the custom class name to ~/config/maverick/localconf.json, eg:  
```json
"classes": [
    "custom_module_class_name"
]
```

## Example Custom Modules

### Simple custom module to add a software component
A simple example of a custom module is to download and install a piece of software from github.  This is a very developer common action on any computer, in particular UAV companion computers.  We will use the fiducial marker project AprilTag as an example.

First we create a blank module layout in *~/code/maverick/custom-modules/custom_apriltag:  
```
manifests/
  index.pp
```
ie. just create a single file in a new directory structure:  
```bash
mkdir -p ~/code/maverick/custom-modules/custom_apriltag/manifests
touch ~/code/maverick/custom-modules/custom_apriltag/manifests/index.pp
```

Now add the declarative puppet code to *manifests/index.pp* that will download, compile and install the software:  
```puppet
class custom_apriltag (
  $github_repo = "https://github.com/AprilRobotics/apriltag",
  $github_branch = "3.1.1",
) {
    oncevcsrepo { "apriltag-gitclone":
        gitsource   => $github_repo,
        revision    => $github_branch,
        dest        => "/srv/maverick/var/build/apriltag",
    } ->
    exec { "apriltag-cmake":
        user        => "mav",
        command     => "/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/apriltag . >/srv/maverick/var/log/build/apriltag.cmake.out 2>&1",
        cwd         => "/srv/maverick/var/build/apriltag",
        creates     => "/srv/maverick/var/build/apriltag/CMakeFiles",
    } ->
    exec { "apriltag-make":
        user        => "mav",
        timeout     => 0,
        command     => "/usr/bin/make >/srv/maverick/var/log/build/apriltag.make.out 2>&1",
        cwd         => "/srv/maverick/var/build/apriltag",
        creates     => "/srv/maverick/var/build/apriltag/apriltag_demo",
    } ->
    exec { "apriltag-install":
        user        => "mav",
        timeout     => 0,
        environment => ["PREFIX=/srv/maverick/software/apriltag"],
        command     => "/usr/bin/make install >/srv/maverick/var/log/build/apriltag.install.out 2>&1",
        cwd         => "/srv/maverick/var/build/apriltag",
        creates     => "/srv/maverick/software/apriltag/bin/apriltag_demo",
    }
}
```

The custom module must be declared to the main puppet config otherwise it will not be included.  Add the custom class name to localconf classes (*~/config/maverick/localconf.json*):  
```json
"classes": [ 
    "custom_apriltag"
]
```

Now run `maverick configure`, and you should get something like the following output:
```bash
[flight] [mav@maverick-raspberry ~/var/build]$ maverick configure

Maverick - UAV Companion Computer System - Version 1.2.0-beta

WARNING: Maverick is using branch:master, not stable

Environment marker set and is being used to set maverick environment: flight
Maverick Environment:    flight
Proceeding to update system configuration - please be patient, this can take a while..

Notice: Compiled catalog for maverick-raspberry.local in environment flight in 15.73 seconds
Notice: /Stage[main]/Custom_apriltag/Oncevcsrepo[apriltag-gitclone]/File[/srv/maverick/var/build/apriltag]/ensure: created
Notice: /Stage[main]/Custom_apriltag/Oncevcsrepo[apriltag-gitclone]/Vcsrepo[/srv/maverick/var/build/apriltag]/ensure: created
Notice: /Stage[main]/Custom_apriltag/Exec[apriltag-cmake]/returns: executed successfully
Notice: /Stage[main]/Custom_apriltag/Exec[apriltag-make]/returns: executed successfully
Notice: /Stage[main]/Custom_apriltag/Exec[apriltag-install]/returns: executed successfully
Notice: Applied catalog in 133.36 seconds

Maverick finished, happy flying :)
```

And you should now see the software installed into *~/software/apriltag*:  
```bash
[flight] [mav@maverick-raspberry ~/var/build]$ ls -l ~/software/apriltag/
total 16
drwxr-xr-x 2 mav mav 4096 Apr 29 14:40 bin
drwxr-xr-x 3 mav mav 4096 Apr 29 14:40 include
drwxr-xr-x 3 mav mav 4096 Apr 29 14:40 lib
drwxr-xr-x 3 mav mav 4096 Apr 29 14:40 share

[flight] [mav@maverick-raspberry ~/var/build]$ ~/software/apriltag/bin/apriltag_demo
Summary
hamm     0     0     0     0     0     0     0     0     0     0        0.000     0
```

This fits in nicely with the Maverick Configuration Management model, so `maverick configure` can be run as many times as you want and it will detect if the software is still installed, and if so then skip the actions and do nothing.  In the future you can deploy your ~/code/maverick/custom-modules/custom_apriltag module to any other Maverick install.  You may want to keep ~/code/maverick/custom-modules/custom_apriltag under your own github project for quick deployment.

### Second SITL Module
Below is a custom module sample for creating a swarm of SITL instances.
```Python
class sample_sitl_swarm (
) {

    # Create a custom APSITL instance (copter by default)
    maverick_dev::apsitl { "custom_sitl":
        instance_name       => "custom",
        instance_number     => 1,
    }

    # Create a swarm of SITL instances through simple iteration
    $instances = {
        2 => { "instance_name" => "copter2", "vehicle_type" => "copter", "ros_instance" => true, "api_instance" => true, "mavlink_proxy" => "mavlink-router" },
        3 => { "instance_name" => "copter3", "vehicle_type" => "copter", "ros_instance" => true, "api_instance" => true, "mavlink_proxy" => "mavlink-router" },
        4 => { "instance_name" => "copter4", "vehicle_type" => "copter", "ros_instance" => false, "api_instance" => false, "mavlink_proxy" => "mavlink-router" },
        5 => { "instance_name" => "copter5", "vehicle_type" => "copter", "ros_instance" => false, "api_instance" => false, "mavlink_proxy" => "mavlink-router" },
        6 => { "instance_name" => "plane1", "vehicle_type" => "plane", "ros_instance" => true, "api_instance" => true, "mavlink_proxy" => "mavlink-router" },
        7 => { "instance_name" => "plane2", "vehicle_type" => "plane", "ros_instance" => false, "api_instance" => false, "mavlink_proxy" => "mavlink-router" },
        8 => { "instance_name" => "rover1", "vehicle_type" => "rover", "ros_instance" => true, "api_instance" => true, "mavlink_proxy" => "mavlink-router" },
        9 => { "instance_name" => "sub1", "vehicle_type" => "sub", "ros_instance" => true, "api_instance" => true, "mavlink_proxy" => "mavlink-router" }
    }
    $instances.each |Integer $instance_number, Hash $instance_vars| {
        maverick_dev::apsitl { $instance_vars['instance_name']:
            instance_number => $instance_number,
            *               => $instance_vars,
        }
    }

}
```
To add the Sample SITL Swarm Module, add sample_sitl_swarm class to ~/config/maverick/localconf.json in the format described in [Adding Custom Modules to Maverick](#add-custom-module) Section.
```json
"classes": [ “sample_sitl_swarm"
]
```
