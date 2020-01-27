# @summary
#   Maverick_dev::ardupilot class
#   This class installs/manages the Ardupilot software/firmware environment.
#
# @example Declaring the class
#   This class is included from maverick_dev class and should not be included from elsewhere
#   It could be included selectively from eg. minimal environment.
#
# @param ardupilot_source
#   Git repo to clone, that is used to compile ardupilot (in ~/code/ardupilot).  Change this if you want to use a forked repo.
# @param ardupilot_setupstream
#   If true, set the upstream repo.  Useful if using a forked repo for upstream updates and PRs.
# @param ardupilot_upstream
#   Upstream Git repo.  Should usually not be changed.
# @param ardupilot_branch
#   Git branch to use when compiling Arudpilot.
# @param ardupilot_buildsystem
#   'waf' is the new buildsystem, 'make' is the old buildsystem.  Should pretty much always be 'waf' these days.
# @param ardupilot_all_vehicles
#   The list of all vehicles to pre-compile for the SITL environment.  This makes all of these vehicle's firmware available to SITL instances.
# @param ardupilot_vehicle
#   Which vehicle to build for the old 'make' buildsystem.
# @param sitl
#   If set to true, compile firmware for the SITL environment.
# @param armeabi_packages
#   If set to true, install the compiler chain software needed to cross-compile firmware for actual flight controller hardware.
# @param install_jsbsim
#   If set to true, compile and install the jsbsim software which is needed to run ArduPlane SITL.
# @param jsbsim_source
#   Git repo to use for jsbsim
#
class maverick_dev::ardupilot (
    String $ardupilot_source = "https://github.com/ArduPilot/ardupilot.git",
    Boolean $ardupilot_setupstream = true,
    String $ardupilot_upstream = "https://github.com/ArduPilot/ardupilot.git",
    String $ardupilot_branch = "master", # eg. master, Copter-3.3, ArduPlane-release
    Enum['waf', 'make'] $ardupilot_buildsystem = "waf", # waf (Copter >=3.4) or make (Copter <3.3)
    Hash $ardupilot_all_vehicles = {"copter" => "arducopter", "plane" => "arduplane", "rover" => "ardurover", "sub" => "ardusub", "heli" => "arducopter-heli", "antennatracker" => "antennatracker"},
    String $ardupilot_vehicle = "copter", # copter, plane or rover
    Optional[String] $sitl, # passed from init.pp
    Boolean $armeabi_packages = false, # needed to cross-compile firmware for actual FC boards
    Boolean $install_jsbsim = true,
    String $jsbsim_source = "http://github.com/JSBSim-Team/jsbsim.git",
) {
    
    # Install ardupilot from git
    oncevcsrepo { "git-ardupilot":
        gitsource   => $ardupilot_source,
        dest        => "/srv/maverick/code/ardupilot",
        revision	=> $ardupilot_branch,
        submodules  => true,
    }
    ensure_packages(["make", "gawk", "g++", "zip", "genromfs", "python-empy", "python-future"])
    if $armeabi_packages == true {
        if $::operatingsystem == "Ubuntu" and versioncmp($::operatingsystemmajrelease, "18") >= 0 {
            ensure_packages(["gcc-arm-none-eabi", "binutils-arm-none-eabi", "gdb-multiarch", "libnewlib-arm-none-eabi", "libstdc++-arm-none-eabi-newlib"], {'ensure'=>'present'})
        } else {
            ensure_packages(["gcc-arm-none-eabi", "binutils-arm-none-eabi", "gdb-arm-none-eabi", "libnewlib-arm-none-eabi", "libstdc++-arm-none-eabi-newlib"], {'ensure'=>'present'})
        }
    }
    
    # If a custom ardupilot repo is specified, configure the upstream automagically
    exec { "ardupilot_setupstream":
        command     => "/usr/bin/git remote add upstream ${ardupilot_upstream}",
        unless      => "/usr/bin/git remote -v | /bin/grep -i ${ardupilot_upstream}",
        cwd         => "/srv/maverick/code/ardupilot",
        require     => Oncevcsrepo["git-ardupilot"],
    }
    
    # Compile SITL
    if $sitl and $ardupilot_buildsystem == "waf" {
        # Compile all vehicle types by default for waf build
        $ardupilot_all_vehicles.each |String $vehicle, String $buildfile| {
            maverick_dev::fwbuildwaf { "sitl_waf_${vehicle}":
                require     => [ Oncevcsrepo["git-ardupilot"], Exec["ardupilot_setupstream"], Package["python-future"] ],
                board       => "sitl",
                vehicle     => $vehicle,
                buildfile   => $buildfile,
            }
        }
    } elsif $sitl and $ardupilot_buildsystem == "make" {
        if $ardupilot_vehicle == "copter" {
            $ardupilot_type = "ArduCopter"
        } elsif $ardupilot_vehicle == "plane" {
            $ardupilot_type = "ArduPlane"
        } elsif $ardupilot_vehicle == "rover" {
            $ardupilot_type = "ArduRover"
        } elsif $ardupilot_vehicle == "sub" {
            $ardupilot_type = "ArduSub"
        } elsif $ardupilot_vehicle == "antennatracker" {
            $ardupilot_type = "AntennaTracker"
        }
        fwbuildmake { "sitl_${ardupilot_vehicle}": 
            require     => [ Oncevcsrepo["git-ardupilot"], Exec["ardupilot_setupstream"] ],
            board       => "sitl",
            build       => $ardupilot_type,
        }
    }

    # Compile jsbsim and install service, for plane
    if $install_jsbsim and ! ("install_flag_jsbsim" in $installflags) {
        ensure_packages(["libexpat1-dev"])
        oncevcsrepo { "git-jsbsim":
            gitsource   => $jsbsim_source,
            dest        => "/srv/maverick/var/build/jsbsim",
        } ->
        file { '//srv/maverick/var/build/jsbsim/build':
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
        } ->
        exec { "jsbsim-cmake":
            command     => "/usr/bin/cmake -DCMAKE_INSTALL_PREFIX=/srv/maverick/software/jsbsim  ..",
            cwd         => "/srv/maverick/var/build/jsbsim/build",
            creates     => "/srv/maverick/var/build/jsbsim/build/Makefile",
            user        => "mav",
            require     => Package["libexpat1-dev"],
        } ->
        exec { "jsbsim-make":
            command     => "/usr/bin/make",
            timeout     => 0,
            cwd         => "/srv/maverick/var/build/jsbsim/build",
            creates     => "/srv/maverick/var/build/jsbsim/build/src/JSBSim",
            user        => "mav",
        } ->
        exec { "jsbsim-makeinstall":
            command     => "/usr/bin/make install",
            cwd         => "/srv/maverick/var/build/jsbsim/build",
            creates     => "/srv/maverick/software/jsbsim/bin/JSBSim",
            user        => "mav",
        } ->
        file { "/srv/maverick/software/jsbsim/bin":
            ensure      => directory,
            owner       => "mav",
            group       => "mav",
        } ->
        exec { "jsbsim-cpbin":
            command     => "/bin/cp /srv/maverick/var/build/jsbsim/build/src/JSBSim /srv/maverick/software/jsbsim/bin",
            creates     => "/srv/maverick/software/jsbsim/bin/JSBSim",
        } ->
        file { "/srv/maverick/var/build/.install_flag_jsbsim":
            ensure      => file,
            owner       => "mav",
            group       => "mav",
            mode        => "644",
        }
    }

}
