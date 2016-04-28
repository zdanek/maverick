class maverick-vision::camera::ocam {
    
    ensure_packages(["qt4-default", "libv4l-dev", "libudev-dev"])
    
    # Add ocam_viewer repo from git
    file { "/srv/maverick/software/ocam":
        ensure 		=> directory,
        require		=> File["/srv/maverick/software"],
        mode		=> 755,
        owner		=> "mav",
        group		=> "mav",
    } ->
    vcsrepo { "/srv/maverick/software/ocam":
        ensure		=> present,
        provider 	=> git,
        source		=> "https://github.com/withrobot/oCam.git",
        revision	=> "master",
        owner		=> "mav",
        group		=> "mav",
    }
    #exec { "ocam-compile":
    #    user        => "mav",
    #    timeout     => 0,
    #    command     => "/usr/bin/make -j${::processorcount} sitl",
    #    cwd         => "/srv/maverick/code/dronekit-sitl/sitl-fw/${build}",
    #    creates     => "/srv/maverick/code/dronekit-sitl/sitl-fw/${build}/${build}.elf",
    #}
    
}