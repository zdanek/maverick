class maverick_dev::dronekit (
    ) {
        
    file { "/srv/maverick/code/dronekit-apps":
        ensure      => "directory",
        owner       => "mav",
        group       => "mav",
        mode        => 755,
    }
    
    # Install rmackay9 red balloon finder
    oncevcsrepo { "git-red-balloon-finder":
        gitsource   => "https://github.com/rmackay9/ardupilot-balloon-finder.git",
        dest        => "/srv/maverick/code/dronekit-apps/red-balloon-finder",
    }
    # Install djnugent precision landing
    oncevcsrepo { "git-precision-landing":
        gitsource   => "https://github.com/djnugent/Precland.git",
        dest        => "/srv/maverick/code/dronekit-apps/precision-landing",
    }
        
}