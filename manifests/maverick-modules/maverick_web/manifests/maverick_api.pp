# @summary
#   Maverick_web::Maverick_api class
#   This class installs and manages the Maverick-api software.
#
# @see
#   https://github.com/goodrobots/maverick-api
#
# @example Declaring the class
#   This class is included from maverick_web class and should not be included from elsewhere
#
class maverick_web::maverick_api {

    # Install python components

    oncevcsrepo { "git-maverick-api":
        gitsource   => "https://github.com/goodrobots/maverick-api.git",
        dest        => "/srv/maverick/code/maverick-api",
        revision    => "master",
        depth       => undef,
    }

    # python::requirements isn't idempotent, so only run once on initial install
    if ! ("install_flag_apirequirements" in $installflags) {
        python::requirements { "/srv/maverick/code/maverick-api/requirements.txt":
            cwd             => "/srv/maverick/code/maverick-api",
            pip_provider    => "pip3",
            environment     => ["PATH=/srv/maverick/software/python/bin:\$PATH"],
            timeout         => 0,
            forceupdate     => true,
            fix_requirements_owner => false,
            manage_requirements => false,
            require         => Oncevcsrepo["git-maverick-api"],
            owner           => "mav",
            group           => "mav",
        } ->
        file { "/srv/maverick/var/build/.install_flag_apirequirements":
            ensure      => present,
        }
    }

    file { "/etc/systemd/system/maverick-api@.service":
        owner       => "root",
        group       => "root",
        mode        => "644",
        source      => "puppet:///modules/maverick_web/maverick-api@.service",
        notify      => Exec["maverick-systemctl-daemon-reload"],
        require     => Oncevcsrepo["git-maverick-api"],
    } -> 
    # Create a symlink to api launch script
    file { "/srv/maverick/software/maverick/bin/api.sh":
        ensure      => link,
        target      => "/srv/maverick/software/maverick/manifests/maverick-modules/maverick_web/files/api.sh",
    }
    
    file { "/srv/maverick/var/log/web/api":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    }
    
}
