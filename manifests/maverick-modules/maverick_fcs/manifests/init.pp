class maverick_fcs (
) {
    # FCS (Flying Control Station) is the concept of an onboard GCS.  It will have multiple interfaces, initially touchscreen and web.
    # The central process (maverickd) is a RESTful API that all the interfaces talk to

    ensure_packages(["cython", "libsdl2-dev", "libsdl2-image-dev", "libsdl2-mixer-dev", "libsdl2-ttf-dev", "pkg-config", "libgl1-mesa-dev", "libgles2-mesa-dev"])

    ### Install flask microframework that maverickd is constructed from
    # Install a python virtualenv for maverickd
    python::virtualenv { '/srv/maverick/software/maverick-fcs':
        ensure       => present,
        version      => 'system',
        systempkgs   => false,
        distribute   => true,
        venv_dir     => '/srv/maverick/.virtualenvs/maverick-fcs',
        owner        => 'mav',
        group        => 'mav',
        cwd          => '/srv/maverick/software/maverick-fcs',
        timeout      => 0,
    } ->
    file { "/srv/maverick/.virtualenvs/maverick_fcs/lib/python2.7/no-global-site-packages.txt":
        ensure  => absent
    }

    # Install maverick_fcs from git
    oncevcsrepo { "github-maverick_fcs":
        gitsource   => "https://github.com/fnoop/maverick-fcs",
        dest        => "/srv/maverick/software/maverick-fcs",
    }

    ### Install Kivy    
    oncevcsrepo { "git-kivy":
        gitsource   => "https://github.com/kivy/kivy",
        dest        => "/srv/maverick/software/kivy",
    } ->
    exec { "build-kivy":
        user        => "mav",
        timeout     => 0,
        command     => "/usr/bin/make >/srv/maverick/data/logs/build/kivy.build.out 2>&1",
        cwd         => "/srv/maverick/software/kivy",
        creates     => "/srv/maverick/software/kivy/kivy/properties.so",
        require     => [ Package["cython"], Oncevcsrepo["git-kivy"], Package["libgstreamer1.0-dev"] ] # ensure we have all the dependencies satisfied
    }
    file { "/etc/profile.d/kivy-path.sh":
        ensure      => present,
        content     => "export PYTHONPATH=/srv/maverick/software/kivy:\$PYTHONPATH",
    }

}