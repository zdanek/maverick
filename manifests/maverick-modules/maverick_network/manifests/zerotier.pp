class maverick_network::zerotier (
    $libzt = false,
    $active = false,
) {
 
    # Workaround for ubilinux
    if $::lsbdistid == "ubilinux" and $::lsbdistcodename == "dolcetto" {
        $_release = "stretch"
    } else {
        $_release = $::lsbdistcodename
    }
    
    # Install core zerotier pgp key, repo and package
    apt::key { 'zerotier':
      id      => '74A5E9C458E1A431F1DA57A71657198823E52A61',
      server  => 'pgp.mit.edu',
    } ->
    apt::source { 'zerotier':
      location      => "https://download.zerotier.com/debian/${_release}",
      release       => $_release,
      repos         => 'main',
      key           => {'id' => '74A5E9C458E1A431F1DA57A71657198823E52A61'},
      notify        => Exec["apt_update"],
    } ->
    package { "zerotier-one":
        ensure      => installed,
        require     => Exec["apt_update"],
    }
    
    # Install libzt - library is a WIP
    if $libzt == true {
        ensure_packages(["swig", "swig-examples"]) # for python_module target
        oncevcsrepo { "git-libzt":
            gitsource   => "https://github.com/zerotier/libzt.git",
            dest        => "/srv/maverick/software/libzt",
            submodules  => true,
        } ->
        exec { "libzt-make":
            command     => "/usr/bin/make -j ${::processorcount} static_lib tests",
            cwd         => "/srv/maverick/software/libzt",
            creates     => "/bin/blah",
            user        => "mav",
            require     => Package["swig"],
        }
    }
    
    if $active == true {
        service { "zerotier-one":
            ensure      => running,
            enable      => true,
        }
    } else {
        service { "zerotier-one":
            ensure      => stopped,
            enable      => false,
        }
    }
    
}