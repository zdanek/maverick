class maverick_web::nodejs (
) {
    
    # Workaround for ubilinux
    if $::lsbdistid == "ubilinux" and $::lsbdistcodename == "dolcetto" {
        $_release = "stretch"
    } else {
        $_release = $::lsbdistcodename
    }
    
    # Nodesource repo doesn't support Pi Zero/armv6l, install manually
    if $::raspberry_present == "yes" {
        exec { "armv6l-nodejs":
            command     => "/usr/bin/wget -O - https://raw.githubusercontent.com/sdesalas/node-pi-zero/master/install-node-v8.9.0.sh | bash",
            creates     => "/opt/nodejs/bin/node",
            timeout     => 0,
        }
    } else {
        class { 'nodejs':
            repo_url_suffix => '8.x',
            repo_release    => $_release,
            legacy_debian_symlinks => false,
            nodejs_package_ensure => latest,
        }
    }
    
}