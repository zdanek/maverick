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
            command     => "/usr/bin/wget -O - https://raw.githubusercontent.com/sdesalas/node-pi-zero/master/install-node-v10.15.0.sh | bash",
            creates     => "/opt/nodejs/bin/node",
            timeout     => 0,
        } ->
        file { "/usr/bin/node":
            ensure  => link,
            target  => "/opt/nodejs/bin/node",
        }
    } else {
        class { 'nodejs':
            repo_url_suffix           => '10.x',
            repo_release              => $_release,
            nodejs_package_ensure     => latest,
        }
    }

}
