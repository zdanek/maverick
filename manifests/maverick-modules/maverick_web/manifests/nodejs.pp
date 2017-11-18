class maverick_web::nodejs (
) {
    
    # Workaround for ubilinux
    if $::lsbdistid == "ubilinux" and $::lsbdistcodename == "dolcetto" {
        $_release = "stretch"
    } else {
        $_release = $::lsbdistcodename
    }
    
    class { 'nodejs':
        repo_url_suffix => '8.x',
        repo_release    => $_release,
        legacy_debian_symlinks => false,
        nodejs_package_ensure => latest,
    }

}