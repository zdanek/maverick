class maverick_web::nodejs (
) {
    
    class { 'nodejs':
        repo_url_suffix => '8.x',
        legacy_debian_symlinks => false,
        nodejs_package_ensure => latest,
    }

}