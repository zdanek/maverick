class maverick_web::maverick_docs (
    $server_hostname = $maverick_web::server_fqdn,
    $ardupilot_docs = true,
    $px4_docs = true,
) {

    file { "/srv/maverick/software/maverick-docs":
        ensure      => absent,
        force       => true,
    }

    nginx::resource::location { "web-maverick-docs":
        ensure          => present,
        ssl             => true,
        location        => "/web/docs/maverick",
        location_alias  => "/srv/maverick/software/maverick/docs",
        index_files     => ["index.html"],
        server          => $server_hostname,
        require         => [ Class["maverick_gcs::fcs"], Class["nginx"], Service_wrapper["system-nginx"] ],
    }

    if $ardupilot_docs == true {
        file { "/srv/maverick/var/lib/web/ardupilot_wiki":
            owner       => "mav",
            group       => "mav",
            ensure      => directory,
        } ->
        install_python_module { "pip-sphinx":
            pkgname     => "sphinx",
            ensure      => present,
        } ->
        install_python_module { "pip-sphinx-rtd":
            pkgname     => "sphinx-rtd-theme",
            url         => "git+https://github.com/fnoop/sphinx_rtd_theme.git",
            ensure      => present,
        } ->
        install_python_module { "pip-sphinx-youtube":
            pkgname     => "sphinxcontrib-youtube",
            url         => "git+https://github.com/sphinx-contrib/youtube.git",
            ensure      => present,
        } ->
        install_python_module { "pip-sphinx-vimeo":
            pkgname     => "sphinxcontrib.vimeo",
            url         => "git+https://github.com/fnoop/sphinxcontrib.vimeo.git",
            ensure      => present,
        } ->
        oncevcsrepo { "git-ardupilot_docs":
            gitsource   => "https://github.com/fnoop/ardupilot_wiki.git",
            dest        => "/srv/maverick/software/ardupilot_wiki",
            revision    => "destdir",
        } ->
        file { "/srv/maverick/software/ardupilot_wiki/common_conf.py":
            source      => "puppet:///modules/maverick_web/arduwiki_common_conf.py",
        } ->
        file { "/srv/maverick/software/ardupilot_wiki/ardupilot/source/index.rst":
            source      => "puppet:///modules/maverick_web/arduwiki_index.rst",
        } ->
        exec { "ardupilot_wiki_compile":
            command     => "/srv/maverick/software/python/bin/python3 /srv/maverick/software/ardupilot_wiki/update.py --destdir=/srv/maverick/var/lib/web/ardupilot_wiki",
            user        => 'mav',
            timeout     => 0,
            cwd         => "/srv/maverick/software/ardupilot_wiki",
            creates     => '/srv/maverick/var/lib/web/ardupilot_wiki/copter/index.html',
        } ->
        nginx::resource::location { "web-ardupilot-wiki":
            ensure          => present,
            ssl             => true,
            location        => "/web/docs/ardupilot",
            location_alias  => "/srv/maverick/var/lib/web/ardupilot_wiki",
            index_files     => ["index.html"],
            server          => $server_hostname,
            require         => [ Class["maverick_gcs::fcs"], Class["nginx"], Service_wrapper["system-nginx"] ],
        }
    }

    if $px4_docs == true {
        oncevcsrepo { "git-px4_docs":
            gitsource   => "https://github.com/PX4/px4_user_guide.git",
            dest        => "/srv/maverick/software/px4_user_guide",
        } ->
        package { 'gitbook-cli':
            ensure   => 'latest',
            provider => 'npm',
        } ->
        exec { 'gitbook-px4-install':
            path    => ["/bin", "/usr/bin", "/opt/nodejs/bin"],
            command => "gitbook install",
            user    => "mav",
            timeout => 0,
            cwd     => "/srv/maverick/software/px4_user_guide",
            creates => "/srv/maverick/software/px4_user_guide/node_modules/gitbook-plugin-mathjax",
        } ->
        exec { 'gitbook-px4-build':
            path    => ["/bin", "/usr/bin", "/opt/nodejs/bin"],
            command => "gitbook build --format website",
            user    => "mav",
            timeout => 0,
            cwd     => "/srv/maverick/software/px4_user_guide",
            creates => "/srv/maverick/software/px4_user_guide/_book/en/index.html",
        } ->
        nginx::resource::location { "web-px4-guide":
            ensure          => present,
            ssl             => true,
            location        => "/web/docs/px4",
            location_alias  => "/srv/maverick/software/px4_user_guide/_book",
            index_files     => ["index.html"],
            server          => $server_hostname,
            require         => [ Class["maverick_gcs::fcs"], Class["nginx"], Service_wrapper["system-nginx"] ],
        }
    }
}
