# @summary
#   Maverick_web::Maverick_docs class
#   This class installs and manages the Maverick documentation.
#
# @example Declaring the class
#   This class is included from maverick_web class and should not be included from elsewhere
#
# @param server_hostname
#   Specifies which webserver vhost to use for the documentation.  Should not be set in normal circumstances.
# @param arudpilot_docs
#   If true, compile and install the Ardupilot reference documentation.
# @param ardupilot_builddir
#   Path to clone github repo and build docs
# @param ardupilot_installdir
#   Path to install compiled docs to
# @param px4_docs
#   If true, compile and install the PX4 reference documentation.
# @param devframe_docs
#   If true, compile and install the Goodrobots Devframe documentation.
#
class maverick_web::maverick_docs (
    String $server_hostname = $maverick_web::server_fqdn,
    Boolean $ardupilot_docs = false,
    String $ardupilot_builddir = "/srv/maverick/var/build/ardupilot_wiki",
    String $ardupilot_installdir = "/srv/maverick/software/ardupilot_wiki",
    Boolean $px4_docs = false,
    Boolean $devframe_docs = false,
    String $devframe_docs_gitpath = "https://github.com/goodrobots/devframe.git",
) {

    file { "/srv/maverick/software/maverick-docs":
        ensure      => absent,
        force       => true,
    }

    # Maverick Docs
    nginx::resource::location { "web-maverick-docs":
        ensure          => present,
        ssl             => true,
        location        => "/web/docs/maverick",
        location_alias  => "/srv/maverick/software/maverick/docs",
        index_files     => ["index.html"],
        server          => $server_hostname,
        require         => [ Class["nginx"], Service["nginx"] ],
    }

    # Create Maverick parameter docs from puppet strings
    file { "/srv/maverick/var/lib/web/maverick-parameters":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
    } ->
    exec { "maverick-puppetstrings":
        cwd         => "/srv/maverick/var/lib/web/maverick-parameters",
        command     => "/usr/local/bin/puppet strings generate /srv/maverick/software/maverick/manifests/maverick-modules/**/manifests/*.pp",
        creates     => "/srv/maverick/var/lib/web/maverick-parameters/doc",
        user        => "mav",
        environment => ['HOME=/srv/maverick'],
    } ->
    nginx::resource::location { "web-maverick-parameters-docs":
        ensure          => present,
        ssl             => true,
        location        => "/web/docs/maverick-parameters",
        location_alias  => "/srv/maverick/var/lib/web/maverick-parameters/doc",
        index_files     => ["index.html"],
        server          => $server_hostname,
        require         => [ Class["nginx"], Service["nginx"] ],
    }

    if $devframe_docs == true {
        package { "bundler":
            ensure      => "2.1.2",
            provider    => "gem",
        } ->
        package { "bundle":
            ensure      => present,
            provider    => "gem",
        } ->
        oncevcsrepo { "git-devframe_docs":
            gitsource   => $devframe_docs_gitpath,
            dest        => "/srv/maverick/software/devframe",
        } ->
        exec { "devframe_docs-bundle-install":
            command     => "/usr/local/bin/bundle install --deployment",
            cwd         => "/srv/maverick/software/devframe/docs",
            unless      => "/bin/ls -l /srv/maverick/software/devframe/docs/vendor/bundle/ruby/**/gems/html-pipeline-**",
            user        => "mav",
            timeout     => 0,
        } ->
        exec { "devframe_docs-docs":
            environment => ["HOME=/srv/maverick"],
            command     => "/usr/local/bin/bundle exec jekyll build --verbose --trace",
            cwd         => "/srv/maverick/software/devframe/docs",
            creates     => "/srv/maverick/software/devframe/docs/_site/builds",
            user        => "mav",
        } ->
        nginx::resource::location { "web-devframe-docs":
            ensure          => present,
            ssl             => true,
            location        => "/web/docs/devframe",
            location_alias  => "/srv/maverick/software/devframe/docs/_site",
            index_files     => ["index.html"],
            server          => $server_hostname,
            require         => [ Class["nginx"], Service["nginx"] ],
        }
    }

    if $ardupilot_docs == true {
        if ! ("install_flag_ardupilot_wiki" in $installflags) {
            file { $ardupilot_installdir:
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
                dest        => $ardupilot_builddir,
                revision    => "destdir",
            } ->
            file { "${ardupilot_builddir}/common_conf.py":
                source      => "puppet:///modules/maverick_web/arduwiki_common_conf.py",
            } ->
            file { "${ardupilot_builddir}/ardupilot/source/index.rst":
                source      => "puppet:///modules/maverick_web/arduwiki_index.rst",
            } ->
            exec { "ardupilot_wiki_compile":
                command     => "/srv/maverick/software/python/bin/python3 ${ardupilot_builddir}/update.py --destdir=${ardupilot_installdir}",
                user        => 'mav',
                timeout     => 0,
                cwd         => $ardupilot_builddir,
                creates     => "${ardupilot_installdir}/copter/index.html",
            } ->
            file { "/srv/maverick/var/build/.install_flag_ardupilot_wiki":
                ensure      => present,
            }
        }
        nginx::resource::location { "web-ardupilot-wiki":
            ensure          => present,
            ssl             => true,
            location        => "/web/docs/ardupilot",
            location_alias  => $ardupilot_installdir,
            index_files     => ["index.html"],
            server          => $server_hostname,
            require         => [ Class["nginx"], Service["nginx"] ],
        }
    }

    if $px4_docs == true {
        oncevcsrepo { "git-px4_docs":
            gitsource   => "https://github.com/PX4/px4_user_guide.git",
            dest        => "/srv/maverick/software/px4_user_guide",
        } ->
        package { 'gitbook-cli':
            ensure   => present,
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
            require         => [ Class["nginx"], Service["nginx"] ],
        }
    }
}
