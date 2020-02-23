# @summary
#   Maverick_web::Maverick_web_legacy class
#   This class installs/manages the legacy web environment, which will disappear as soon as maverick-web is ready.
#
# @example Declaring the class
#   This class is included from maverick_web class and should not be included from elsewhere
#
class maverick_web::maverick_web_legacy (
) {

    # Remove the old legacy repo
    file { "/srv/maverick/software/maverick-fcs":
        ensure      => absent,
        force       => true,
        recurse     => true,
    }

    # Install maverick_web_legacy from git
    oncevcsrepo { "github-maverick_web_legacy`":
        gitsource   => "https://github.com/goodrobots/maverick-web-legacy",
        dest        => "/srv/maverick/software/maverick-web-legacy",
    }

}
