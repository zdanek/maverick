class maverick-baremetal::odroid::init {

    file { "/srv/maverick/software/odroid-utility":
        ensure 		=> directory,
        require		=> File["/srv/maverick/software"],
        mode		=> 755,
        owner		=> "mav",
        group		=> "mav",
    } ->
    vcsrepo { "/srv/maverick/software/odroid-utility":
        ensure		=> present,
        provider 	=> git,
        source		=> "https://github.com/mdrjr/odroid-utility.git",
        revision	=> "master",
        owner		=> "mav",
        group		=> "mav",
    }

    ensure_packages(["axel", "build-essential", "xz-utils", "whiptail", "unzip", "wget", "curl"])
        
}