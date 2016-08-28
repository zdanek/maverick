class maverick_telemetry::teensy (
    $teensy_loader_cli_revision = "master",
    $teensy_hwversion = "mkl26z64", # teensy3.[1,2]: mk20dx256, teensy3.0: mk20dx128, teensylc: mkl26z64
) {
    
    ensure_packages(["libusb-dev"])
    # Install teensy_loader_cli
    oncevcsrepo { "git-teensy_loader_cli":
        gitsource   => "https://github.com/PaulStoffregen/teensy_loader_cli.git",
        dest        => "/srv/maverick/software/teensy_loader_cli",
        revision	=> "${teensy_loader_cli_revision}",
        submodules  => false
    } ->
    # Compile teensy_loader_cli
    exec { "compile-teensy_loader_cli":
        command     => "/usr/bin/make -j${::processorcount} >/srv/maverick/var/log/build/teensy-load-cli.build.log 2>&1",
        creates     => "/srv/maverick/software/teensy_loader_cli/teensy_loader_cli",
        user        => "mav",
        cwd         => "/srv/maverick/software/teensy_loader_cli",
        timeout     => 0,
        require     => Package["libusb-dev"],
    }
    
}