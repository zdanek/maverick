class maverick_intelligence::tensorflow (
    $source = "https://github.com/tensorflow/tensorflow.git",
    $revision = "r1.2",
    $active = false,
) {
    
    # Ensure package dependencies are installed
    ensure_packages(["openjdk-8-jdk", "zlib1g-dev", "swig"])
    install_python_module { "tensorflow-numpy":
        pkgname     => "numpy",
        ensure      => present,
    }
    file { "/srv/maverick/var/build/tensorflow":
        ensure      => directory,
        owner       => "mav",
        group       => "mav",
        mode        => "755",
    }
    
    # Set variables per platform, tensorbuild is quite specific per platform due to the numebr of kludges necessary
    if $joule_present == "yes" {
        $java_home = "/usr/lib/jvm/java-8-openjdk-amd64"
    } elsif $odroid_present == "yes" {
        $java_home = "/usr/lib/jvm/java-8-openjdk-armhf"
    } elsif $raspberry_present == "yes" {
        $java_home = "/usr/lib/jvm/java-8-openjdk-armhf"
    }

    if ! ("install_flag_tensorflow" in $installflags) {
        # Install bazel.  This is a bit hacky, due to the wierd way bazel decides to distribute itself..
        exec { "download-bazel":
            command     => "/usr/bin/wget https://github.com/bazelbuild/bazel/releases/download/0.5.0/bazel-0.5.0-dist.zip",
            cwd         => "/srv/maverick/var/build/tensorflow",
            creates     => "/srv/maverick/var/build/tensorflow/bazel-0.5.0-dist.zip",
            user        => "mav",
        } ->
        file { "/srv/maverick/var/build/tensorflow/bazel":
            owner       => "mav",
            group       => "mav",
            mode        => "755",
            ensure      => directory,
        } ->
        exec { "unzip-bazel":
            command     => "/usr/bin/unzip /srv/maverick/var/build/tensorflow/bazel-0.5.0-dist.zip -d /srv/maverick/var/build/tensorflow/bazel",
            cwd         => "/srv/maverick/var/build/tensorflow",
            user        => "mav",
            creates     => "/srv/maverick/var/build/tensorflow/bazel/README.md",
        }
        if $odroid_present == "yes" or $raspberry_present == "yes" {
            file { "/srv/maverick/var/build/tensorflow/bazel/scripts/bootstrap/compile.sh":
                ensure      => present,
                source      => "puppet:///modules/maverick_intelligence/bazel-compile-lowmem.sh",
                owner       => "mav",
                group       => "mav",
                mode        => "755",
                require     => Exec["unzip-bazel"],
                before      => Exec["compile-bazel"],
            }
        }
        exec { "compile-bazel":
            environment => "JAVA_HOME=$java_home",
            command     => "/srv/maverick/var/build/tensorflow/bazel/compile.sh >/srv/maverick/var/log/build/bazel.compile.log 2>&1",
            cwd         => "/srv/maverick/var/build/tensorflow/bazel",
            user        => "mav",
            timeout     => 0,
            creates     => "/srv/maverick/var/build/tensorflow/bazel/output/bazel",
            require     => [ Exec["unzip-bazel"], Package["openjdk-8-jdk"] ],
        } ->
        # Install tensorflow
        oncevcsrepo { "git-tensorflow":
            gitsource   => $source,
            dest        => "/srv/maverick/var/build/tensorflow/tensorflow",
            revision    => $revision,
            submodules  => true,
        } ->
        exec { "configure-tensorflow":
            environment => [
                "PYTHON_BIN_PATH=/usr/bin/python", 
                "PYTHON_LIB_PATH=/usr/local/lib/python2.7/dist-packages", 
                "TF_NEED_MKL=0",
                "TF_NEED_JEMALLOC=1",
                "TF_NEED_GCP=0",
                "TF_NEED_HDFS=0",
                "TF_NEED_VERBS=0",
                "TF_NEED_OPENCL=0",
                "TF_NEED_CUDA=0",
                "TF_ENABLE_XLA=0",
                "CC_OPT_FLAGS=\"-march=native\"",
                "PATH=/srv/maverick/var/build/tensorflow/bazel/output:/usr/bin:/bin",
            ],
            command     => "/bin/bash /srv/maverick/var/build/tensorflow/tensorflow/configure",
            cwd         => "/srv/maverick/var/build/tensorflow/tensorflow",
            user        => "mav",
            timeout     => 0,
        } ->
        exec { "compile-tensorflow":
            command     => "/srv/maverick/var/build/tensorflow/bazel/output/bazel build --config=opt //tensorflow/tools/pip_package:build_pip_package --local_resources 1024,1.0,1.0 >/srv/maverick/var/log/build/tensorflow.compile.log 2>&1",
            cwd         => "/srv/maverick/var/build/tensorflow/tensorflow",
            user        => "mav",
            timeout     => 0,
        } ->
        file { "/srv/maverick/var/build/.install_flag_tensorflow":
            owner       => "mav",
            group       => "mav",
            mode        => "644",
            # ensure      => present,
            ensure      => absent,
        }
    }
 
}