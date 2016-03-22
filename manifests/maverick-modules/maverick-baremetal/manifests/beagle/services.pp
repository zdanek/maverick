class maverick-baremetal::beagle::services {
    
    # These services are being stopped to save cpu and memory at boot.
    # This is a somewhat temporary situation, in the long term we should deal with these properly
    #  in their own manifests, particularly apache.
    service { "jekyll-autorun":
        ensure      => stopped,
        enable      => false,
    }
    service { "bonescript-autorun":
        ensure      => stopped,
        enable      => false,
    }
    service { "ofono":
        ensure      => stopped,
        enable      => false,
    }
    service { "bluetooth":
        ensure      => stopped,
        enable      => false,
    }
    service { "apache2":
        ensure      => stopped,
        enable      => false,
    }
    
}