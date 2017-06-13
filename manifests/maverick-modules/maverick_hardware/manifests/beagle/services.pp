class maverick_hardware::beagle::services {
    
    # These services are being stopped to save cpu and memory at boot.
    # This is a somewhat temporary situation, in the long term we should deal with these properly
    #  in their own manifests, particularly apache.
    service_wrapper { "jekyll-autorun":
        ensure      => stopped,
        enable      => false,
    }
    service_wrapper { "bonescript-autorun":
        ensure      => stopped,
        enable      => false,
    }
    service_wrapper { "ofono":
        ensure      => stopped,
        enable      => false,
    }
    service_wrapper { "bluetooth":
        ensure      => stopped,
        enable      => false,
    }
    
}