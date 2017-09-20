class maverick_hardware::peripheral::picam (
    $docs = false,
) {
    
    ensure_packages(["python-picamera", "python3-picamera", "omxplayer"])
    if $docs == true {
        ensure_packages(["pyton-picamera-docs"])
    }
    
}