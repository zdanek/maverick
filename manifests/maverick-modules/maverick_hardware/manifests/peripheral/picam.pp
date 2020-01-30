# @summary
#   Maverick_hardware::Peripheral::Picam class
#   This class installs/manages the Raspberry Pi camera support.
#
# @example Declaring the class
#   This class is included from maverick_hardware class and should not be included from elsewhere
#
# @param docs
#   If true, install documentation for picam
#
class maverick_hardware::peripheral::picam (
    Boolean $docs = false,
) {
    
    ensure_packages(["python-picamera", "python3-picamera", "omxplayer"])
    if $docs == true {
        ensure_packages(["pyton-picamera-docs"])
    }
    
}
