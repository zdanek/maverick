class maverick_telemetry (
    $teensy = true,
) {
    
    if $teensy {
        class { "maverick_telemetry::teensy": }
    }
    
}