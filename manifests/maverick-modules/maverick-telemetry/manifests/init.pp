class maverick-telemetry (
    $teensy = true,
) {
    
    if $teensy {
        class { "maverick-telemetry::teensy": }
    }
    
}