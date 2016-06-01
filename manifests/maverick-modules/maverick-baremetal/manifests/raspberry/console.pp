class maverick-baremetal::raspberry::console (
    $width = 800,
    $height = 600
) {
    
    confval{ "console_fbwidth": file => "/boot/config.txt", field => "framebuffer_width", value => $width }
    confval{ "console_fbheight": file => "/boot/config.txt", field => "framebuffer_height", value => $height }
    confval{ "console_hdmiforce": file => "/boot/config.txt", field => "hdmi_force_hotplug", value => 1 }
    confval{ "console_hdmigroup": file => "/boot/config.txt", field => "hdmi_group", value => 2 }
    confval{ "console_hdmimode": file => "/boot/config.txt", field => "hdmi_mode", value => 87 }
    confval{ "console_hdmicustom": file => "/boot/config.txt", field => "hdmi_cvt", value => "${width} ${height} 60 1 0 0 0" }
    
}