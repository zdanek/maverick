### Declare defines here so they can be accessed everywhere
define speak ($message = "", $level = "") {
    notify { $name:
        message     => $message,
        loglevel    => $level,
    }
}
### End of defines

node default {
    # This is a 'catch-all' node statement.
    # Instead of declaring nodes, or using an ENC, we use hiera to assign 
    #  classes and data to nodes in a hierarchical, segregated fashion.
}

hiera_include('classes')
