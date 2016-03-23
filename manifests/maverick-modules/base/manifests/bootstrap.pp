# Note: This base class is NOT included by base::init by default.  It's called by the bootstrap environment through hiera.
class base::bootstrap {
    
    # Bit of a hacky way to display a message after everything has finished, but puppet doesn't make it easy to do deterministic ordering!
    class sayfinish {
        speak { "sayfinish":
            message     => "\n\n========================\nIf this is the first bootstrap run please reboot now to activate system changes, otherwise subsequent runs may fail with unexpected results.\n========================\n",
            level       => "warning",
        }
    }
    class { "sayfinish":
        stage       => "finish",
    }
    
}