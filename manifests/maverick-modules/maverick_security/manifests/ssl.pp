class maverick_security::ssl (
) {

    class { "openssl": }

    ## Is that it?
    # Well, yes for here.  SSL resources, certificate delcarations etc are created elsewhere in other modules.
}