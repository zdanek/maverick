# Base::Stages class
#
# This class declares puppet stages.
#
# @example Declaring the class
#   This class is included from base class and should not be included from elsewhere
#
class base::stages {

    # Declare a bootstrap run stage.  This is primarily for classes that are really crucial to run first, like package repos.
    stage { "bootstrap": 
        before => Stage['main'],
    }

    
    # Declare an end stage.  This is for classes that are important to run last.
    stage { "finish": 
        require => Stage['main'],
    }
    
}