# @summary
#   Maverick_intelligence class
#   This class controls all other classes in maverick_intelligence module.
#   The maverick_intelligence class covers Machine Learning/Artificial Intelligence software.
#
# @example Declaring the class
#   This class is included from the environment manifests and is not usually included elsewhere.
#   It could be included selectively from eg. minimal environment.
#
# @param tensorflow
#   If true, include the maverick_intelligence::tensorflow class.  Note this doesn't activate tensorflow itself, just includes the class.
# @param pytorch
#   If true, include the maverick_intelligence::pytorch class.  Note this doesn't activate pytorch itself, just includes the class.
#
class maverick_intelligence (
    Boolean $tensorflow = true,
    Boolean $pytorch = true,
) {
    
    if $tensorflow == true {
        class { "maverick_intelligence::tensorflow": }
    }
    
    if $pytorch == true {
        class { "maverick_intelligence::pytorch": }
    }

}
