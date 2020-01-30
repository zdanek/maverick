# @summary
#   Maverick_intelligence:pytorch class
#   This class installs/manages the pytorch Machine Learning software (pytorch.org).
#
# @example Declaring the class
#   This class is included from maverick_intelligence class and should not be included from elsewhere
#
# @param source
#   Github repo to use when compiling from source.
# @param source_version
#   Github tag/branch to use when compiling from source.
#
class maverick_intelligence::pytorch (
    $source = "https://github.com/pytorch/pytorch.git",
    $source_version = "1.4",
) {

    install_python_module { "pytorch-torch":
        pkgname     => 'torch',
        ensure      => present,
        timeout     => 0,
    } ->
    install_python_module { "pytorch-torchvision":
        pkgname     => 'torchvision',
        ensure      => present,
        timeout     => 0,
    }

}
