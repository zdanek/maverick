class maverick_gcs::skysense (
) {
    # WIP: This doesn't work yet, at least on odroid ubuntu
    
    # Pull skysense from github
    oncevcsrepo { "git-skysense_planner":
        gitsource   => "https://github.com/skyscode/planner.git",
        dest        => "/srv/maverick/software/skysense-planner",
    }
    
    
    
}