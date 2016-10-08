class maverick_gcs::skysense (
) {
    
    # Pull skysense from github
    oncevcsrepo { "git-skysense_planner":
        gitsource   => "https://github.com/skyscode/planner.git",
        dest        => "/srv/maverick/software/skysense-planner",
    }
    
    
    
}