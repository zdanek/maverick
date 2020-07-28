# Contributing - Documentation
One of the most helpful contributions that anyone can make to a project is documentation.  Anyone can contribute to documentation even if design or coding isn't your thing.  And most coders hate documentation, so will be very grateful :)

## Maverick Documentation
Maverick documentation is deliberately very simple and easy to use and update.  In contrast to other doc systems that require toolchain installation and compiling, the documentation of Maverick requires no installation and compilation to update (although there is an optional docserver).  The docs use [Docsify](https://docsify.js.org/#/) which takes standard Markdown (.md) files and parses and displays them on the fly, through clientside Javascript.

There are several ways you can update Maverick docs:  

### Local Repo
Loading docsify is part of the main docs index.html file, so you could simply clone the main Maverick git repo anywhere on your laptop or computer and simply load the docs folder in a browser, update the markdown and reload the browser to immediately see updates.  The workflow to update docs locally would be something like:
 - Fork the Maverick repo to your own github account  
 - Create a new branch for your docs updates.  Click on the 'Branch: stable' dropdown, choose 'master' first, then type a new branch name in the input box, eg. 'mydocs' and click on 'Create branch: mydocs from master'
 - Clone the Maverick repo: `git clone https://github.com/myaccount/maverick.git`  
 - Make sure the new 'mydocs' branch is active: `git checkout mydocs`  
 - Update the docs, make your contribution!
 - Commmit and push the updates to github: `git commit -am 'My docs updates'; git push`
 - Back in the github website, go to your Maverick fork and you should see a prompt to create a PR from your 'mydocs' branch.  Push the button, raise a PR!

### Custom Maverick module
Another way to update and contribute to the docs (or any other part of Maverick), is to create a Maverick clone within your running Maverick companion computer.  [As described here](/modules/custom), you can easily extend Maverick by creating a custom module.  In this custom module, it's really simple to clone a Maverick fork to a safe area, and then add a webserver endpoint for the docs.  

As with the Local Repo option above, start by forking the Maverick repo and creating a branch for your contributions:  
 - Fork the Maverick repo to your own github account  
 - Create a new branch for your docs updates.  Click on the 'Branch: stable' dropdown and type a new branch name in the input box, eg. 'mydocs'

Next, create a custom module (refer to the [Custom Module](/modules/custom) docs for background to the process):  
 - Create a new custom module: `mkdir -p ~/code/maverick/custom-modules/mavcontrib/manifests`  
 - Create a default manifest for the new module (put this content into *~/code/maverick/custom-modules/mavcontrib/manifests/init.pp*):  
    ```puppet
    class mavcontrib {
        oncevcsrepo { "mavcontrib-maverick-gitclone":
            gitsource   => "https://github.com/fnoop/maverick",
            revision    => "mydocs",
            dest        => "/srv/maverick/code/maverick.myfork",
        } ->
        nginx::resource::location { "mavcontrib-maverick-docs":
            ensure          => present,
            ssl             => true,
            location        => "/custom/mavdocs",
            location_alias  => "/srv/maverick/code/maverick.myfork/docs",
            index_files     => ["index.html"],
            server          => $::fqdn,
            require         => [ Class["nginx"], Service["nginx"] ],
        }
    }
    ```
 - Add the custom module, by adding the class to *~/config/maverick/localconf.json*:  
    ```puppet
    "classes": [ 
        "mavcontrib"
    ]
    ```
 - Run a maverick configure to activate the new custom module:  
    ```
    maverick configure
    ```

This clones your Maverick fork, checks out your 'mydocs' branch, and then adds the docs directory to the webserver under a custom URL, eg.:  
  http://maverick-raspberry.local/custom/mavdocs

You can edit the docs, make your changes and you should see them update immediately through your local website.

?> If your content does not update immediately, try to Shift-Reload the page in your browser - the old content may be cached in your browser

