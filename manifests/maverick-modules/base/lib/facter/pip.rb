Facter.add(:python_modules) do
  setcode do
    envs = ["maverick", "global", "global3", "sitl", "fc"]
    pippaths = {"maverick" => "/srv/maverick/software/python/bin/pip3", "global" => "pip", "global3" => "pip3", "sitl" => "/srv/maverick/.virtualenvs/sitl/bin/pip", "fc" => "/srv/maverick/.virtualenvs/fc/bin/pip"}
    modules = {}
    envs.each do |env|
        modules[env] = {}
    end
    
    pippaths.each do |pipenv,pippath|
        # Suppress deprecation warnings for pip2
        if pippath == "pip"
            pip_output = Facter::Core::Execution::exec("PYTHONWARNINGS=ignore:DEPRECATION #{pippath} --disable-pip-version-check freeze")
        else
            pip_output = Facter::Core::Execution::exec("#{pippath} --disable-pip-version-check freeze")
        end
        # Scan and log pip entries for each environment
        unless pip_output.to_s.empty?
            pips = pip_output.lines.map(&:chomp)
            pips.each do |pip|
                pipfrags = pip.split("==")
                modules[pipenv][pipfrags[0]] = pipfrags[1]
            end
        end
    end
    
    modules
  end
end