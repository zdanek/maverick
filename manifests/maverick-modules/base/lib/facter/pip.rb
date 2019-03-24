Facter.add(:python_modules) do
  setcode do
    envs = ["global", "global3", "sitl", "fc"]
    pippaths = {"global" => "pip", "global3" => "pip3", "sitl" => "/srv/maverick/.virtualenvs/sitl/bin/pip", "fc" => "/srv/maverick/.virtualenvs/fc/bin/pip"}
    modules = {}
    envs.each do |env|
        modules[env] = {}
    end
    
    pippaths.each do |pipenv,pippath|
        pip_output = Facter::Core::Execution::exec("#{pippath} --disable-pip-version-check freeze")
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