Facter.add(:python_modules) do
  setcode do
    envs = ["global", "sitl", "fc"]
    pippaths = {"global" => "pip", "sitl" => "/srv/maverick/.virtualenvs/sitl/bin/pip", "fc" => "/srv/maverick/.virtualenvs/fc/bin/pip"}
    modules = {}
    envs.each do |env|
        modules[env] = {}
    end
    
    pippaths.each do |pipenv,pippath|
        pip_output = Facter::Core::Execution::exec("#{pippath} freeze")
        pips = pip_output.lines.map(&:chomp)
        pips.each do |pip|
            pipfrags = pip.split("==")
            modules[pipenv][pipfrags[0]] = pipfrags[1]
        end
    end
    
    modules
  end
end