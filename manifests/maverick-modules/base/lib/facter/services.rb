Facter.add(:installed_services) do
  setcode do
    services = []
    systemctl_output = Facter::Core::Execution::exec("systemctl --plain -a -t service list-units |awk {'print $1'} |grep service |sed 's/\.service//'")
    unless systemctl_output.to_s.empty?
        _services = systemctl_output.lines.map(&:chomp)
        _services.each do |_service|
            services.push(_service)
        end
    end
    services
  end
end