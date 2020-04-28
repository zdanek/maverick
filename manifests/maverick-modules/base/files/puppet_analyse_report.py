#!/usr/bin/env python3

import yaml
reportData = yaml.load(open('/var/lib/puppet/state/last_run_report.yaml'), Loader=yaml.BaseLoader)

groupTimes = {}
for data in reportData['metrics']['time']['values']:
    groupTimes[data[0]] = data[2]
print(sorted(groupTimes.items(), key=lambda x: x[1]))

resourceTimes = {}
for key,data in reportData['resource_statuses'].items():
    if key and data['evaluation_time']:
        resourceTimes[key] = data['evaluation_time']
for resource in sorted(resourceTimes.items(), key=lambda x: x[1]):
    print(resource[0], ": ", resource[1])

