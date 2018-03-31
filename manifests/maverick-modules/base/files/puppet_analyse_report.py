#!/usr/bin/python3 -tt

import yaml
reportData = yaml.load(open('/var/tmp/last_run_report.yaml'))

groupTimes = {}
for data in reportData['metrics']['time']['values']:
    groupTimes[data[0]] = data[2]
print(sorted(groupTimes.items(), key=lambda x: x[1]))

resourceTimes = {}
for key,data in reportData['resource_statuses'].items():
    resourceTimes[key] = data['evaluation_time']
for resource in sorted(resourceTimes.items(), key=lambda x: x[1]):
    print(resource[0], ": ", resource[1])

