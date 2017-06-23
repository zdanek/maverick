#!/usr/bin/env python
#
# Script to import ArduPilot logs into influxdb for analysis/display

import sys, datetime
from pymavlink import DFReader
from pymavlink.DFReader import DFReader, DFReader_binary, DFReader_text
from influxdb import InfluxDBClient, SeriesHelper

database = "maverick"
client = InfluxDBClient('localhost', 8086, 'mav', 'wingman', database)
client.create_database(database)

filename = sys.argv[1]
if filename.endswith('.log'):
    log = DFReader_text(filename, False)
else:
    log = DFReader_binary(filename, False)

parameters = {}
formats = {}
json_points = []
counter = 0
last_timestamp = None
while True:
    entry = log.recv_msg()
    if entry is None:
        break
    msgtype = entry.fmt.name
    if msgtype == "PARM":
        parameters[entry.Name] = entry.Value
    elif msgtype == "FMT":
        formats[entry.Name] = {'format': list(entry.Format), 'columns': entry.Columns.split(",")}
        #newClass = seriesFactory(msgtype.lower())
        #print newClass, type(newClass())
    elif msgtype == "MSG":
        print("Message:",entry.TimeUS,":",entry.Message)
    else:
        if msgtype in formats:
            #print(msgtype,":",entry._timestamp,":", [{column:getattr(entry, column)} for column in formats[msgtype]['columns']])
            _fields = {column:getattr(entry, column) for column in formats[msgtype]['columns']}
            last_timestamp = datetime.datetime.fromtimestamp(entry._timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')
            json_body = {
                "database": database,
                "time_precision": "ms",
                "measurement": "flight_"+msgtype.lower(),
                "tags": {
                    "filename": filename
                },
                # "time": str(int(entry._timestamp*1000000)), # microseconds since epoch
                "time": last_timestamp,
                "fields": _fields
            }
            json_points.append(json_body)
            counter += 1
        else:
            print("Msg not recognised:",entry)

    if counter % 1000 == 0:
        client.write_points(json_points)
        json_points = [] # Clear out json_points after bulk write

client.write_points(json_points)
print str(counter) + " points inserted.  Last timestamp is " + str(last_timestamp)