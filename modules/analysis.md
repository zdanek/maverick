# Analysis Module

The Analysis module introduces a number of tools to assist in visualising and analysing data.  

## Automated Data Analysis
Maverick includes a preinstalled, preconfigured analysis suite that can be used to translate, store, visualise and analyse data from Ardupilot dataflash and telemetry logs (this will be extended to px4 logs in the future).  
It uses best-of-breed components:  
 - *InfluxDB*: Nanosecond-resolution time series database for storing all system and flight data
 - *Collectd*: Efficient, extensive system metrics collection daemon
 - *Grafana*: Rich automated graphing dashboard system

*Grafana* is automatically setup as a web dashboard service, and 'Index' dashboards are automatically created and updated whenever data is added to the system.  Access the dashboards by going to port 6790 (by default) in a web browser, eg:  
http://maverick-raspberry.local:6790/

Firewall rules are automatically created for this service, and the default web port can be altered if required by setting a localconf parameter:  
`"maverick_analysis::grafana::webport": "1234"`

### Flight Data
Flight data in the form of dataflash or telemetry logs can be easily imported by simply dropping them into an 'inbox' folder: `~/data/mavlink/inbox`.  Whenever a file is copied or moved into this location, the Maverick 'mavlogd' service will automatically import it and then archive it into `~/data/mavlink/archive/inbox`.  The mavlogd service also watches other directories for data, by default:  
 - ~/data/mavlink/inbox
 - ~/data/mavlink/fc
 - ~/data/mavlink/sitl

The last two entries are the locations that 'mavlink-router' mavlink proxy logs flight data to automatically.  The mavlogd service waits until the data files are closed (which usually happens when the flight controller is disarmed or shut down), and then automatically processes and archives them.  So a few seconds after each flight is completed, the data is automatically processed and ready to view through the dashboards.

There are several dashboards available for flight data analysis:  
 - *Flight Data Analysis*: Extensive preconfigured graphs for most sections of available flight data, grouped by section
 - *Flight EKF2 Analsyis*: Pre-configured graph sets for EKF2 data, setup for comparing multiple IMUs side by side
 - *Flight EKF3 Analysis*: Pre-configured graph sets for EKF3 data, setup for comparing multiple IMUs side by side
 - *Flight EKF2-EKF3 Analysis*: Pre-configured graph sets, setup for comparing one IMU with EKF2 and one IMU with EKF3
 - *MAVExplorer Mavgraphs*: Example dashboard showing a 'port' of MAVExplorer mavgraphs.  Additional MAVEXplorer graph sets may be added or merged in the future.

All data is imported into the same database, so is all available in a single 'time universe'.  Because flights tend to take a relatively short period of time, this can make finding and navigating the data quite difficult on a normal timeline, so Index dashboards are automatically created and updated to make finding and visualising the data as simple as a click:  
 - *Flight Logs Index*: A table of all flight logs imported into the system, with time ranges and links to each available/relevant dashboard.  The type of data contained in the log is automatically detected and only links to the relevant dashboards (eg. EKF2/EKF3) are presented for that set of data.  Each link takes you to the dashboard and presets the start and finish timeframe for you.
 - *Error Events Index*: This is a very simple example of the type of powerful analysis that can be done once flight data has been imported into the time series database.  Each entry in the index represents an 'Error Event' in the flight data, across all flights, and presents links to immediately zoom in and visualise the flight data around that event.

### System Metrics
System metrics are automatically configured, collected, translated and stored into the InfluxDB database using *collectd*.  It is a very lightweight system and should not interfere with the running of Maverick services.  However, on very resource-limited platforms (eg. Raspberry Pi Zero), collectd can be disabled by setting a localconf parameter:  
`"maverick_analysis::collect::active": false`

System metrics can be visualised by going to the Grafana dashboards as described in the previous section, and choosing 'System Dashboard' from the dashboard dropdown.

### Dashboards
*Grafana* is used as the main system for visualising flight and system data.  It provides a rich interactive environment for viewing data, and interfaces directly to InfluxDB, the nanosecond-resolution time series database that Maverick uses to store all the system and flight data.  It is however really meant for analysing realtime system metrics, and needs some manipulating to view specific narrow time ranges of high density sensor data like that of flight data.  

#### Dashboard Switcher
To make it much easier to find and view flight data, a *Flight Logs Index* dashboard is dynamically updated whenever logs are processed.  To change dashboards, use the dashboard switcher in the top left of the home page:

![Dashboard Switcher](/media/analysis/dashboard-switcher.jpg)  

#### Logs Index
The Logs Index lists each flight data file processed along with the start and end time of the data points, and then provides relevant links to the various graphing dashboards.  It detects what EKF data is available and only presents links to the available data.  
The main flight data is available in the *Flight Data* dashboard link.  There is also a *Mavgraphs* dashboard link which is an example port of the MAVExplorer mavgraphs set of graphs.  
Then there are the EKF links.  
 - *EKF2* is a dashboard that displays EKF2 data comparing multiple IMUs running EKF2.  
 - The same layout is available for *EKF3* if multiple IMUs are running EKF3.
 - If there is one IMU running EKF2 and one IMU running EKF3, the *EKF2-EKF3* dashboard is available that is laid out to compare EKF2 vs EKF3.

![Logs Index](/media/analysis/logs-index.jpg)  

#### Error Events Index
The Error Events Index is a list of all Error events that have occurred across all events.  It provides links that take you directly to the event, centered in the time range, to quickly analyse what occurred around the time of the event.

![Error Events Index](/media/analysis/error-index.jpg)  

#### Dashboards
<a href="/media/analysis/flightdata-dashboard.jpg" target=_dashboard_><img src="/media/analysis/flightdata-dashboard.jpg"></a>

The dashboards are richly interactive.  In the dashboard screenshot above, there are several labels to highlight the interactions:  
 - *Time Range*: This section allows you to select or alter the time range in view.  It is recommended to use the *Logs Index Dashboard* for selecting time ranges as these are preset to the start and finish times of each flight data log, but it can also be useful for looking at flights as a whole.  There are handy presets, so for example if you have had a full days flying at the airfield, you can select the 'Today' time range and see all data for all flights today.  This is also useful for selecting the system data to view, for example 'Last 30 minutes'.
 - *Annotations*: These are selectors that toggle interesting flight data events.  In the screenshot you can see vertical blue and green lines - these are *Mode* and *Message* events respectively, and details can be viewed by hovering over the small triangles at the bottom of each line (an EKF3 in-flight yaw alignment event is shown, as marked by *Annotation* in the screenshot).  By default, only *Mode* and *Error* are selected - *Message* and *Event* can show quite a lot of events and can slow down the interface, so are not selected by default.
 - *Panel Header*: Clicking on the Panel Header (Title) brings up some useful actions.  In particular, the *View* action is really helpful when looking at very dense data - it 'full sizes' a particular graph to the full size of the screen, which makes it much easier to view a particular set of data.  When looking at a single graph full screen, a separate 'Back to dashboard' link appears to go back to the full dashboard.
 - *Data Tooltip*: When you hover over a graph, it displays a vertical red line underneath the mouse pointer location, and for that set of data points shows a tooltip of the value of each data point at that specific time.  Depending on the dashboard, it also shows either corresponding vertical redlines in all other visible graphs on the page that match exactly to the same time as the graph that is being hovered over, or some dashboards also show corresponding tooltips which can be very useful for comparing data (eg. between IMUs).
 - *Row Header*: Each dashboard contains many graphs which are grouped into rows, or sets of graphs.  Each row is collapsed by default as this is the fastest method of displaying the dashboard, and allows the fastest selection of required data to view.  In order to expand or collapse a row header, simply click on it.  Note that the more graphs open in a dashboard, the slower the display and interactions will be, so only keep open the rows desired to view at any one time.

<a href="/media/analysis/dashboard-zoomin.jpg" target=_dashboard_><img src="/media/analysis/dashboard-zoomin.jpg"></a>  
In addition, any graph can be click-dragged to select and zoom in to a specific range of data points.  This is very useful for visually zooming in on interesting data.  As one graph is zoomed in, so all other graphs in the dashboard zoom in to the same time scale so they always all show the same range of data points at any one time.
