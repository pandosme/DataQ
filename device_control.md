# Development Device
THis file describes the devive being used to develop the ACAP.  Use this for testing, validation and troubleshooting.

Address: 10.13.8.236
User: nodered
Password: rednode
Platform: aarch64

Note the Axis device uses digest authentication

# Building the ACAP
use the script build.sh to build the ACAP

# Installing
Use the script install.sh to install the eap file to upload the ACAP.  If the ACAP is running it will automatically restart.  If not running it will not start.
python install.py MyACAP.eap 10.13.8.20 user password

# Starting stoppig the ACAP
http://{address}/axis-cgi/applications/control.cgi?action=start&package={PACAKGE NAME}
http://{address}/axis-cgi/applications/control.cgi?action=stop&package={PACAKGE NAME}

# Log files
Access the ACAP specific logs with
http://{address}/axis-cgi/admin/systemlog.cgi?appname={PACKAGE NAME}

Access the devce syslog
http://{address}/axis-cgi/admin/systemlog.cgi
