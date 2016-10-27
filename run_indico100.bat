cd "C:\Epics\iplApps_3_0\support\cpisync"
procServ --allow -n "XRAY-SYNC" -p pid.txt -L log.txt -e "C:\WinPython-64bit-2.7.10.3\python-2.7.10.amd64\pythonw.exe" -i ^D^C 2007 cpisync_v12.py
