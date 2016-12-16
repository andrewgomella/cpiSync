cd "C:\Epics\iplApps\support\cpiSync"
procServ --allow -n "XRAY-SYNC" -p pid.txt -L log.txt -e "C:\WinPython-64bit-2.7.10.3\python-2.7.10.amd64\pythonw.exe" -i ^D^C 2007 indico100Sync.py
