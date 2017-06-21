WINDOWS users: just run exe file


MAC OS users:  GUI uses a serial communication, that needs to create a lock file. 
To allow it, you should to do the following steps:
1. Start terminal (navigate to /Applications/Utilities and double click on Terminal)
2. Make folder "/var/lock" by command: sudo mkdir /var/lock
3. Change permissions by command: sudo chmod 777 /var/lock
4. Allow to run non-signed applications in  System Preferences > Security & Privacy > 
	General > Allow Applications downloaded from: Anywhere  (see http://d.pr/i/9fAm )


LINUX users: 
run.sh



ALL SYSTEMS: 

Special system drivers may be required. Check the recomendations of the seller of your board,
or try general drivers that may suit:
	
	FTDI http://www.ftdichip.com/Drivers/VCP.htm
	CP2102 http://www.silabs.com/products/mcu/pages/usbtouartbridgevcpdrivers.aspx


CONSOLE MODE

In this mode Java console is shown, that may be usefull for debugging
Start application by running script "run_console.bat" (or create similar script for your operating sysytem).


COMMAND-LINE MODE

There are several usefull commands that can be run in command-line mode (without opening the main interface window).
Execute "sbgc_cmd.bat" and it will print the usage prompt.