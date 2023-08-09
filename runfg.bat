C:

cd C:\Program Files\FlightGear 2020.3

SET FG_ROOT=C:\Program Files\FlightGear 2020.3\data

START .\\bin\fgfs.exe --aircraft=HL20 --fdm=null --max-fps=30 --native-fdm=socket,out,30,localhost,5501,udp --native-fdm=socket,in,30,localhost,5502,udp --native-gui=socket,out,30,localhost,5504,udp --native-gui=socket,in,30,localhost,5505,udp
