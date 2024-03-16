%https://www.mathworks.com/help/simulink/ug/using-the-sim-command.html
clear
disp("Initializing...")
COM = serialportlist;
if COM.size(2) ~= 1
    error("Invalid number of COM ports");
end

SP = serialport(COM, 115200);
pause(1.5)
SP.writeline("*ID?")
ID = readline(SP);
ID = convertStringsToChars(ID);
ID = ID(1:end - 1);
if ID ~= "generator"
    error("Invalid module");
end

clear SP

set_param("genSimu/generator PID/generator serial configuration", "Port", COM)
set_param("genSimu/generator PID/getVal serial send", "Port", COM)
set_param("genSimu/generator PID/getVal serial receive", "Port", COM)
set_param("genSimu/generator PID/setMot serial send", "Port", COM)

set_param("genSimu/P", "value", "5")
set_param("genSimu/I", "value", "25")
set_param("genSimu/D", "value", "0.001")
set_param("genSimu/setpoint", "value", "120")

disp("Starting Simulation...")