s = serialport("COM3",115200);
info = readline(s);
data = str2double(strsplit(strtrim(erase(info,{char(0),char(10)}))));

T = data(1:2:end);
P = data(2:2:end);

flush(s)
s.NumBytesAvailable
