devlist = bluetoothlist
%devlist = bluetoothlist("Timeout",5)
device = bluetooth("HC-06")
configureCallback(device,"byte",512,@getBluetoothData)
% device.BytesAvailableFcnMode
% device.BytesAvailableFcnCount
% device.BytesAvailableFcn
% device.NumBytesAvailable
read(device,device.NumBytesAvailable);

% write(device,[0,1,2,3])
% write(device,'HC06-0')

device.NumBytesWritten
% flush(device)
% flush(device,"input")
% flush(device,"output")

try
    device = bluetooth("HC-06")
catch ME
    ME
end