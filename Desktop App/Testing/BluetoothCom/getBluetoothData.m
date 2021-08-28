function getBluetoothData(src,evt)
    tic
    src.UserData = read(src,src.BytesAvailableFcnCount);
    toc
    disp(src.UserData);
%     uint16(typecast(cast(src.UserData(2:3),'uint8'),'uint16'))
end 
