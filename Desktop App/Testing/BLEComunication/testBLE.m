%para apagar el modulo AT+MODE = 2
%para encenderlo primero mandar 10 bits FF para despertarloo
%despues ponerlo en modo AT+MODE=0 (full power)

l = blelist
b = ble("HC-08")
b.Services
%b.Characteristics
c = characteristic(b,'FFE0','FFE1')
c.DataAvailableFcn = @displayCharacteristicData;
% data = 'HC08-0';
% write(c,data,"withoutresponse")
subscribe(c);
%char(read(c))
% unsubscribe(c);%se deja de recibir info del modulo, pero se puede seguir escribiendo
% clear b
% data = read(c)
% char(data)
% c.Descriptors
% d = descriptor(c,'2901')
% data = read(d)
% char(data)