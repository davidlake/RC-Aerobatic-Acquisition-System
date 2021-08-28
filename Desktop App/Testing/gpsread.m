ublox_gps = gpsReceiver('COM3');
rawGPSData = read(ublox_gps);
[GPRMC, GPGGA, GPVTG, GPGSA] = HelperParseGPSData(rawGPSData);

flush(ublox_gps);
delete(ublox_gps);