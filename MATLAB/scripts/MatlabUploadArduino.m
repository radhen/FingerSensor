strOutput = '/usr/share/arduino/hardware/tools/avr/bin/avr-objcopy -O ihex -R .eeprom /tmp/build9002385807620855633.tmp/examplecode_vcnl4000_ino.cpp.elf /tmp/build9002385807620855633.tmp/examplecode_vcnl4000_ino12.cpp.hex' ;
comPort = '/dev/ttyACM1';

idxSpace = regexp(strOutput,'\s');
strAVRBIN = 'avr.bin';
idxAVRBIN = regexp(strOutput, strAVRBIN );
idxSpace = idxSpace( idxSpace>idxAVRBIN );
strAVR = strOutput(1:idxSpace(1)-1);
pathAVR = strAVR(1:idxAVRBIN+size(strAVRBIN,2));
pathAVR = sprintf('%s%s',pathAVR,'avrdude');

idxHEX = regexp(strOutput,'hex');
pathHEX = strOutput(idxSpace(end)+1:idxHEX(end)+2);

accountName = 'radhen';
pathHexLib = sprintf('/home/%s/Desktop/',accountName);



idxSlash = regexp(pathHEX,'[/]');
filenameHEX = pathHEX(idxSlash(end)+1:end);
pathHEX_new = sprintf('%s%s',pathHexLib,filenameHEX);
[status] = copyfile(pathHEX,pathHEX_new);
if ~status
[status] = copyfile(pathHEX,pathHEX_new,'f');
end

pathAVRconf = sprintf('%s%s%s',pathAVR(1:end-11),'etc','/avrdude.conf');
deviceType = 'atmega328p';
programmerType = 'arduino';
baudRate = '9600';

strUpload = sprintf('%s -C%s -p%s -c%s -P%s -D -Uflash:w:%s',...
pathAVR,pathAVRconf,deviceType,programmerType,comPort,pathHEX_new);

[status,result] = system( strUpload );