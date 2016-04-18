figure(6);
hold on

% %Forces and distances at 40mA (PDMS)
 Dpdmsx = [1 2 3 4 5];
 Fpdmsx = [0 1 2 3 4 5];
 load('exp#4.mat');
 Dpdmsy = distance81(2,[2,4,6,7,8]);
 Fpdmsy = pressure81(2,[1,6,7,8]);
 plot([-flip(Fpdmsx) Dpdmsx],[flip(Fpdmsy) Dpdmsy])
 
 hold on
 
 Dpdmsy = distance101(2,[2,4,6,7,8]);
 Fpdmsy = pressure101(2,[1,6,7,8]);
 plot([-flip(Fpdmsx) Dpdmsx],[flip(Fpdmsy) Dpdmsy])
 
 hold on 
 
 Dpdmsy = distance121(2,[2,4,6,7,8]);
 Fpdmsy = pressure121(2,[1,6,7,8]);
 plot([-flip(Fpdmsx) Dpdmsx],[flip(Fpdmsy) Dpdmsy])
 
legend('8:1','10:1', '12:1');
set(xlabel('-Force [N] / Distance [cm] '),'FontSize',14)
set(ylabel('16-bit ADC value'),'FontSize',14)
set(title('PDMS|40mA, 6mm'),'FontSize',14)
axis([-5 5 0 65535]);
plot([0 0],[0 65535],'r--')
print('force_distance','-dpng')
