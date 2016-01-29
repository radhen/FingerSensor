D=load('red_ycb_cup.txt');
w=12; % distance between sensors (cm)
load ('fit_individualsensor','f');
% Read measurements
R=[D(:,2:8) D(:,10:16)];

% Read base values
B=load('baseValues_14sensors.txt');
B=[B(:,2:8) B(:,10:16)];
B=repmat(mean(B),size(R,1),1);

% Convert into decibles
Rdb=10*log10(R./B);


for i=1:14
    Rm(:,i)=((Rdb(:,i)-f{i}.c)/f{i}.a).^(1/(f{i}.b));
end

% Filter sensor values
Fvalues=abs(Rm)<w/2;
Z=repmat([7 6 5 4 3 2 1 7 6 5 4 3 2 1],size(D,1),1);

X=(w/2-Rm).*[repmat(cos(D(:,1)),1,7) repmat(cos(D(:,1)+pi),1,7)];
Y=(w/2-Rm).*[repmat(sin(D(:,1)),1,7) repmat(sin(D(:,1)+pi),1,7)];

% subplot(1,2,1)
% surf(X,Y,Z); axis([-10 10 -10 10 0 10])

hold on
%subplot(1,2,1)
for I=1:14, 
 for J=1:size(Rm,1),
    if(Fvalues(J,I)),
     if(I<=7) 
         plot3(X(J,I),Y(J,I),Z(J,I),'g.'); 
     else
         plot3(X(J,I),Y(J,I),Z(J,I),'r.','markersize',12);
     end;
    end;
 end;    
end;
drawnow; 


axis([-10 10 -10 10 0 10])
hold off