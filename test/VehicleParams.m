clear;
clc;
% load('VehicleParams_B_Class.mat')
load('VehicleParams_C_Class.mat')

Fy(1,:)=0;
Fy(:,1)=0;

subplot(2,1,1);
[X,Y]=meshgrid(alpha,Fz);
surf(X, Y, Fy');

subplot(2,1,2);
Fzf = lr/L*m*9.806;
Fzr = lf/L*m*9.806;
Vqf = interp2(X, Y, Fy',alpha,Fzf,'spline');
Vqr = interp2(X, Y, Fy',alpha,Fzr,'spline');
plot(alpha,Vqf,'b',alpha,Vqr,'r');
p1=polyfit(alpha(alpha<=4)*pi/180,Vqf(alpha<=4)',1);
p2=polyfit(alpha(alpha<=4)*pi/180,Vqr(alpha<=4)',1);
Cf=p1(1);
Cr=p2(1);



% Cf = diff(Vq1)./diff(alpha);
% Cr = diff(Vq2)./diff(alpha);

