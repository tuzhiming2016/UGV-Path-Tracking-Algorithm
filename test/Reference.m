clear;
clc;
course = "eight"; % road eight double test
%% double lane change course
if course == "double"
    s1 = 65;
    s2 = 30;
    s3 = 25;
    s4 = 25;
    s5 = 65;

    ds = 0.5;
    D = 3.5;
    x1 = 0:ds:s1;
    y1 = 0*x1;
    x2 = s1+ds:ds:s1+s2;
    h1 = 30/2/pi*sin(2*pi*(x2-65)/30);
    y2 = (D/30)*(x2-65-h1);
    x3 = s1+s2+ds:ds:s1+s2+s3;
    y3 = D*ones(1,length(x3));
    x4 = s1+s2+s3+ds:ds:s1+s2+s3+s4;
    h2 = 25/2/pi*sin(2*pi*(x4-120)/25);
    y4 = D-(D/25)*(x4-120-h2);
    x5 = s1+s2+s3+s4+ds:ds:s1+s2+s3+s4+s5;
    y5 = 0*x5;

    cx=[x1,x2,x3,x4,x5];
    cy=[y1,y2,y3,y4,y5];
end

%% road course
if course == "road"
    load road.mat;
end

%% eight course
if course == "eight"
    ds = 1;
    R = 60;
    dtheta = ds/R;
    theta1 = -pi/2:dtheta:-pi/2+2*pi-dtheta;
    x1 = R*cos(theta1);
    y1 = R+R*sin(theta1);
    theta2 = pi/2:-dtheta:pi/2-2*pi-dtheta;
    x2 = R*cos(theta2);
    y2 = -R+R*sin(theta2);
    cx = [x1,x2];
    cy = [y1,y2];
end
%% test
if course == "test"
    theta = 0:0.1:2*pi;
    R=100;
    cx=R*cos(theta);
    cy=R*sin(theta);
end
%% Splinfy
n = 0:1:length(cx)-1;
N = length(n);

dl = sqrt(diff(cx).^2+diff(cy).^2);
l = cumsum(dl);
l = [0 l];
ppx = spline(l, cx);
ppy = spline(l, cy);

dppx = ppx;
dppx.coefs=ppx.coefs(:,1:end-1).*kron(ones(size(ppx.coefs,1),1),(size(ppx.coefs,2)-1):-1:1);
dppx.order = ppx.order-1;
dppy = ppy;
dppy.coefs = ppy.coefs(:,1:end-1).*kron(ones(size(ppy.coefs,1),1),(size(ppy.coefs,2)-1):-1:1);
dppy.order = ppy.order-1;

ddppx = dppx;
ddppx.coefs=dppx.coefs(:,1:end-1).*kron(ones(size(dppx.coefs,1),1),(size(dppx.coefs,2)-1):-1:1);
ddppx.order = dppx.order-1;
ddppy = dppy;
ddppy.coefs = dppy.coefs(:,1:end-1).*kron(ones(size(dppy.coefs,1),1),(size(dppy.coefs,2)-1):-1:1);
ddppy.order = dppy.order-1;

figure(1);
ds = 0.1;
s = 0:ds:l(end);
subplot(4,1,1)
x = ppval(ppx, s);
y = ppval(ppy, s);
plot(cx,cy,'b*',x,y,'r');
subplot(4,1,2)
k = ((ppval(dppx,s).*ppval(ddppy,s)-ppval(dppy,s).*ppval(ddppx,s))) ./ (ppval(dppx,s).^2+ppval(dppy,s).^2).^(3/2);
r = 1./k;
r(r>10000)=10000;
r(r<-10000)=-10000;
plot(s,k,'r');
subplot(4,1,3);
yaw = atan2(ppval(dppy,s),ppval(dppx,s));
yaw = wrapToPi(yaw);
% yaw(find(abs(yaw-2*pi)<0.001)) = 0;
plot(s,yaw,'b');
subplot(4,1,4);
plot(s,r,'k');

figure(2)
[x, y, yaw, k, s, ~] = CalcSplineCourse(cx, cy, ds);
yaw=wrapToPi(yaw);
ds = 0.1;
subplot(4,1,1)
plot(cx,cy,'b*',x,y,'r');
subplot(4,1,2)
plot(s,k,'r');
subplot(4,1,3);
plot(s,yaw,'b');
subplot(4,1,4);
r=1./k;
r(r>10000)=10000;
r(r<-10000)=-10000;
plot(s,r,'k');