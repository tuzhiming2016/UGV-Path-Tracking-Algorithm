function Reference = splinfy(Reference)
    cx = Reference.cx;
    cy = Reference.cy;
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
    
    ds = 0.1;
    s = 0:ds:l(end);
    
    x = ppval(ppx, s);
    y = ppval(ppy, s);
    k = ((ppval(dppx,s).*ppval(ddppy,s)-ppval(dppy,s).*ppval(ddppx,s))) ./ (ppval(dppx,s).^2+ppval(dppy,s).^2).^(3/2);
    yaw = atan2(ppval(dppy,s),ppval(dppx,s));
    yaw = wrapTo2Pi(yaw);
    yaw(abs(yaw-2*pi)<0.001) = 0;
    r = 1./k;
    
    screen_size = get(groot, 'ScreenSize');
    screen_width = screen_size(3);
    screen_height = screen_size(4);
    
    rshow=r;
    rshow(r>1000)=0;rshow(r<-1000)=0;
    figure('name','TargetCourse','Position',...
        [0 0 3/4*screen_width 3/4*screen_height]);
    subplot(1,2,1)
    plot(cx,cy,'b*',x,y,'r');
    title('Reference Course','FontSize',12,'FontWeight','bold');
    xlabel('X(m)');ylabel('Y(m)');
    subplot(3,2,2)
    plot(s,k,'r');
    title('Reference Curvature','FontSize',12,'FontWeight','bold');
    xlabel('S(m)');ylabel('Curvature(m^(-1))');
    subplot(3,2,4);
    plot(s,yaw,'b');
    title('Reference Yaw','FontSize',12,'FontWeight','bold');
    xlabel('S(m)');ylabel('Yaw(deg)');
    subplot(3,2,6);
    plot(s,rshow,'k');
    title('Reference Radius','FontSize',12,'FontWeight','bold');
    xlabel('S(m)');ylabel('R(m)');
    
    Reference.cx = x;
    Reference.cy = y;
    Reference.cyaw = yaw;
    Reference.ck = k;
    Reference.s = s;
    Reference.r = r;
    Reference.ds = ds;