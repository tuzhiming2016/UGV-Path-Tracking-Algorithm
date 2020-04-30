function Reference = getTargetCourseParams(roadmap_name)
    if roadmap_name == "eight"
        ds = 1;
        R = 60;
        dtheta = ds/R;
        theta1 = -pi/2:dtheta:-pi/2+2*pi-dtheta;
        x1 = R*cos(theta1);
        y1 = R+R*sin(theta1);
        theta2 = pi/2:-dtheta:pi/2-2*pi;
        x2 = R*cos(theta2);
        y2 = -R+R*sin(theta2);
        cx = [x1,x2];
        cy = [y1,y2];
        Reference.cx=cx;
        Reference.cy=cy;
%         Reference = load('eight');
    end
    if roadmap_name == "road"
        Reference = load('road');
    end
    if roadmap_name == "double"
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
        Reference.cx=cx;
        Reference.cy=cy;
%         Reference = load('double');
    end