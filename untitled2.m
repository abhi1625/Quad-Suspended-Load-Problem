function [X,tout] = PRG
t0 = 0; tend = 2;
% constants
g = 9.81; %m/s/s
mQ = 0.5; %mass of quadrotor, kg
mL = 0.08; %mass of load, kg
IQ = [2.32e-3,0,0;0,2.32e-3,0;0,0,4e-3] ;
JQ = IQ(2,2) ;
l = 1; %length of cable, m

% control gains
% mode 1
kpx = [2*5*0.1; 20*5*0.1];
kdx = [2*0.5; 10*2*0.5];
kpL = 0.7;
kdL = 0.1;
kpQ = 160;
kdQ = 56;

% mode 2
kp = 0.8;
kd = 1;
kp_phi = 20; 
kd_phi = 7;


[p, dp,d2p, d3p, d4p, f0_des, phiL0_des, dphiL0_des,d2phiL0_des,phiQ0_des, dphiQ0_des, d2phiQ0_des] = diffFlatness1(0,mQ,mL,l,g);
[xL_des dxL_des d2xL_des d3xL_des d4xL_des d5xL_des d6xL_des] = destraj(0);

xL0 = xL_des';
dxL0 = dxL_des';
xQ0 = [xL_des(1,1) xL_des(2,1)+l];
xQdot0 = [dxL_des(1,1) dxL_des(2,1)];
phiQ0 = phiQ0_des;
phiQdot0 = dphiQ0_des;

if pdist([xL0;xQ0]) == l
    phiL0 = phiL0_des;
    phiLdot0 = dphiL0_des;
    currentmode = 1;
elseif pdist([xL0,xQ0]) < l
    phiL0 = 0;
    phiLdot0 = 0;
    currentmode = 2;
end
%Initial Conditions

if currentmode ==1
    x10 = [xL0 dxL0 phiL0 phiLdot0 phiQ0 phiQdot0];
    X = [x10 0 0];
elseif currentmode == 2
    x20 = [xL0 dL0 xQ0 xQdot0 phiQ0 phiQdot0];
    X = x20;
end
%Time Span
tspan = [t0,tend];
tout = t0;
%Numerical Integrations
while t0<tend
    if currentmode == 1
        options1 = odeset('Events',@slack);
        [time,x1] = ode45(@case1,tspan,x10,options1);
        n = length(time);
        tout = [tout; time(2:n)];
        X = [X; [x1(2:n,:) zeros(n-1,2)]];
    
    %when string goes from taut to slack
        currentmode = 2;
        x20 = [x1(n,1) x1(n,2) x1(n,3) x1(n,4)...
        x1(n,1)-(l*sin(x1(n,5))) x1(n,2)+(l*cos(x1(n,5)))...
        x1(n,3)-(l*x1(n,6))*cos(x1(n,5)) x1(n,4)-l*x1(n,6)*sin(x1(n,5))...
        x1(n,7) x1(n,8)];
        t0 = time(n);
    
    elseif currentmode == 2
        options2 = odeset('Events',@taut);
        [time,x2] = ode45(@case2,tspan,x20,options2);
        n = length(time);
        tout = [tout; time(2:n)];
        X = [X; x2(2:n,:)];
        %xout = [xout;x2(2:n,:)];
    
        %when string goes from slack to taut
        currentmode=1;
        x10 = [x2(n,1) x2(n,2) ((mL*x2(n,3)+mQ*x2(n,7))/(mQ+mL)) ((mL*x2(n,4)+mQ*x2(n,8))/(mQ+mL)) acos(-(x2(n,2)-x2(n,6))/l)...
        0 x2(n,9) x2(n,10)];
        t0 = time(n);
    end
end
%differential equations
function x1dot = case1(time,x1)
    yL = x1(1, 1); zL = x1(2, 1); vyL = x1(3, 1); vzL = x1(4, 1); phiL = x1(5, 1); phiLdot = x1(6, 1); ...
    phiQ = x1(7, 1); phiQdot = x1(8, 1);
        
    [f, M] = inputs1(time,x1,kpx,kdx,g,mQ,mL,l,kpL,kdL,JQ,kpQ,kdQ);
    x1dot = zeros(8,1);
    x1dot(1,1) = vyL;
    x1dot(2,1) = vzL;
    x1dot(3,1) = ((-f*cos(dif(phiQ,phiL))-mQ*l*phiLdot^2)/(mQ+mL))*sin(phiL);
    x1dot(4,1) = ((f*cos(dif(phiQ,phiL))+mQ*l*phiLdot^2)/(mQ+mL))*cos(phiL) -g;
    x1dot(5,1) = phiLdot;
    x1dot(6,1) = f*sin(dif(phiQ,phiL))/(mQ*l);
    x1dot(7,1) = phiQdot;
    x1dot(8,1) = M/JQ;
    
end

function x2dot = case2(time,x2)
     yL = x2(1, 1); zL = x2(2, 1); vyL = x2(3, 1); vzL = x2(4, 1); yQ = x2(5, 1); zQ = x2(6, 1); ...
    vyQ = x2(7, 1); vzQ = x2(8, 1); phiQ = x2(9,1); phiQdot = x2(10,1);
        
    [f, M] = inputs2(time,x2,mQ,JQ,g,kp,kd,kp_phi,kd_phi);
    x2dot = [vyL;...
            vzL;...
            0;...
            -g;...
            vyQ;...
            vzQ;...
            -sin(phiQ)*f/mQ;...
            -g+cos(phiQ)*f/mQ;...
            phiQdot;...
            M/JQ];
end

%%events functions
function [check,isterminal,direction] = slack(t,x1)
    direction = []; %default
    isterminal = 1; %terminate integration when event occurs
    check = mL*norm([x1(3,1);x1(4,1)] + [0;g]); %Tension in string = 0
end
function[check,isterminal,direction] = taut(t,x2)
    direction =[];
    isterminal =1;
    check = pdist([x2(1,1), x2(2,1); x2(5,1),x2(6,1)]) - l; %length of string is max
end

function d = dif(a1,a2)
        d = a1-a2;
        while d> pi
            d = d - 2*pi;
        end
        while d <= -pi
            d = d + 2*pi;
        end
end 
end