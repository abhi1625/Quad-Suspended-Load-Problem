function RPG
%%test
%constants
g = 9.81; 
mQ = 0.5; 
mL = 0.09; 
%IQ = [2.32e-3,0,0;0,2.32e-3,0;0,0,4e-3] ;
JQ = 2.32e-3 ;
l = 1; %length of cable, m

% control gains 1
kpx = [2*5*0.1; 20*5*0.1];
kdx = [2*0.5; 10*2*0.5];
kpL = 0.7;
kdL = 0.1;
kpQ = 160;
kdQ = 56;

% control gains 2
kp = 0.8;
kd = 1;
kp_phi = 20; 
kd_phi = 7;
 
% initial conditions 
t0 = 0;
tend = 25; %total time of simulation, s
[xL_des, dxL_des, d2xL_des, d3xL_des, d4xL_des, d5xL_des, d6xL_des] = destraj(0);
[p_nom, dp_nom, d2p_nom, d3p_nom, d4p_nom,f_nom, phiL_nom, dphiL_nom, d2phiL_nom, phiQ_nom, dphiQ_nom, d2phiQ_nom] = diffFlatness(0, g, mL, mQ, l);

xL0 = xL_des'; 
vL0 = dxL_des'; 
xQ0 = [xL_des(1, 1) xL_des(2, 1)+l];
vQ0 = [dxL_des(1, 1) dxL_des(2, 1)];
phiQ0 = phiQ_nom; 
phidotQ0 = dphiQ_nom; 

if pdist([xL0; xQ0]) == l
    phiL0 = phiL_nom;
    phidotL0 = dphiL_nom;
    currentMode = 1; %case 1 is when cable is taut
elseif pdist([xL0; xQ0]) < l
    phiL0 = 0;
    phidotL0 = 0;
    currentMode = 2; %case 2 is when cable is slack
end

if currentMode == 1
    x10 = [xL0 vL0 phiL0 phidotL0 phiQ0 phidotQ0];
elseif currentMode == 2
    x20 = [xL0 vL0 xQ0 vQ0 phiQ0 phidotQ0];
end

tout = t0;
if currentMode == 1
    X = [x10 0 0];
elseif currentMode == 2
    X = x20;
end
mode = currentMode;
% Numerical integration
while t0 < tend

    if currentMode == 1
        
        options1 = odeset('Events', @slack);
        [t, x1] = ode45(@case1,[t0 tend],x10,options1)%integrateMode1([t0 tend], options1, x10, g, mL, mQ, JQ, l, kpx, kdx, kpL, kdL, kpQ, kdQ);
        
        n = length(t);
        tout = [tout; t(2:n)];
        X = [X; [x1(2:n, :) zeros(n-1, 2)]];
        mode = [mode; currentMode*ones(n-1, 1)];
        % when string goes from taut to slack       
        currentMode = 2;
        x20 = [x1(n, 1) x1(n, 2) x1(n, 3) x1(n, 4) x1(n, 1)-l*sin(x1(n, 5)) x1(n, 2)+l*cos(x1(n, 5)) x1(n, 3)-l*x1(n, 6)*cos(x1(n, 5)) x1(n, 4)-l*x1(n, 6)*sin(x1(n, 5)) x1(n, 7) x1(n, 8)];
        t0 = t(n);
        
    elseif currentMode == 2
        
        options2 = odeset('Events', @taut);
        [t,x2] = ode45(@case2, [t0 tend],x20,options2);
        %[t, x2] = ([t0 tend], options2, x20, g, mQ, JQ, kp, kd, kp_phi, kd_phi); 
        
        n = length(t);
        tout = [tout; t(2:n)];
        X = [X; x2(2:n, :)];
        mode = [mode; currentMode(n-1,1)];
        % when string goes from slack to taut        
        currentMode = 1;
        x10 = [x2(n, 1) x2(n, 2) (mL*x2(n, 3)+mQ*x2(n, 7))/(mL+mQ) (mL*x2(n, 4)+mQ*x2(n, 8))/(mL+mQ) ...
            real(acos(-(x2(n, 2)-x2(n, 6))/l)) 0 x2(n, 9) x2(n, 10)];
        t0 = t(n);
        
    end
end
%%differential equations
function x1dot = case1(t,x1)
    yL = x1(1, 1); zL = x1(2, 1); vyL = x1(3, 1); vzL = x1(4, 1); phiL = x1(5, 1); phiLdot = x1(6, 1); ...
    phiQ = x1(7, 1); phiQdot = x1(8, 1);
        
    [f, M] = ctrlInputs1(t, x1, g, mL, mQ, JQ, l, kpx, kdx, kpL, kdL, kpQ, kdQ);
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

function x2dot = case2(t,x2)
     yL = x2(1, 1); zL = x2(2, 1); vyL = x2(3, 1); vzL = x2(4, 1); yQ = x2(5, 1); zQ = x2(6, 1); ...
    vyQ = x2(7, 1); vzQ = x2(8, 1); phiQ = x2(9,1); phiQdot = x2(10,1);
        
    [f, M] = ctrlInputs2(t, x2, g, mQ, JQ, kp, kd, kp_phi, kd_phi);
    x2dot = zeros(10,1);
    x2dot(1,1) = vyL;
    x2dot(2,1) = vzL;
    x2dot(3,1) = 0;
    x2dot(4,1) = -g;
    x2dot(5,1) = vyQ;
    x2dot(6,1) = vzQ;
    x2dot(7,1) = -sin(phiQ)*f/mQ;
    x2dot(8,1) = -g+(cos(phiQ)/f);
    x2dot(9,1) = phiQdot;
    x2dot(10,1)= M/JQ;
            
end

%%events functions
function [check,isterminal,direction] = slack(t,x1)
    direction = []; %default
    isterminal = 1; %terminate integration when event occurs
    check =mL*norm([x1(3,1);x1(4,1)] + [0;g]); %Tension in string = 0
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
%% plots
quad = zeros(length(tout),2);
err = zeros(length(tout),2);
for i=1:length(tout)
    if (mode(i,1) == 1)
        quad(i,:) = [(X(i,1)-l*sin(X(i,5))) (X(i,2)+l*cos(X(i,5)))];
    elseif (mode(i,1) == 2)
        quad(i,:) = [(X(i,5)) (X(i,6))];
    end
    [xL_des] = destraj(tout(i));
    xT(:,i) = xL_des;
    err(i,:) = [(X(i,1)-xT(1,i)) (X(i,2)-xT(2,i))];
end

figure()
hold on
plot(tout, X(:,1)','g');
plot(tout, X(:,2)','r');
plot(tout, xT(1,:),'b--');
plot(tout, xT(2,:),'k--');
xlabel('time(s)');ylabel('position(m)');
title('load position over time');
legend('x position','z position', 'desired x', 'desired z')

figure()
hold on
plot(tout, err(:,1),'b');
plot(tout,err(:,2),'k');
xlabel('time');ylabel('error in position');
title('psition error vs time');
legend('error y pos','error z pos')
figure()
hold on
%plot(X(:,1),X(:,2),'b');
%plot(quad(:,1) , quad(:,2),'g');
plot(xT(1,:),xT(2,:),'g');
legend('load','quad','des load traj');
xlabel('y position');
ylabel('z position')
curve1 = animatedline('Color','r','LineStyle',':');
curve2 = animatedline;
axis equal
grid minor
grid on
for v = 1:length(tout)
    addpoints(curve1,quad(v,1),quad(v,2));
    addpoints(curve2,X(v,1),X(v,2));
    drawnow
end


%figure()
%hold on
%plot(X(:,1),'g');
%plot(xT(1,:),'r--');



end
