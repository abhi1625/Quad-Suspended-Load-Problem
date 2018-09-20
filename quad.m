function quad
clc
close all
clear all

[t,x,te,y,ie] = ode45(@h, tspan, IC, options)
%Case 1: When String is taut
    function xdot = h(t,x1)
        yL = x1(1, 1); zL = x1(2, 1); vyL = x1(3, 1); vzL = x1(4, 1); phiL = x1(5, 1); phiLdot = x1(6, 1); ...
    phiQ = x1(7, 1); phiQdot = x1(8, 1);
        
        [f, M] = inputs1(t,x1,kpx,kdx,g,mQ,mL,l,kpL,kdL,JQ,kpQ,kdQ);
        xdot = [vyL;
                vzL;
                {(-f*cos(dif(phiQ,phiL))-mQ*l*phiLdot^2)/(mQ+mL)}*sin(phiL);
                {(f*cos(dif(phiQ,phiL))+mQ*l*phiLdot^2)/(mQ+mL)}*cos(phiL) -g;
                phiLdot;
                f*sin(dif(phiQ,phiL))/(mQ*l);
                phiQdot;
                M/JQ];
            
            
            
            
            
    function d = dif(a1,a2)
        d = a1-a2;
        while d> pi
            d = d - 2*pi;
        end
        while d <= -pi
            d = d + 2*pi;
        end
    end