function [f, M] = inputs1(t,x1,kpx,kdx,g,mQ,mL,l,kpL,kdL,JQ,kpQ,kdQ)
    yL = x1(1, 1); zL = x1(2, 1); vyL = x1(3, 1); vzL = x1(4, 1); phiL = x1(5, 1); phiLdot = x1(6, 1); ...
    phiQ = x1(7, 1); phiQdot = x1(8, 1);
    
    [xL_des,dxL_des, d2xL_des,d3xL_des,d4xL_des,d5xL_des,d6xL_des] = destraj(t); 
    [p_des, dp_des,d2p_des, d3p_des, d4p_des, f_des, phiL_des, phiLdot_des,phiL2dot_des,phiQ_des, phiQdot_des, phiQ2dot_des] = diffFlatness1(t,mQ,mL,l,g); %load position controller
    %A = -kpx.*([yL;zL] - xL_des) - kdx.*([vyL;vzL] - dxL_des) + d2xL_des + g.*[0;1];
    %B = mQ.*(g.*[0;1]) + mL.*d2xL_des - mQ.*(l.*d2p_des);
    %Fx = ((mL.*A)+B);
    %f = -Fx(1,1)*sin(phiQ)+ Fx(2,1)*cos(phiQ);
    %p_des = -A./norm(A);
    %phiL_des =  atan2(-A(1,1),A(2,1));
    
    A = -kpx.*([yL;zL]-xL_des) - kdx.*([vyL; vzL]-dxL_des) + d2xL_des + g.*[0; 1];
    B = mQ.*d2xL_des-mQ.*l.*d2p_des+mQ.*g.*[0;1];
    F = (mL*(A)+B);
    f = -F(1, 1)*sin(phiQ) + F(2, 1)*cos(phiQ);
    p_des = - A./norm(A);
    phiL_des = atan2(-A(1, 1), A(2, 1));

    %load attitude controller
    Z = -(kpL-1)*(dif(phiL,phiL_des)) - kdL*(phiLdot - phiLdot_des) + asin(phiL2dot_des*mQ*l/f);
    phiQ_des = phiL_des + Z;
    
    if (abs(phiL2dot_des*mQ*l/f) > 1)
        error('greater than 1')
    end
    %quad attitude controller
    M = JQ*(-kpQ*(dif(phiQ,phiQ_des)) - kdQ*(phiQdot - phiQdot_des) + phiQ2dot_des);
    
    
    
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