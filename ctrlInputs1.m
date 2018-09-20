function [f, M] = ctrlInputs1(t, x1, g, mL, mQ, JQ, l, kpx, kdx, kpL, kdL, kpQ, kdQ)

yL = x1(1, 1); zL = x1(2, 1); vyL = x1(3, 1); vzL = x1(4, 1); phiL = x1(5, 1); phidotL = x1(6, 1); ...
    phiQ = x1(7, 1); phidotQ = x1(8, 1);

[xL_des, dxL_des, d2xL_des, d3xL_des, d4xL_des, d5xL_des, d6xL_des] = destraj(t);

[p_c, dp_c, d2p_c, d3p_c, d4p_c,f_c, phiL_c, dphiL_c, d2phiL_c, phiQ_c, dphiQ_c, d2phiQ_c] = diffFlatness(t, g, mL, mQ, l);

%load position control
A = -kpx.*([yL;zL]-xL_des) - kdx.*([vyL; vzL]-dxL_des) + d2xL_des + g.*[0; 1];
p_des = - A./norm(A);
phiL_des = atan2(-A(1, 1), A(2, 1));

B = mQ.*d2xL_des-mQ.*l.*d2p_c+mQ.*g.*[0;1];
F = (mL*(A)+B);
f = -F(1, 1)*sin(phiQ) + F(2, 1)*cos(phiQ);

%load attitude control
phiQ_des = phiL_des -(kpL-1)*(dif(phiL, phiL_des))-kdL*(phidotL-dphiL_c)+ asin(d2phiL_c*mQ*l/f);

if (abs(d2phiL_c*mQ*l/f) > 1)
    error('>1')
end
 
%quadrotor attitude control
M = JQ*(-kpQ*(phiQ-phiQ_des) - kdQ*(phidotQ-dphiQ_c)) + JQ*d2phiQ_c;

    function d = dif(x1, x2)
        d = x1-x2;
        
        while d > pi
            d = d - 2*pi;
        end
        
        while d <= -pi
            d = d + 2*pi;
        end
    end

end

