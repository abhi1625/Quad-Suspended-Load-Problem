function [f, M] = ctrlInputs2(t, x2, g, mQ, JQ, kp, kd, kp_phi, kd_phi)
        yL = x2(1, 1); zL = x2(2, 1); vyL = x2(3, 1); vzL = x2(4, 1); 
        yQ = x2(5, 1); zQ = x2(6, 1); vyQ = x2(7, 1); vzQ = x2(8, 1); 
        phiQ = x2(9, 1); phidotQ = x2(10, 1);
        
        [xL_des, dxL_des, d2xL_des, d3xL_des, d4xL_des] = destraj(t);
        
        %load position control
        F1 =  mQ*g.*[0; 1] + mQ.*d2xL_des;
        F = -kp*([yQ; zQ] - xL_des) - kd*([vyQ; vzQ] - dxL_des) + F1;
        f = - F(1, 1)*sin(phiQ) + F(2, 1)*cos(phiQ);
        phiQ_des = atan2(-F(1, 1), F(2, 1));
        
        F_nom = mQ*d2xL_des + mQ.*g.*[0; 1];
        phiQ_nom = atan2(-F_nom(1, 1), F_nom(2, 1));
        f_nom = -F_nom(1, 1)*sin(phiQ_nom)+F_nom(2, 1)*cos(phiQ_nom);
        
        %quad attitude control
        phidotQ_des = - mQ/f_nom * (d3xL_des(1, 1)*cos(phiQ_nom) + d3xL_des(2, 1)*sin(phiQ_nom));        
        phiddotQ_des = - mQ/f_nom * (d4xL_des(1, 1)*cos(phiQ_nom) + d4xL_des(2, 1)*sin(phiQ_nom))-2*mQ^2*phidotQ_des/f_nom^2 * (d2xT(1, 1)*d3xT(1, 1) + (d2xT(2, 1) + g) * d3xT(2, 1));
        
        M_des = JQ*phiddotQ_des;
        M = JQ* (-kp_phi*(phiQ-phiQ_des)-kd_phi*(phidotQ-phidotQ_des)) + M_des;
        
        
end

