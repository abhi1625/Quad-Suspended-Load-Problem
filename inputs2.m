function [f, M] = ctrlinputs2(t,x2,mQ,JQ,g,kp,kd,kp_phi,kd_phi)
yL = x2(1, 1); zL = x2(2, 1); vyL = x2(3, 1); vzL = x2(4, 1); 
        yQ = x2(5, 1); zQ = x2(6, 1); vyQ = x2(7, 1); vzQ = x2(8, 1); 
        phiQ = x2(9, 1); phidotQ = x2(10, 1);
        
    [xL_des,dxL_des, d2xL_des,d3xL_des,d4xL_des,~,~] = destraj(t);     
    %%quad position controller
    F1 = (mQ*g).*[0 1]' + mQ.*d2xL_des;
    F = -kp.*([yL;zL] - xL_des) - kd.*([vyL;vzL] - dxL_des) + F1;
    f = -F(1,1)*sin(phiQ) + F(2,1)*cos(phiQ);
    phiQ_des =  atan2(-F(1,1),F(2,1));
    phiQ_c = atan2(-F1(1,1),F1(2,1));
    
    %differential flatness in terms of f_des
    f_des = -F1(1,1)*sin(phiQ_c) + F1(2,1)*cos(phiQ_c);
    
    phiQdot_des = - mQ/f_des * (d3xL_des(1, 1)*cos(phiQ_c) + d3xL_des(2, 1)*sin(phiQ_c));        
    phiQ2dot_des = - mQ/f_des * (d4xL_des(1, 1)*cos(phiQ_c) + d4xL_des(2, 1)*sin(phiQ_c)) - (2*mQ^2*phiQdot_des)/(f_des)^2 * (d2xL_des(1, 1)*d3xL_des(1, 1) + (d2xL_des(2, 1) + g) * d3xL_des(2, 1));
        
    %quad attitude controller
    M_des = JQ*phiQ2dot_des;
    M = JQ* (-kp_phi*(phiQ-phiQ_des)-kd_phi*(phidotQ-phiQdot_des)) + M_des;

end
    