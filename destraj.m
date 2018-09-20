function [xL_des,dxL_des, d2xL_des,d3xL_des,d4xL_des,d5xL_des,d6xL_des] = destraj(t)
    
    xL_des = 5.*[cos(t); sin(t)];
    dxL_des = 5.*[-sin(t); cos(t) ];
    d2xL_des = 5.*[-cos(t); -sin(t) ];
    d3xL_des = 5.*[ sin(t); -cos(t)];
    d4xL_des = 5.*[ cos(t); sin(t)];
    d5xL_des = 5.*[-sin(t); cos(t)];
    d6xL_des = 5.*[-cos(t); -sin(t)];
    
    %xL_des = [ sin(t/2); -cos(t/2)];
    %dxL_des = [ cos(t/2)/2 ; sin(t/2)/2 ];
    %d2xL_des = [ -sin(t/2)/4 ; cos(t/2)/4 ];
    %d3xL_des = [ -cos(t/2)/8 ; -sin(t/2)/8];
    %d4xL_des = [sin(t/2)/16 ; -cos(t/2)/16];
    %d5xL_des = [cos(t/2)/32 ; sin(t/2)/32];
    %d6xL_des = [-sin(t/2)/64 ; cos(t/2)/64];
    
    %xL_des = [ sin(2*t); -cos(2*t)];
    %dxL_des = [ cos(t*2)*2 ; sin(t*2)*2 ];
    %d2xL_des = [ -sin(t*2)*4 ; cos(t*2)*4 ];
    %d3xL_des = [ -cos(t*2)*8 ; -sin(t*2)*8];
    %d4xL_des = [sin(t*2)*16 ; -cos(t*2)*16];
    %d5xL_des = [cos(t*2)*32 ; sin(t*2)*32];
    %d6xL_des = [-sin(t*2)*64 ; cos(t*2)*64];
    
    %xL_des = 5.*[t; sin(t/2)];
    %dxL_des = 5.*[1; cos(t/2)/2 ];
    %d2xL_des = 5.*[0; -sin(t/2)/4 ];
    %d3xL_des = 5.*[ 0; -cos(t/2)/8];
    %d4xL_des = 5.*[ 0; sin(t/2)/16];
    %d5xL_des = 5.*[0; cos(t/2)/32];
    %d6xL_des = 5.*[0; -sin(t/2)/64];
end