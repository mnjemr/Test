function [dydt] = attitude_dynamics(t, Y)
    
    global we wo In Gta kmg I mmax kw md_monitor t_monitor
    persistent idx;   
                        
    if t == 0        
        idx = 1;
    end 

    q = Y(1:4,1);
    w = Y(5:7,1);
        
    TBO=quat2dcm([Y(4) Y(1:3)']); %orbit->body direction cosine matrix
    Beta1=we*t;
    b=TBO*magField(wo*t,In,Gta,Beta1,kmg);
    
    wr=Y(5:7)-TBO*[0;-wo;0];    
    % Don't understand the next two commented lines, but the result is the
    % same with the more standard code later
%     dydt(1:3,1)=1/2*(Y(4)*wr-cross(wr,Y(1:3)));
%     dydt(4,1)=-1/2*wr'*Y(1:3);
    
    dydt(1:4,1) = 1/2 * [    0   wr(3)  -wr(2)  wr(1); 
                         -wr(3)     0    wr(1)  wr(2);
                          wr(2) -wr(1)      0   wr(3);
                         -wr(1) -wr(2)  -wr(3)     0] * q;
    
    Md = getMC(kw,b,wr,mmax);
    
    %Md = [-20 0 0]';
                   
    dydt(5:7,1) = I\(cross(Md,b)-cross(w,I*w));
    
    md_monitor(:,idx) = Md;
    t_monitor(idx) = t;
    
    idx = idx + 1;
end

% Adding some test for version control purposes
