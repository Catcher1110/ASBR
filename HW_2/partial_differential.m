function [q0_dot]=partial_differential(Jb,theta,theta0,theta_iplus1)

    Jb0            =double(subs(Jb, theta, theta0));
    Wq0            =sqrt(det(Jb0*Jb0'));
    
    for i=1:7
        
        thetanew       =theta0;        
        thetanew(i)    =theta_iplus1(i);
        Jbi            =double(subs(Jb, theta, thetanew));
        Wqi            =sqrt(det(Jbi*Jbi'));
        delta_q        =thetanew(i)-theta0(i);
        delta_w        =Wqi-Wq0;
        q0_doti        =delta_w/delta_q;
        q0_dot(i)      =real(q0_doti);
        
    end
end