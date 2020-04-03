function ellipsoid_plot(J)
Jw = J(1:3, :);
Jv = J(4:6, :);

Aw = Jw * Jw';
Av = Jv * Jv';

[~,Sw,Vw] = svd(Aw);
[~,Sv,Vv] = svd(Av);
xwr =Sw(1,1);
ywr =Sw(2,2);
zwr =Sw(3,3);
xc =0;
yc =0;
zc =0;
[xw, yw, zw] = ellipsoid(xc,yc,zc,xwr,ywr,zwr);
figure(1)
hw = surf(xw, yw, zw);
[ww, thetaw] = R2AA(Vw);
rotate(hw, ww, thetaw);
title('Angular Velocities Elliposid');

figure(2);
[xv, yv, zv] = ellipsoid(xc,yc,zc,Sv(1, 1),Sv(2, 2),Sv(3, 3));
hv = surf(xv, yv, zv);
[wv, thetav] = R2AA(Vv);
rotate(hv, wv, thetav);
title('Linear Velocities Elliposid');

end