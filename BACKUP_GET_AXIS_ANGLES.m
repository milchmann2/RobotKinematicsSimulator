for i=1:3             
    sol1{i} = t_2_xyzabc(p1z{q_i}{i},1);
    sol2{i} = t_2_xyzabc(p1z{q_i-1}{i},1);

    z1{i} = sol1{i}(1);
    y1{i} = sol1{i}(2);
    x1{i} = sol1{i}(3);
    r1z{i} = rotz(z1{i});
    r1y{i} = roty(y1{i});
    r1x{i} = rotx(x1){i};
    z2{i} = sol2(1{i});
    y2{i} = sol2(2{i});
    x2{i} = sol2(3{i});
    r2z{i} = rotz(z2{i});
    r2y{i} = roty(y2{i});
    r2x{i} = rotx(x2{i});
end

for i=1:3
    T1x{i} = r1x{i}* trans(sol1{i}(4),sol1{i}(5),sol1{i}(6));
    T2x{i} = r2x{i}* trans(sol2{i}(4),sol2{i}(5),sol2{i}(6));
    T1y{i} = r1y{i}*T1x{i};
    T2y{i} = r2y{i}*T2x{i};
    T1z{i} = r1z{i}*T1y{i};
    T2z{i} = r2z{i}*T2y{i};
end

    B = B + (T1x{1}(1:4,4) * T2x{1}(1:4,4)');
    B = B + (T1x{2}(1:4,4) * T2x{2}(1:4,4)');
    B = B + (T1x{3}(1:4,4) * T2x{3}(1:4,4)'); 

    [U, S, V] = svd(B);
    M = diag([1 1 det(U) det(V)]);
    R = U*M*V';
    R(1:3,4) = 0;
    R(4,1:3) = 0;
    R(4,4) = 0;

    %R2 = [cosd(a) -sind(a) 0 0; sind(a) cosd(a) 0 0; 0 0 1 0; 0 0 0 1];
    %RD = [-sind(a) -cosd(a) 0 0; cosd(a) -sind(a) 0 0; 0 0 0 0; 0 0 0 0];
    T=[1 0 0 0; 0 cosd(a) -sind(a) 0; 0 sind(a) cosd(a) 0; 0 0 0 1];
    %RD = [R(1,2) -R(1,1), 0, 0; R(1, 1), R(1,2) 0 0; 0 0 0 0; 0 0 0 0];
    RxD = [0 0 0 0; 0 R(2,3) -R(2,2) 0; 0 R(2,2) R(2,3) 0; 0 0 0 0];
    angle = asind(R(3,2));
    accel = (RxD*R')*angle/(1/t_ipo);
    gyro1x(q_i) = accel(2,1);

