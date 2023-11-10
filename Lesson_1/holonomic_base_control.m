function holonomic_base_control(x,y,phi)



if nargin == 0
    qd = [10,10,-3*pi/2];
elseif nargin == 3
    qd = [x,y,phi];
else
    error(['Wrong number of parameters. Try holonomic_base_control(x,y,phi),'...
        ' where (x, y, phi) is the desired configuration. Alternatively,'...
        ' type holonomic_base_control() to simulate using default parameters.']);
    
    for i = 1:2
    if qd(i) > 0
        pos{i} = 1.5*qd(i);
        neg{i} = -0.5*qd(i);
    elseif qd(i) < 0
        pos{i} = -0.5*qd(i);
        neg{i} = 1.5*qd(i);
    else
        pos{i} = 1;
        neg{i} = -1;
    end
end

axis([neg{1}, pos{1}, neg{2}, pos{2}, 0, 1]);
axis vis3d;
xlabel('X');
ylabel('Y');
hold on;

end