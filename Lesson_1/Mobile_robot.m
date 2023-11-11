clc, clear all, close all

%% Sample Time Definition
dt = 0.1;  
total_time = 40; 
t = (0: dt:total_time);
a = 0.1;
x_init = 1;
y_init = 1;

angle_init = 0;


%% Initial Contidion vector space
x = x_init + a*cos(angle_init);
y = y_init + a*sin(angle_init);
theta = angle_init;
h = [x; y; theta];

%% Control actions
vx = 0*ones(1, length(t));  


r = 0*sin(2*t);

K = -0.1*eye(2);
%% Iterate for each time step
for i = 1:length(t)
    pos_x_e(i) = h(1);
    pos_y_e(i) = h(2);
    pos_theta_e(i) = h(3);
    h(3) = Angulo(h(3));
    
    %% Get error
    he = [h(1); h(2)];
    J_control = [cos(h(3)), -a*sin(h(3));...
               sin(h(3)), a*cos(h(3))];
    u = (pinv(J_control)*K*he);
    
    vx(1, i) = u(1);
    r(1, i) = u(2);
    
    J = [cos(h(3)), -a*sin(h(3));...
        sin(h(3)), a*cos(h(3));...
        0, 1];
    
    hp = J*[vx(i); r(i)];
    
    h = h + hp*dt;
    

end

fprintf('Final Position (x, y): (%f, %f)\n', pos_x_e(i), pos_y_e(i));
fprintf('Final Orientation (theta): %f\n', pos_theta_e(i));

figure
hold on
plot(pos_theta_e,'-')


figure
plot(vx,'-')
hold on;
plot(r,'--')

figure
hold on 
plot(pos_x_e, pos_y_e, '-')