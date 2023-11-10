%% Control 2 DOF Robot based on dual quaternions
clc, clear all, close all;

%% geometric parameters of the robot
l_1 = 1;
l_2 = 0.5;
theta1(1) = -1.54;
theta2(1) = 0.7;
%% Time definition
ts = 0.01;
t_final = 10;
t = (0:ts:t_final);
[H_traslation_init(: ,1), H_rotation_init(:, 1)] = forward_kinematics_2dof(theta1(1), theta2(1), l_1, l_2);
%% Desired Dual quaternion
t_d = [0;1;0;0];
angle_d = 0;

%% The rotation is similiar a rotation vector formation
r_d = rotation_quaternion(angle_d, [0;0;1]);
h_d =  pose_dual(t_d,r_d);
h_d_t =  get_traslatation_dual(h_d);
h_d_r =  get_rotation_dual(h_d);

K1 = 1*eye(4, 4);
for k = 1:length(t)
   h_e_t(:, k) = h_d_t - H_traslation_init(:, k);
   J = [0, 0;...
        -l_2*sin(theta1(k) + theta2(k)) - l_1*sin(theta1(k)), -l_2*sin(theta1(k) + theta2(k));...
         l_2*cos(theta1(k) + theta2(k)) + l_1*cos(theta1(k)),  l_2*cos(theta1(k) + theta2(k));...
         0, 0];
   
   theta_p = pinv(J)*K1*h_e_t(:, k);
   theta1(k + 1) = theta1(k) + theta_p(1)*ts;
   theta2(k + 1) = theta2(k) + theta_p(2)*ts;
   [H_traslation_init(: ,k+1), H_rotation_init(:, k+1)] = forward_kinematics_2dof(theta1(k+1), theta2(k+1), l_1, l_2);
end

figure(1)
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);

subplot(1,1,1)
plot(h_e_t(1, :),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(h_e_t(2, :),'-','Color',[20,76,44]/255,'linewidth',1); hold on
plot(h_e_t(3, :),'-','Color',[226,76,100]/255,'linewidth',1); hold on
plot(h_e_t(4, :),'-','Color',[226,150,44]/255,'linewidth',1); hold on
legend({'$\tilde{h}_1$','$\tilde{h}_2$', '$\tilde{h}_3$', '$\tilde{h}_4$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m]$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background

figure(2)
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);

subplot(1,1,1)
plot(theta1(1, :),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(theta2(1, :),'-','Color',[226,76,44]/255,'linewidth',1); hold on
legend({'$\theta_1$', '$\theta_2$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background
