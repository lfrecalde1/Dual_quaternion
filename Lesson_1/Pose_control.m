%% Control 2 DOF Robot based on dual quaternions
clc, clear all, close all;

%% Geometric parameters and initial conditions
l_1 = 1;
l_2 = 0.5;
l_3 = 0.5;
theta1(1) = -1.54;
theta2(1) = 0.7;
theta3(1) = 0.2;

%% Time definition
ts = 0.01;
t_final = 20;
t = (0:ts:t_final);

%% Forward Kinematics based on dual quaternions
[H_dual(:, 1), H_traslation_init(: ,1), H_rotation_init(:, 1)] = FK_3DOF(theta1(1),theta2(1), theta3(1), l_1, l_2, l_3);


%% Desired Dual quaternion
t_d = [0;1.25;1.25;0];
angle_d = 0.49;

%% The rotation is similiar a rotation vector formation
r_d = rotation_quaternion(angle_d, [0;0;1]);
h_d =  pose_dual(t_d,r_d);
h_d_t =  get_traslatation_dual(h_d);
h_d_r =  get_rotation_dual(h_d);
h_d = h_d.*ones(1, length(t));
h_d_t = h_d_t.*ones(1, length(t));
h_d_r = h_d_r.*ones(1, length(t));
%% Simulation system
K1 = 1*eye(8, 8);
for k = 1:length(t)
    h_e_t(:, k) = h_d(:, k) - H_dual(:, k);
    J_traslation = J_t(theta1(k), theta2(k), theta3(k), l_1, l_2, l_3);
    J_rotation = J_r(theta1(k), theta2(k), theta3(k));
    J = [J_rotation; J_traslation];
    theta_p = pinv(J)*K1*h_e_t(:, k);
    theta1(k + 1) = theta1(k) + theta_p(1)*ts;
    theta2(k + 1) = theta2(k) + theta_p(2)*ts;
    theta3(k + 1) = theta3(k) + theta_p(3)*ts;
    
    [H_dual(:, k+1), H_traslation_init(: ,k+1), H_rotation_init(:, k+1)] = FK_3DOF(theta1(k+1),theta2(k+1), theta3(k+1), l_1, l_2, l_3);
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
plot(theta1(1, :),'-','Color',[226,76,10]/255,'linewidth',1); hold on
grid on;
plot(theta2(1, :),'-','Color',[226,76,44]/255,'linewidth',1); hold on
plot(theta3(1, :),'-','Color',[100,76,44]/255,'linewidth',1); hold on
legend({'$\theta_1$', '$\theta_2$', '$\theta_2$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background

figure(3)
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);

subplot(1,1,1)
plot(H_rotation_init(1, :),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(H_rotation_init(2, :),'-','Color',[20,76,44]/255,'linewidth',1); hold on
plot(H_rotation_init(3, :),'-','Color',[226,76,100]/255,'linewidth',1); hold on
plot(H_rotation_init(4, :),'-','Color',[226,150,44]/255,'linewidth',1); hold on

plot(h_d_r(1, :),'--','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(h_d_r(2, :),'--','Color',[20,76,44]/255,'linewidth',1); hold on
plot(h_d_r(3, :),'--','Color',[226,76,100]/255,'linewidth',1); hold on
plot(h_d_r(4, :),'--','Color',[226,150,44]/255,'linewidth',1); hold on

legend({'${r}_1$','${r}_2$', '${r}_3$', '${r}_4$','${r}_{1d}$','${r}_{2d}$', '${r}_{3d}$', '${r}_{4d}$' },'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m]$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background

figure(4)
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);

subplot(1,1,1)
plot(H_traslation_init(1, :),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(H_traslation_init(2, :),'-','Color',[20,76,44]/255,'linewidth',1); hold on
plot(H_traslation_init(3, :),'-','Color',[226,76,100]/255,'linewidth',1); hold on
plot(H_traslation_init(4, :),'-','Color',[226,150,44]/255,'linewidth',1); hold on

plot(h_d_t(1, :),'--','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(h_d_t(2, :),'--','Color',[20,76,44]/255,'linewidth',1); hold on
plot(h_d_t(3, :),'--','Color',[226,76,100]/255,'linewidth',1); hold on
plot(h_d_t(4, :),'--','Color',[226,150,44]/255,'linewidth',1); hold on

legend({'${h}_1$','${h}_2$', '${h}_3$', '${h}_4$','${h}_{1d}$','${h}_{2d}$', '${h}_{3d}$', '${h}_{4d}$' },'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[m]$','Interpreter','latex','FontSize',9);
set(gcf, 'Color', 'w'); % Sets axes background