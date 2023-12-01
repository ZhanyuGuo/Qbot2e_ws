clear; close all; clc

V_E = 0.2; V_P = 0.19; % 逃跑者E、追捕者P的速度
lamda = V_P / V_E; % 速度比
P_pose = zeros([3, 3]); P = zeros([3, 2]);
E_pose = zeros([1, 3]); E = zeros([1, 2]);
N = size(P, 1); % 追捕者的个数
lim_omega = 1.5;

% Setting ROS_MASTER_URI (run roscore in local terminal first!)
rosshutdown
setenv('ROS_MASTER_URI', 'http://10.1.1.101:11311') % the address of my computer in IVCMlab

% Starting ROS MASTER
rosinit
posesub120 = rossubscriber('/vrpn_client_node/Qbot2e_120/pose', 'DataFormat', 'struct');
[posedata120, ~, ~] = receive(posesub120, 5);
posesub121 = rossubscriber('/vrpn_client_node/Qbot2e_121/pose', 'DataFormat', 'struct');
[posedata121, ~, ~] = receive(posesub121, 5);
posesub122 = rossubscriber('/vrpn_client_node/Qbot2e_122/pose', 'DataFormat', 'struct');
[posedata122, ~, ~] = receive(posesub122, 5);
posesub123 = rossubscriber('/vrpn_client_node/Qbot2e_123/pose', 'DataFormat', 'struct');
[posedata123, ~, ~] = receive(posesub123, 5);

% initial position of P and E
P_pose(1, :) = getPose(posedata120); P(1, :) = P_pose(1, 1:2);
P_pose(2, :) = getPose(posedata121); P(2, :) = P_pose(2, 1:2);
P_pose(3, :) = getPose(posedata122); P(3, :) = P_pose(3, 1:2);
E_pose(1, :) = getPose(posedata123); E(1, :) = E_pose(1, 1:2);

% figure(1)  % 创建figure1窗口
% set(gcf,'position',[700 50 600 600], 'Color', 'white')  % 后两个范围由分辨率限制
% box on
% xlabel('X/(m)');ylabel('Y/(m)')
% xlim([-5 5]);ylim([-5 5])
% plot(P(:,1), P(:,2), 'b.', 'MarkerSize', 20);hold on;
% plot(E(:,1), E(:,2), 'r.', 'MarkerSize', 20);

% ApollonianCircles
Apolo_O = [(P(:, 1) - power(lamda, 2) * E(:, 1)) / (1 - power(lamda, 2)), ...
               (P(:, 2) - power(lamda, 2) * E(:, 2)) / (1 - power(lamda, 2))]; % 圆心
Apolo_R = lamda * sqrt(power(P(:, 1) - E(:, 1), 2) + power(P(:, 2) - E(:, 2), 2)) / ...
    (1 - power(lamda, 2)); % 半径
% viscircles(Apolo_O,Apolo_R,'Color','k','LineStyle','--', 'LineWidth',1);hold on;  % 画圆
% plot(Apolo_O(:,1),Apolo_O(:,2),'k.','MarkerSize',10);  % 圆心

% 凸包
xP = P(:, 1); % 追捕者的x坐标
yP = P(:, 2); % 追捕者的y坐标
k = convhull(xP, yP); % 构成凸包的索引顺序
N_c = length(k) - 1; % 构成凸包的追捕者个数
inter = zeros([N_c * 2, 2]); % 相邻圆之间的交点坐标
% (暂时是不正确的，因为相邻园有两个交点。后面会判断，留下近的那个)
% plot(xP(k),yP(k), 'b--', 'LineWidth', 1);  % 凸包，索引顺序改为k
for i = 1:N_c
    % 得到交点的解析解
    syms x y
    [x, y] = solve((x - Apolo_O(k(i), 1)) ^ 2 + (y - Apolo_O(k(i), 2)) ^ 2 - Apolo_R(k(i)) ^ 2, ...
        (x - Apolo_O(k(i + 1), 1)) ^ 2 + (y - Apolo_O(k(i + 1), 2)) ^ 2 - Apolo_R(k(i + 1)) ^ 2);
    inter([2 * k(i) - 1; 2 * k(i)], 1) = eval(x); inter([2 * k(i) - 1; 2 * k(i)], 2) = eval(y);
end

% plot(inter(:,1),inter(:,2),'g.','MarkerSize',10);  % 交点

%% "贪婪最优策略（初始化）
id_fake = zeros([N_c, 1]); % 不正确的交点
inter_ = inter; % 调试观察用
inter = real(inter); % 虚部丢掉
dis_E = sqrt(sum((real(inter) - E) .^ 2, 2)); % 所有交点到逃跑者的距离
id_E = [];
flag_leak = 0; % 判断是否有缺口的标志

for i = 1:N_c

    if dis_E(2 * i - 1) == dis_E(2 * i) % 有缺口
        id_E(end + 1, :) = i; flag_leak = 1; % 如果有多个缺口，都保存下来，
        %         id_E = i;flag_leak=1;  % 对应的交点的索引（只管一个缺口，对应test1、2.gif）
    end

    [~, id_fake(i)] = max([dis_E(2 * i - 1), dis_E(2 * i)]); % 离得远的交点是不正确的交点
    id_fake(i) = id_fake(i) + 2 * (i - 1);
end

inter(id_fake, :) = []; % 删除不正确的交点
dis_E(id_fake, :) = [];

% 找最远/有缺口的交点（缺口不等于最远的交点！）
[~, id__] = max(dis_E(id_E, :)); id_E = id_E(id__); % 如果有多个缺口，朝最远的那个缺口跑

if ~flag_leak % 没有缺口
    [~, id_E] = max(dis_E); % 最远的交点的索引
end

inter_far = inter(id_E, :); % 最远的那个交点

% 逃跑者的运动方向(单位向量)
dir_E = inter_far - E; dir_E = dir_E / norm(dir_E);

%追捕者的运动方向(单位向量)
dir_P = repmat(dir_E, N, 1); % 先初始化所有追捕者的运动方向=dir_E
dis_P = sqrt(sum((inter_far - P) .^ 2, 2)); % 所有交点到逃跑者的距离
[~, id_Prela] = mink(dis_P, 2); % 距离交点/缺口最近的两个追捕者（相关）
dir_P(id_Prela, :) = inter_far - P(id_Prela, :); % 相关追捕者的运动方向
dir_P(id_Prela, :) = dir_P(id_Prela, :) / norm(dir_P(id_Prela, :)); % 单位化
% quiver(E(1,1),E(1,2),dir_E(:,1),dir_E(:,2),'c-','LineWidth',1,'MaxHeadSize',0.5);
% quiver(P(:,1),P(:,2)',dir_P(:,1),dir_P(:,2),0.3,'c-','LineWidth',1,'MaxHeadSize',0.5);

% calculate the velocity to be published base on 'dir'
velocity_pub120 = rospublisher('/Qbot2e_120/mobile_base/commands/velocity', 'geometry_msgs/Twist');
velocity_pub121 = rospublisher('/Qbot2e_121/mobile_base/commands/velocity', 'geometry_msgs/Twist');
velocity_pub122 = rospublisher('/Qbot2e_122/mobile_base/commands/velocity', 'geometry_msgs/Twist');
velocity_pub123 = rospublisher('/Qbot2e_123/mobile_base/commands/velocity', 'geometry_msgs/Twist');
velocity120 = rosmessage(velocity_pub120);
velocity121 = rosmessage(velocity_pub121);
velocity122 = rosmessage(velocity_pub122);
velocity123 = rosmessage(velocity_pub123);
[velocity120.Linear.X, velocity120.Angular.Z] = sendVel(V_P, dir_P(1, :), P_pose(1, :), lim_omega);
[velocity121.Linear.X, velocity121.Angular.Z] = sendVel(V_P, dir_P(2, :), P_pose(2, :), lim_omega);
[velocity122.Linear.X, velocity122.Angular.Z] = sendVel(V_P, dir_P(3, :), P_pose(3, :), lim_omega);
[velocity123.Linear.X, velocity123.Angular.Z] = sendVel(V_E, dir_E(1, :), E_pose(1, :), lim_omega);
send(velocity_pub120, velocity120);
send(velocity_pub121, velocity121);
send(velocity_pub122, velocity122);
send(velocity_pub123, velocity123);

%% 运动过程

dt = 0.1; T = 10;

% figure(1)  % 创建figure1窗口
% set(gcf,'position',[700 50 600 600], 'Color', 'white')  % 后两个范围由分辨率限制
% box on
% title('Apollonian Circle.','FontSize',FSize)
% xlabel('X/(m)');ylabel('Y/(m)')
% xlim([-20 40]);ylim([-20 40])
% pic_num = 1;  % 画gif用

for t = 1:T / dt
    [posedata, ~, ~] = receive(posesub120, 5);
    P_pose(1, :) = getPose(posedata); P(1, :) = P_pose(1, 1:2);
    [posedata, ~, ~] = receive(posesub121, 5);
    P_pose(2, :) = getPose(posedata); P(2, :) = P_pose(2, 1:2);
    [posedata, ~, ~] = receive(posesub122, 5);
    P_pose(3, :) = getPose(posedata); P(3, :) = P_pose(3, 1:2);
    [posedata, ~, ~] = receive(posesub123, 5);
    E_pose(1, :) = getPose(posedata); E(1, :) = E_pose(1, 1:2);

    Apolo_O = [(P(:, 1) - power(lamda, 2) * E(:, 1)) / (1 - power(lamda, 2)), ...
                   (P(:, 2) - power(lamda, 2) * E(:, 2)) / (1 - power(lamda, 2))]; % 圆心
    Apolo_R = lamda * sqrt(power(P(:, 1) - E(:, 1), 2) + power(P(:, 2) - E(:, 2), 2)) / ...
        (1 - power(lamda, 2)); % 半径
    % viscircles(Apolo_O,Apolo_R,'Color','k','LineStyle','--', 'LineWidth',1);hold on;  % 画圆
    % plot(Apolo_O(:,1),Apolo_O(:,2),'k.','MarkerSize',10);  % 圆心
    % 凸包
    xP = P(:, 1); % 追捕者的x坐标
    yP = P(:, 2); % 追捕者的y坐标
    k = convhull(xP, yP); % 构成凸包的索引顺序
    N_c = length(k) - 1; % 构成凸包的追捕者个数
    inter = zeros([N_c * 2, 2]); % 相邻圆之间的交点坐标

    for i = 1:N_c
        % 得到交点的解析解
        syms x y
        [x, y] = solve((x - Apolo_O(k(i), 1)) ^ 2 + (y - Apolo_O(k(i), 2)) ^ 2 - Apolo_R(k(i)) ^ 2, ...
            (x - Apolo_O(k(i + 1), 1)) ^ 2 + (y - Apolo_O(k(i + 1), 2)) ^ 2 - Apolo_R(k(i + 1)) ^ 2);
        inter([2 * k(i) - 1; 2 * k(i)], 1) = eval(x); inter([2 * k(i) - 1; 2 * k(i)], 2) = eval(y);
    end

    id_fake = zeros([N_c, 1]);
    inter_ = inter;
    inter = real(inter);
    dis_E = sqrt(sum((real(inter) - E) .^ 2, 2)); % 所有交点到逃跑者的距离
    id_E = [];
    flag_leak = 0; % 判断是否有缺口的标志

    for i = 1:N_c

        if dis_E(2 * i - 1) == dis_E(2 * i) % 有缺口
            %             id_E(end+1,:) = i;flag_leak=1;  % 如果有多个缺口，都保存下来，
            id_E = i; flag_leak = 1; % 对应的交点的索引（只管一个缺口，对应test1、2.gif）
        end

        [xxx, id_fake(i)] = max([dis_E(2 * i - 1), dis_E(2 * i)]);
        id_fake(i) = id_fake(i) + 2 * (i - 1);
    end

    inter(id_fake, :) = []; % 删除不正确的交点
    dis_E(id_fake, :) = [];

    % 找最远的交点 / 有缺口的交点（缺口不一定是最远的交点！）
    [~, id__] = max(dis_E(id_E, :)); id_E = id_E(id__); % 如果有多个缺口，朝最远的那个缺口跑

    if ~flag_leak % 没有缺口
        [~, id_E] = max(dis_E); % 最远的交点的索引
    end

    inter_far = inter(id_E, :); % 最远的那个交点

    % 逃跑者的运动方向(单位向量)
    dir_E = (inter_far - E); dir_E = dir_E / norm(dir_E);

    %追捕者的运动方向(单位向量)
    dir_P = repmat(dir_E, N, 1); % 初始化所有追捕者的运动方向=dir_E
    dis_P = sqrt(sum((inter_far - P) .^ 2, 2)); % 所有交点到逃跑者的距离
    [~, id_Prela] = mink(dis_P, 2);
    dir_P(id_Prela, :) = inter_far - P(id_Prela, :); % 相关追捕者的运动方向
    dir_P(id_Prela, :) = dir_P(id_Prela, :) / norm(dir_P(id_Prela, :));

    [velocity120.Linear.X, velocity120.Angular.Z] = sendVel(V_P, dir_P(1, :), P_pose(1, :), lim_omega);
    [velocity121.Linear.X, velocity121.Angular.Z] = sendVel(V_P, dir_P(2, :), P_pose(2, :), lim_omega);
    [velocity122.Linear.X, velocity122.Angular.Z] = sendVel(V_P, dir_P(3, :), P_pose(3, :), lim_omega);
    [velocity123.Linear.X, velocity123.Angular.Z] = sendVel(V_E, dir_E(1, :), E_pose(1, :), lim_omega);
    send(velocity_pub120, velocity120);
    send(velocity_pub121, velocity121);
    send(velocity_pub122, velocity122);
    send(velocity_pub123, velocity123);

    %% 追捕成功：P、E距离小于0.3
    %     dis_near = sqrt(sum((E - P).^2,2));
    %     if max(dis_near) <= 0.5
    %         break;
    %     end

    % capture fail - E is out of the convex
    %     if ~inpolygon(E(1),E(2),P(k,1),P(k,2))
    %         break;
    %     end
end
