clear;close all;clc
V_E=4;V_P=2;  % 逃跑者E、追捕者P的速度
lamda=V_P / V_E;  % 速度比
P=[0,3; 1,7; 7,11; 4,-1; 9,0; 15,3];  % 追捕者位置（这里是6个）
E=[5,5];
N = length(P);  % 追捕者的个数



% ApollonianCircles
Apolo_O=[(P(:,1)-power(lamda,2)*E(:,1))/(1-power(lamda,2)), ...
    (P(:,2)-power(lamda,2)*E(:,2))/(1-power(lamda,2))];  % 圆心
Apolo_R=lamda*sqrt(power(P(:,1)-E(:,1),2)+power(P(:,2)-E(:,2),2))/ ...
    (1-power(lamda,2));  % 半径

% 凸包
xP=P(:,1);  % 追捕者的x坐标
yP=P(:,2);  % 追捕者的y坐标
k=convhull(xP,yP);  % 构成凸包的索引顺序 
N_c = length(k)-1;  % 构成凸包的追捕者个数
inter = zeros([N_c*2,2]);  % 相邻圆之间的交点坐标
% (暂时是不正确的，因为相邻园有两个交点。后面会判断，留下近的那个)

for i = 1:N_c
    % 得到交点的解析解
    syms x y
    [x,y] = solve((x-Apolo_O(k(i),1))^2+(y-Apolo_O(k(i),2))^2-Apolo_R(k(i))^2,...
        (x-Apolo_O(k(i+1),1))^2+(y-Apolo_O(k(i+1),2))^2-Apolo_R(k(i+1))^2);
    inter([2*k(i)-1;2*k(i)],1)=eval(x); inter([2*k(i)-1;2*k(i)],2)=eval(y);
end

%% "贪婪最优策略（初始化）
id_fake = zeros([N_c,1]);  % 不正确的交点
inter_ = inter;  % 调试观察用
inter= real(inter);  % 虚部丢掉
dis_E = sqrt(sum((real(inter)-E).^2,2));  % 所有交点到逃跑者的距离
id_E = [];
flag_leak=0;  % 判断是否有缺口的标志
for i = 1:N_c
    if dis_E(2*i-1) == dis_E(2*i)  % 有缺口
        id_E(end+1,:) = i;flag_leak=1;  % 如果有多个缺口，都保存下来，
%         id_E = i;flag_leak=1;  % 对应的交点的索引（只管一个缺口，对应test1、2.gif）
    end
    [~,id_fake(i)] = max([dis_E(2*i-1),dis_E(2*i)]);  % 离得远的交点是不正确的交点
    id_fake(i) = id_fake(i) + 2*(i-1);
end
inter(id_fake,:) = [];  % 删除不正确的交点
dis_E(id_fake,:) = [];

% 找最远/有缺口的交点（缺口不等于最远的交点！）
[~,id__] = max(dis_E(id_E,:)); id_E = id_E(id__);  % 如果有多个缺口，朝最远的那个缺口跑
if ~flag_leak  % 没有缺口
    [~,id_E] = max(dis_E);  % 最远的交点的索引
end
inter_far = inter(id_E,:);  % 最远的那个交点

% 逃跑者的运动方向(单位向量)
dir_E = inter_far - E; dir_E = dir_E / norm(dir_E);

%追捕者的运动方向(单位向量)
dir_P = repmat(dir_E,N,1);  % 先初始化所有追捕者的运动方向=dir_E
dis_P = sqrt(sum((inter_far-P).^2,2));  % 所有交点到逃跑者的距离
[~,id_Prela] = mink(dis_P,2);  % 距离交点/缺口最近的两个追捕者（相关）
dir_P(id_Prela,:) = inter_far - P(id_Prela,:);  % 相关追捕者的运动方向
dir_P(id_Prela,:) = dir_P(id_Prela,:) / norm(dir_P(id_Prela,:));  % 单位化

%% 运动过程
MSize = 12;  % MarkerSize
LWidth = 1.0;  % LineWidth
FSize = 14;  % FontSize

dt=0.1;T=5;
Pt = zeros(2,N,T/dt+1);Et = zeros(2,T/dt+1);  % 追捕者、逃跑者的位置序列
Pt(:,:,1)=P'; Et(:,1)=E';   % 初始位置

figure(1)  % 创建figure1窗口
set(gcf,'position',[700 50 600 600], 'Color', 'white')  % 后两个范围由分辨率限制
box on
title('Apollonian Circle.','FontSize',FSize)
xlabel('X/(m)');ylabel('Y/(m)')
xlim([-20 40]);ylim([-20 40])
pic_num = 1;  % 画gif用

for t = 1:T/dt
    
    Et(:,t+1) = Et(:,t)+ dir_E'*V_E*dt;
    Pt(:,:,t+1) = Pt(:,:,t) + dir_P'*V_P*dt;

    % 更新爱是缘
    Apolo_O=[(Pt(1,:,t+1)-power(lamda,2)*Et(1,t+1))/(1-power(lamda,2)); ...
        (Pt(2,:,t+1)-power(lamda,2)*Et(2,t+1))/(1-power(lamda,2))]';
    Apolo_R=lamda*sqrt(power(Pt(1,:,t+1)-Et(1,t+1),2)+...
        power(Pt(2,:,t+1)-Et(2,t+1),2))'/(1-power(lamda,2));

    % 凸包（行列向量没太写好，所以用了很多转置）
    xPt=Pt(1,:,t+1)';
    yPt=Pt(2,:,t+1)';
    k=convhull(xPt,yPt);  % 构成凸包的索引顺序
    N_c = length(k)-1;  % 构成凸包的追捕者个数
    inter = zeros([N_c*2,2]);  % 相邻圆之间的交点坐标
    % (暂时是不正确的，因为每两个圆就有两个交点。后面后判断，留下近的那个)
    for i = 1:N_c
        % 得到交点的解析解
        syms x y
        [x,y] = solve((x-Apolo_O(k(i),1))^2+(y-Apolo_O(k(i),2))^2-Apolo_R(k(i))^2,...
            (x-Apolo_O(k(i+1),1))^2+(y-Apolo_O(k(i+1),2))^2-Apolo_R(k(i+1))^2);
        inter([2*k(i)-1;2*k(i)],1)=eval(x); inter([2*k(i)-1;2*k(i)],2)=eval(y);
    end

    id_fake = zeros([N_c,1]);
    inter_ = inter;
    inter= real(inter);
    dis_E = sqrt(sum((real(inter)-Et(:,t+1)').^2,2));  % 所有交点到逃跑者的距离
    id_E = [];
    flag_leak=0;  % 判断是否有缺口的标志
    for i = 1:N_c
        if dis_E(2*i-1) == dis_E(2*i)  % 有缺口
%             id_E(end+1,:) = i;flag_leak=1;  % 如果有多个缺口，都保存下来，
            id_E = i;flag_leak=1;  % 对应的交点的索引（只管一个缺口，对应test1、2.gif）
        end
        [xxx,id_fake(i)] = max([dis_E(2*i-1),dis_E(2*i)]);
        id_fake(i) = id_fake(i) + 2*(i-1);
    end
    inter(id_fake,:) = [];  % 删除不正确的交点
    dis_E(id_fake,:) = [];
    
    % 找最远的交点 / 有缺口的交点（缺口不一定是最远的交点！）
    [~,id__] = max(dis_E(id_E,:)); id_E = id_E(id__);  % 如果有多个缺口，朝最远的那个缺口跑
    if ~flag_leak  % 没有缺口
        [~,id_E] = max(dis_E);  % 最远的交点的索引
    end
    inter_far = inter(id_E,:);  % 最远的那个交点

    % 逃跑者的运动方向(单位向量)
    dir_E = (inter_far - Et(:,t+1)'); dir_E = dir_E / norm(dir_E);
    
    %追捕者的运动方向(单位向量)
    dir_P = repmat(dir_E,N,1);  % 初始化所有追捕者的运动方向=dir_E
    dis_P = sqrt(sum((inter_far-Pt(:,:,t+1)').^2,2));  % 所有交点到逃跑者的距离
    [~,id_Prela] = mink(dis_P,2);
    dir_P(id_Prela,:) = inter_far - Pt(:,id_Prela',t+1)'; % 相关追捕者的运动方向
    dir_P(id_Prela,:) = dir_P(id_Prela,:) / norm(dir_P(id_Prela,:));

    % 作图
    hold on
    ptmb(1)=plot(Et(1,t+1),Et(2,t+1),'co','MarkerSize',MSize-6,...
        'DisplayName','逃跑者');
    ptmb(2)=viscircles(Apolo_O,Apolo_R,'Color','k','LineStyle','-',...
        'LineWidth',LWidth-0.5);  % 画圆
    ptmb(3)=plot(Pt(1,:,t+1),Pt(2,:,t+1),'b.','MarkerSize',MSize,...
        'DisplayName','追捕者');  % 追捕者
    ptmb(4)=plot(xPt(k),yPt(k), 'b--', 'LineWidth', LWidth-0.5, ...
        'DisplayName','凸包');  % 凸包，索引顺序改为k
    ptmb(5)=plot(Apolo_O(:,1),Apolo_O(:,2),'k.','MarkerSize',MSize-2, ...
        'DisplayName','阿氏圆的圆心');  % 圆心
    ptmb(6)=plot(inter(:,1),inter(:,2),'r.','MarkerSize',MSize, ...
        'DisplayName','阿氏圆的交点');  % 交点
    
    % 策略
    ptmb(7)=quiver(Et(1,t+1),Et(2,t+1),dir_E(:,1),dir_E(:,2),'c-',...
        'LineWidth',LWidth,'MaxHeadSize',0.5, ...
        'DisplayName','逃跑者的运动方向');
    ptmb(8)=quiver(Pt(1,:,t+1)',Pt(2,:,t+1)',dir_P(:,1),dir_P(:,2),0.3, ...
        'b-','LineWidth',LWidth,'MaxHeadSize',0.5, ...
        'DisplayName','追捕者的运动方向');
    legend(ptmb([1;3;5;6;7;8]))  % 选择性展示图例

    % gif
    F=getframe(gcf);
    I=frame2im(F);
    [I,map]=rgb2ind(I,256);
    if pic_num == 1
        imwrite(I,map,'test.gif','gif','Loopcount',inf,'DelayTime',0.1);
    else 
        imwrite(I,map,'test.gif','gif','WriteMode','append','DelayTime',0.1);
    end
    
    delete(ptmb([2;4;5;6;7;8]))  % 选择性删除曲线

    %追捕成功：P、E距离小于0.3
    dis_near = sqrt(sum((Et(:,t+1)' - Pt(:,:,t+1)').^2,2));
    if min(dis_near) <= 0.3
        ptmb(4)=plot(xPt(k),yPt(k), 'b--', 'LineWidth', LWidth-0.5, ...
            'DisplayName','凸包');  % 凸包，索引顺序改为k
        F=getframe(gcf);
        I=frame2im(F);
        [I,map]=rgb2ind(I,256);
        imwrite(I,map,'test.gif','gif','WriteMode','append','DelayTime',1);
        break;
    end
    pic_num = pic_num + 1;
end
