clear;close all;clc
V_E=4;V_P=2.5;
lamda=V_P / V_E;
P=[0,3; 1,7; 7,11; 4,-1; 9,0; 15,3];
E=[5,5];
n=1000; ct=0:2*pi/n:2*pi;  %每个点之间的间隔角度
N = length(P);  % 追捕者的个数

Apolo_O = zeros([N,2]);  % 圆心
Apolo_R = zeros([N,1]);  % 半径
Apolo_X = zeros([N,length(ct)]);  % 横纵坐标，画图用
Apolo_Y = zeros([N,length(ct)]);


% 凸包
xP=P(:,1);  % generate x and y data for your clusters
yP=P(:,2);
k=convhull(xP,yP);  % generate indices marking the outermost points 
N_c = length(k)-1;  % 构成凸包的追捕者个数
inter = zeros([N_c*2,2]);  % 相邻圆之间的交点坐标
% (暂时是不正确的，因为每两个圆就有两个交点。后面后判断，留下近的那个)

Apolo_O=[(P(:,1)-power(lamda,2)*E(:,1))/(1-power(lamda,2)), ...
    (P(:,2)-power(lamda,2)*E(:,2))/(1-power(lamda,2))];
Apolo_R=lamda*sqrt(power(P(:,1)-E(:,1),2)+power(P(:,2)-E(:,2),2))/ ...
    (1-power(lamda,2));

% for i = 1:N
%     Apolo_O(i,:)=[(P(i,1)-power(lamda,2)*E(1,1))/(1-power(lamda,2)), ...
%         (P(i,2)-power(lamda,2)*E(1,2))/(1-power(lamda,2))];
%     Apolo_R(i)=lamda*sqrt(power(P(i,1)-E(1,1),2)+power(P(i,2)-E(1,2),2))/ ...
%         (1-power(lamda,2));
%     Apolo_X(i,:) = Apolo_R(i).*cos(ct)+Apolo_O(i,1);
%     Apolo_Y(i,:) = Apolo_R(i).*sin(ct)+Apolo_O(i,2);
% end

for i = 1:N_c
    % 得到交点的解析解
    syms x y
    [x,y] = solve((x-Apolo_O(k(i),1))^2+(y-Apolo_O(k(i),2))^2-Apolo_R(k(i))^2,...
        (x-Apolo_O(k(i+1),1))^2+(y-Apolo_O(k(i+1),2))^2-Apolo_R(k(i+1))^2);
    inter([2*k(i)-1;2*k(i)],1)=eval(x); inter([2*k(i)-1;2*k(i)],2)=eval(y);
end

%% 策略执行
id_fake = zeros([N_c,1]);
inter_ = inter;
inter= real(inter);
dis_E = sqrt(sum((real(inter)-E).^2,2));  % 所有交点到逃跑者的距离
id_E = [];
flag_leak=0;  % 判断是否有缺口的标志
for i = 1:N_c
    if dis_E(2*i-1) == dis_E(2*i)  % 有缺口
%         id_E(end+1,:) = i;flag_leak=1;  % 如果有多个缺口，都保存下来，
        id_E = i;flag_leak=1;  % 对应的交点的索引（只管一个缺口）
    end
    [xxx,id_fake(i)] = max([dis_E(2*i-1),dis_E(2*i)]);
    id_fake(i) = id_fake(i) + 2*(i-1);
end
inter(id_fake,:) = [];  % 删除不正确的交点
dis_E(id_fake,:) = [];

% 找最远的交点 / 有缺口的交点（有缺口的话，就不是最远的交点！）
[~,id__] = min(dis_E(id_E,:)); id_E = id_E(id__);  % 如果有多个缺口，朝最近的那个缺口跑
if ~flag_leak
    [~,id_E] = max(dis_E);
end
inter_far = inter(id_E,:);  % 最远的那个交点

% 逃跑者的运动方向(单位向量)
dir_E = inter_far - E; dir_E = dir_E / norm(dir_E);

%追捕者的运动方向(单位向量)
dir_P = repmat(dir_E,N,1);  % 初始化所有追捕者的运动方向=dir_E
dis_P = sqrt(sum((inter_far-P).^2,2));  % 所有交点到逃跑者的距离
[~,id_Prela] = mink(dis_P,2);
dir_P(id_Prela,:) = inter_far - P(id_Prela,:); % 相关追捕者的运动方向
dir_P(id_Prela,:) = dir_P(id_Prela,:) / norm(dir_P(id_Prela,:));

%% 运动

%% 作图
MSize = 12;
LWidth = 1.0;
FSize = 14;
figure(1)  % 创建figure1窗口
set(gcf,'position',[700 50 600 600], 'Color', 'white')  % 后两个范围由分辨率限制
box on
title('Apollonian Circle.','FontSize',FSize)
xlabel('X/(m)')
ylabel('Y/(m)')

% xlim([-15 30])
% ylim([-17.5 27.5])

axis equal
hold on
plot(E(1),E(2),'r*','MarkerSize',MSize)

viscircles(Apolo_O,Apolo_R,'Color','k','LineStyle','-',...
    'LineWidth',LWidth-0.5);  % 画圆
plot(P(:,1),P(:,2),'b.','MarkerSize',MSize)  % 追捕者
plot(xP(k),yP(k), 'b--', 'LineWidth', LWidth-0.5)  % 凸包，索引顺序改为k
plot(Apolo_O(:,1),Apolo_O(:,2),'k.','MarkerSize',MSize-2)  % 圆心
plot(inter(:,1),inter(:,2),'r.',...
    'MarkerSize',MSize)  % 交点
for i = 1:N
    text(P(i,1)-0.5,P(i,2)-1,['B' num2str(i)],'FontSize',FSize,'Color','b')
end
text(E(1),E(2)-1.5, 'A','FontSize', FSize,'Color','r')

% 策略
quiver(E(:,1),E(:,2),dir_E(:,1),dir_E(:,2),4,'r-',...
    'LineWidth',LWidth,'MaxHeadSize',0.5)
quiver(P(:,1),P(:,2),dir_P(:,1),dir_P(:,2),0.5,'b-',...
    'LineWidth',LWidth,'MaxHeadSize',0.5)

