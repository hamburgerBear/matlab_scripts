clc;
clear;
close all;
% 求解思路:
% 参考:https://www.cnblogs.com/agvcfy/p/9498556.html?utm_source=debugrun&utm_medium=referral
% 1.已知车体中心速度v,w，求解舵轮的a,v
% 2.车体中心与舵轮在瞬时视为刚体连接，围绕相同旋转中心旋转，w相等
% 3.r=v/w
% 4.为了求得舵轮的v，需要根据几何图形求解出舵轮的旋转半径r，v=r*w

function out = sign(in)
  if in >= 0.0
    out = 1.0;
  else 
    out = -1.0;
  end
end

% angle = to - from
function ret = normalize_angle(angle)
  if angle > pi
    angle = angle - 2*pi;
  elseif angle < -pi
    angle = angle + 2*pi;
   end
   ret = angle;
end 

function dir = calcDirection(start, goal, pose)
    x1 = start(1);
    y1 = start(2);
    x2 = goal(1);
    y2 = goal(2);
    Px = pose(1);
    Py = pose(2);
    ax = x2 - x1;
    ay = y2 - y1;
    bx = Px - x1;
    by = Py - y1;
    if ax*by-bx*ay > 0.0
      dir = 1.0;
    else 
      dir = -1.0;
    end
end 

function [R2, V2, A2, R1, V1, A1] = solve(v, w, A, visual)
  %机器人参数
  %D = 1.0;
  D = 0.50008*2;
  car_center = [0, 0];
  front_steer = [D/2, 0];
  rear_steer = [-D/2, 0];
  weight = 0.6;
  lenght = 1.2;
	R = v / w;
  rotate_center = [0+cos(A+pi/2)*R, 0+sin(A+pi/2)*R]; %旋转中心计算
    
	% R2前R1后
	R2_square = power([D/2+R*cos(pi/2-A)], 2) + power([R*sin(pi/2-A)], 2);
	R2 = sqrt(R2_square);
	%考虑方向
	R2 = sign(R) * R2
	V2 = R2 * w;
	% check this
	tmp_theta2 = atan([R*sin(pi/2-A)]/[D/2+R*cos(pi/2-A)]);
	A2 = pi/2 - tmp_theta2;
	%考虑方向 check this 
	%A2 = A2 * sign(R) * 1.0;
  diff = normalize_angle( atan2(rotate_center(2)-front_steer(2), rotate_center(1)-front_steer(1)) - A2 );
  if (abs(abs(diff) - pi/2)) > 0.001 
    A2 = A2 * -1.0;
  end 
  
  dir = calcDirection(front_steer, [front_steer(1)+cos(A2)*V2, front_steer(2)+sin(A2)*V2], rotate_center);
  if dir ~= sign(R2)
    A2 = A2 + pi; %注意归一化 
  end
  
	R1_square = power([D/2-R*cos(pi/2-A)], 2) + power([R*sin(pi/2-A)], 2);
	R1 = sqrt(R1_square);
	%考虑方向
	R1 = sign(R) * R1;
	V1 = R1 * w;
	% check this
	tmp_theta1 = atan([R*sin(pi/2-A)]/[D/2-R*cos(pi/2-A)]);
	A1 = pi/2 - tmp_theta1;
	%考虑方向 check this 
	%A1 = A1 * sign(R) * -1.0;
  diff = normalize_angle( atan2(rotate_center(2)-rear_steer(2), rotate_center(1)-rear_steer(1)) - A1 );
  if (abs(abs(diff) - pi/2)) > 0.001 
    A1 = A1 * -1.0;
  end 
  dir = calcDirection(rear_steer, [rear_steer(1)+cos(A1)*V1, rear_steer(2)+sin(A1)*V1], rotate_center);
  if dir ~= sign(R1)
    A1 = A1 + pi; %注意归一化 
  end
  
  %可视化
  if visual
    figure(1)
    plot ([-lenght, lenght],[weight, weight], 'g')
    hold on;
    plot ([-lenght, lenght],[-weight, -weight], 'g')
    plot ([lenght, lenght],[-weight, weight], 'g')
    plot ([-lenght, -lenght],[-weight, weight], 'g')
    plot ([-lenght, lenght],[0, 0], 'b--')
    quiver(0,0,cos(A)*v,sin(A)*v) %车体速度向量
    quiver(D/2,0,cos(A2)*V2,sin(A2)*V2) %车前舵轮速度向量
    quiver(-D/2,0,cos(A1)*V1,sin(A1)*V1) %车前舵轮速度向量
    plot(rotate_center(1), rotate_center(2), 'bo')
    plot ([front_steer(1), rotate_center(1)],[front_steer(2), rotate_center(2)], 'b')
    plot ([rear_steer(1), rotate_center(1)],[rear_steer(2), rotate_center(2)], 'b')
    plot ([rear_steer(1), rotate_center(1)],[rear_steer(2), rotate_center(2)], 'b')
    plot ([car_center(1), rotate_center(1)],[car_center(2), rotate_center(2)], 'b')
    
    
    text(car_center(1),car_center(2),'car_center')
    text(front_steer(1),front_steer(2),'front_steer')
    text(rear_steer(1),rear_steer(2),'rear_steer')
    %根据圆心和半径画圆
    theta = 0:pi/20:2*pi; %角度[0,2*pi] 
    x = rotate_center(1)+R*cos(theta);
    y = rotate_center(2)+R*sin(theta);
    plot(x,y,'-')
    %hold off
  end 
end 

%单次使用方法
%visual = 1;
%[R2, V2, A2, R1, V1, A1] = solve(0.3, 0.3, 0, visual)

%for A = -pi:0.17453:pi
%  visual = 1;
%  A
%  
%  [R2, V2, A2, R1, V1, A1] = solve(0.3, 0.3, A, visual)
%  pause(0.1); 
%end

for w = -0.3:0.03:0.3
  visual = 1;
  v = 0.3;
  w
  R=v/w
  a = 0;
  [R2, V2, A2, R1, V1, A1] = solve(v, w, a, visual)
  pause(0.1); 
end
%end
%注意，需要限制旋转半径
%注意，w=0时需要分类
printf("over!\n")
