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

function [R2, V2, A2, R1, V1, A1] = solve(v, w, A, visual)
  %机器人参数
  D = 1.0;
  weight = 0.6;
  lenght = 1.2;
  wheel_rotate_max = 1.91 %110度
  wheel_rotate_min = -1.91 %110度
  R = v / w;
	vx = v*cos(A);
  vy = v*sin(A);
  if abs(w)>0.01
    x1 = -vy/w;%瞬心
    y1 = vx/w;%瞬心
    A1 = atan2(-y1,-D/2-x1) + pi/2;
    if A1>pi
      A1 = A1-2*pi;
    elseif A1<-pi
      A1 = A1+2*pi;
    else
      A1 = A1;
    end
    index = 1;
    if(A1 > wheel_rotate_max)
      A1 = A1-pi
      index = index * -1;
    end
    if(A1< wheel_rotate_min)
      A1 = A1+pi
      index = index * -1;
    end
    V1 = hypot(-y1,-D/2-x1)*w*index;
    R1 = V1/w;
    
    A2 = atan2(-y1,D/2-x1) + pi/2;
    if A2>pi
      A2 = A2-2*pi;
    elseif A2<-pi
      A2 = A2+2*pi;
    else 
      A2 = A2;
    end
    index = 1;
    if(A2 > wheel_rotate_max)
      A2 = A2-pi;
      index = index * -1;
    end
    if(A2< wheel_rotate_min)
      A2 = A2+pi;
      index = index * -1;
    end
    V2 = hypot(-y1,D/2-x1)*w*index;
    R2 = V2/w;
  else
    A1 = atan2(vy,vx);
    A2 = atan2(vy,vx);
    index = 1;
    if(A1 > wheel_rotate_max)
      A1 = A1-pi;
      index = index * -1;
    end
    if(A1< wheel_rotate_min)
      A1 = A1+pi;
      index = index * -1;
    end
    
    index2 = 1;
    if(A2 > wheel_rotate_max)
      A2 = A2-pi;
      index2 = index2 * -1;
    end
    if(A2< wheel_rotate_min)
      A2 = A2+pi;
      index2 = index2 * -1;
    end
    V1 = hypot(vy,vx)*index;
    V2 = hypot(vy,vx)*index2;
    R1 = V1/w;
    R2 = V2/w;
  end
  
  %R = v / w;
	% R2前R1后
	%R2_square = power([D/2+R*cos(pi/2-A)], 2) + power([R*sin(pi/2-A)], 2);
	%R2 = sqrt(R2_square);
	%考虑方向
	%R2 = sign(R) * R2;
	%V2 = R2 * w;
	% check this
	%tmp_theta2 = atan([R*sin(pi/2-A)]/[D/2+R*cos(pi/2-A)]);
	%A2 = pi/2 - tmp_theta2;
	%考虑方向 check this 
	%A2 = A2 * sign(R) * 1.0;
	%R1_square = power([D/2-R*cos(pi/2-A)], 2) + power([R*sin(pi/2-A)], 2);
	%R1 = sqrt(R1_square);
	%考虑方向
	%R1 = sign(R) * R1;
	%V1 = R1 * w;
	% check this
	%tmp_theta1 = atan([R*sin(pi/2-A)]/[D/2-R*cos(pi/2-A)]);
	%A1 = pi/2 - tmp_theta1;
	%考虑方向 check this 
	%A1 = A1 * sign(R) * -1.0;
  
  %可视化
  if visual
    figure(1)
    
    plot ([-lenght, lenght],[weight, weight], 'g')
    hold on 
    plot ([-lenght, lenght],[-weight, -weight], 'g')
    plot ([lenght, lenght],[-weight, weight], 'g')
    plot ([-lenght, -lenght],[-weight, weight], 'g')
    plot ([-lenght, lenght],[0, 0], 'b--')
    quiver(0,0,cos(A)*v,sin(A)*v) %车体速度向量
    quiver(D/2,0,cos(A2)*V2,sin(A2)*V2) %车前舵轮速度向量
    quiver(-D/2,0,cos(A1)*V1,sin(A1)*V1) %车后舵轮速度向量
    rotate_center = [0+cos(A+pi/2)*R, 0+sin(A+pi/2)*R]; %旋转中心计算
    car_center = [0, 0];
    front_steer = [D/2, 0];
    rear_steer = [-D/2, 0];
    plot(rotate_center(1), rotate_center(2), 'bo')
    plot ([front_steer(1), rotate_center(1)],[front_steer(2), rotate_center(2)], 'b')
    plot ([rear_steer(1), rotate_center(1)],[rear_steer(2), rotate_center(2)], 'b')
    plot ([car_center(1), rotate_center(1)],[car_center(2), rotate_center(2)], 'b')
    hold off
  end 
end 

%使用方法
%visual = 1;
%[R2, V2, A2, R1, V1, A1] = solve(0.3, 0.3, pi, visual)
%for i = -pi:0.017453:pi
for i = -1:0.005:1
  visual = 1;
  [R2, V2, A2, R1, V1, A1] = solve(0.3, i, 0, visual)
end