% 根据三点生成圆弧
% 参考：%https://blog.csdn.net/qq_31073871/article/details/109015969

clc;
clear;
close all;

function ret = sign(val)
  if val < 0.0 
    ret = -1.0;
  elseif val > 0.0
    ret = 1.0
  else 
    ret = 0.0
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

start =  [0.0,   0.0];
center = [5.0,  5.0];
goal =   [10.0, 0.0];
resolution = 0.05;

x1 = start(1);
y1 = start(2);
x2 = center(1);
y2 = center(2);
x3 = goal(1);
y3 = goal(2);

x1x1 = x1*x1;
y1y1 = y1*y1;
x2x2 = x2*x2;
y2y2 = y2*y2;
x3x3 = x3*x3;
y3y3 = y3*y3;
 
x2y3 = x2*y3;
x3y2 = x3*y2;

x2_x3 = x2-x3;
y2_y3 = y2-y3;

x1x1py1y1 = x1x1 + y1y1;
x2x2py2y2 = x2x2 + y2y2;
x3x3py3y3 = x3x3 + y3y3;
 
A = x1 * y2_y3 - y1 * x2_x3 + x2y3 - x3y2;
B = x1x1py1y1 * (-y2_y3) + x2x2py2y2 * (y1-y3) + x3x3py3y3 * (y2-y1);
C = x1x1py1y1 * x2_x3 + x2x2py2y2 * (x3 - x1) + x3x3py3y3 * (x1-x2);
D = x1x1py1y1 * (x3y2 - x2y3) + x2x2py2y2 * (x1*y3 - x3*y1) + x3x3py3y3 * (x2*y1-x1*y2);
 
% 圆心：(a, b) 半径：
a = -B/(2*A); 
b = -C/(2*A);
radius = sqrt((B*B+C*C-4*A*D)/(4*A*A));

theta1 = atan2(y1-b, x1-a);
theta2 = atan2(y2-b, x2-a);
theta3 = atan2(y3-b, x3-a);

%圆弧上的点到圆弧起点的角度差
diff1 = normalize_angle(theta2-theta1); 
%圆弧终点点到圆弧起点的角度差
diff2 = normalize_angle(theta3-theta1); 


%同向但角度不在范围区间 或者反向          
if ((sign(diff1) == sign(diff2)) && (abs(diff1) > abs(diff2))) || (sign(diff1) ~= sign(diff2))     
    if sign(diff2) > 0
      diff2 = diff2 - 2*pi;
    else 
      diff2 = diff2 + 2*pi;
    end
end 	

arc_len = 2 * abs(diff2) * radius;
point_num = floor(arc_len / resolution);
diff_step = diff2 / point_num;

figure(1)
hold on 

%圆弧的起点、圆弧上的点、圆弧的终点
plot(x1, y1, 'ro')
text(x1,y1,'start')
plot(x2, y2, 'ro')
text(x2,y2,'center')
plot(x3, y3, 'ro')
text(x3,y3,'goal')

%根据圆心和半径画圆
theta = 0:pi/20:2*pi; %角度[0,2*pi] 
x = a+radius*cos(theta);
y = b+radius*sin(theta);
plot(x,y,'-')

%画出圆弧的点根据分辨率间距
px = [];
py = [];
for i = 1:point_num
  px(end+1)=a+cos(diff_step*i+theta1)*radius;
  py(end+1)=b+sin(diff_step*i+theta1)*radius;
end
plot(px, py, 'r.')
