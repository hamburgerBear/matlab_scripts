% 圆角曲线
% 参考：https://github.com/antvis/X6/blob/346b3b0cc58c7e06b235bcaf4bde45ebd9b72d6a/packages/x6/src/registry/connector/rounded.ts

clc;
clear;
close all;

figure(1)
hold on 

sourcePoint = [0, 100];
targetPoint = [100, 0];
%控制点
routePoints = [0, 0]; 
%routePoints = [12.7, 36.4, 42.2, 36.4]; 
radius = 20;

points.x = sourcePoint(1);
points.y = sourcePoint(2);
text(sourcePoint(1), sourcePoint(2), 'start');
plot(sourcePoint(1), sourcePoint(2), 'ro');

for i = 1 : length(routePoints)/2
  points(end+1).x = routePoints((i-1)*2+1);
  points(end).y = routePoints((i-1)*2+2);
  text(routePoints((i-1)*2+1), routePoints((i-1)*2+2), 'control');
  plot(routePoints((i-1)*2+1), routePoints((i-1)*2+2), 'ro');
end
points(end+1).x = targetPoint(1);
points(end).y = targetPoint(2);
text(targetPoint(1), targetPoint(2), 'goal');
plot(targetPoint(1), targetPoint(2), 'ro');
f13 = 1 / 3;
f23 = 2 / 3;

curve_segment_start = sourcePoint;  
  
for i = 2 : length(points) - 1
  curr = [points(i).x, points(i).y];
  prev = [points(i-1).x, points(i-1).y];  
  next = [points(i+1).x, points(i+1).y];  
  prevDistance = sqrt((curr(1)-prev(1))*(curr(1)-prev(1))+(curr(2)-prev(2))*(curr(2)-prev(2)))/2;
  nextDistance = sqrt((curr(1)-next(1))*(curr(1)-next(1))+(curr(2)-next(2))*(curr(2)-next(2)))/2;
  startMove = min(radius, prevDistance);
  endMove = min(radius, nextDistance);

  angle = atan2(prev(2)-curr(2), prev(1)-curr(1));
  roundedStart = [curr(1) + cos(angle) * startMove, curr(2) + sin(angle) * startMove];
  angle = atan2(next(2)-curr(2), next(1)-curr(1));
  roundedEnd = [curr(1) + cos(angle) * endMove, curr(2) + sin(angle) * endMove];
  
  control1 = [f13 * roundedStart(1) + f23 * curr(1), f23 * curr(2) + f13 * roundedStart(2)];
  control2 = [f13 * roundedEnd(1) + f23 * curr(1), f23 * curr(2) + f13 * roundedEnd(2)];
    	
  % 画直线段
  plot ([curve_segment_start(1), roundedStart(1)], [curve_segment_start(2), roundedStart(2)]);
  % 画bezier曲线段
  p0 = roundedStart;
  p1 = control1;
  p2 = control2;
  p3 = roundedEnd;
  node = 0:0.05:1;
  bx = [];
  by = [];
  for t = 1:length(node) 
    bx(t) = p0(1)*((1-node(t))^3) + 3*p1(1)*node(t)*((1-node(t))^2) + 3*p2(1)*(node(t)^2)*(1-node(t)) + p3(1)*(node(t)^3);
    by(t) = p0(2)*((1-node(t))^3) + 3*p1(2)*node(t)*((1-node(t))^2) + 3*p2(2)*(node(t)^2)*(1-node(t)) + p3(2)*(node(t)^3);
  end
  plot(p0(1),p0(2), 'ro')
  %plot(p1(1),p1(2), 'go')
  %plot(p2(1),p2(2), 'bo')
  plot(p3(1),p3(2), 'bo')
  plot(bx,by, 'r.')
      
  curve_segment_start = roundedEnd;
end

  % 画直线段
  plot ([curve_segment_start(1), targetPoint(1)], [curve_segment_start(2), targetPoint(2)]);
  