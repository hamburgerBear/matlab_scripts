clc;
clear;

function stat = calcCurvature(x, y)

  for i = 2:length(x)-1
    p1 = [x(i-1), y(i-1)];
    p2 = [x(i), y(i)];
    p3 = [x(i+1), y(i+1)];
    
    denominator = norm(p1-p2) * norm(p2-p3) * norm(p3-p1);
    stat(end+1) = (2.0 * ((p2(1) - p1(1)) * (p3(2) - p1(2)) - (p2(2) - p1(2)) * (p3(1) - p1(1))) / denominator);
  end
end

[x y] = textread('test_data.txt');

radius_limit = 0.8;
curvature_limit = 1.0 / radius_limit;
stat = calcCurvature(x, y);

figure(1)
hold on 
%因为不计算首尾两个点的曲率所以单独画首尾点
plot(x(1), y(1), 'b.')
plot(x(end), y(end), 'b.')

%画出曲线
for i = 1 : length(stat)
  if stat(i) > curvature_limit
    turning_point(end+1).x = x(i+1);
    turning_point(end).y = y(i+1);
  else
    common_point(end+1).x = x(i+1);
    common_point(end).y = y(i+1);
  end
end

plot([turning_point(:).x], [turning_point(:).y], 'ro')
plot([common_point(:).x], [common_point(:).y], 'b.')

