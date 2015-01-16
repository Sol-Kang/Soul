function Plot(nSensor, th0, lim, offset0_x, offset0_y, dataColor)

for i = 1:nSensor;
  if(th0(i) >= 0)
    a = 90-th0(i)-lim(i);
    b = 90-th0(i)+lim(i);
  else
    a = 90-th0(i)+lim(i);
    b = 90-th0(i)-lim(i);
  end
  t = linspace(a,b);
  h = offset0_x(i);
  k = offset0_y(i);
  x = 400*cosd(t) + h;
  y = 400*sind(t) + k;
  x = [x h x(1)];
  y = [y k y(1)];
  fill(x,y,dataColor(i,:),'FaceAlpha', 0.1,'EdgeColor','none','AlphaDataMapping','none');
end
end