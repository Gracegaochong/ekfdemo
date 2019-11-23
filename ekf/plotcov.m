function plotcov(x,P)
pi=3.14159265;
  pxy=P(1:2,1:2);
  [ve,va]=eig(pxy);
  value= diag(va);
  if (value(1,1)>=value(2,1))
      big =1;
      small =2;
  end
  if(value(1,1)<value(2,1))
      big =2;
      small =1;
  end
  a=sqrt(value(big,1));
  b=sqrt(value(small,1));
  
  angle = atan2(ve(2,big),ve(1,big));%atan2(y,x)
  % 90-180  0-90
  % -90,-180   0,-90
  if(angle<0)
      angle=angle+2*pi;
  end
  R=[cos(angle) sin(angle);
      -sin(angle) cos(angle)];
  k=1;
  for i=0:0.1:2*pi
      x1 = a*cos(i);
      y1 = b*sin(i);
      fx=R*[x1;y1]+[x(1,1);x(2,1)];
      px(k,:) = fx';
      k=k+1;
  end
  hold on
  plot(x(1,1),x(2,1),'r+');
  plot(px(:,1)',px(:,2)','b');
  axis equal
  hold off
      
      
      