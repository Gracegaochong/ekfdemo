function x= motion_model(x, u)

global DT;

F=[1,0,0,0;
    0,1,0,0;
    0,0,1,0;
    0,0,0,0];
B=[DT*cos(x(3,1)),0;
   DT*sin(x(3,1)),0;
   0,DT;
   1,0];
x=F*x+B*u;
