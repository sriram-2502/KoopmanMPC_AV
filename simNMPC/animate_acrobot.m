function animate_acrobot(t,x,x_pred,params)

if(nargin<3)
    x_pred = [];
    params = [];
    Np = 0;
else
    Np = params.predHorizon;
end

l1 = 1 ; l2 = 1;

N = length(t);

th1 = x(1,1);th2 = x(1,2);
x1 = l1*sin(th1); y1 = -l1*cos(th1);
x2 = x1 + l2*sin(th1+th2); y2 = y1 - l2*cos(th1+th2);

link1 = line([0 x1],[0 y1],'color','k','LineWidth',4);
link2 = line([x1 x2],[y1 y2],'color','r','LineWidth',4);

% plot predictions
if(Np)
    th1_pred = x_pred(1,1); th2_prd = x(1,2);
    x1_pred = l1*sin(th1_pred); y1_pred = -l1*cos(th1_pred);
    x2_pred = x1_pred + l2*sin(th1_pred+th2_prd);
    y2_pred = y1_pred - l2*cos(th1_pred+th2_prd);

    c1 = [150 150 150]./255;
    c2 = [255 150 150]./255;
    
    link1_pred = line([0 x1_pred],[0 y1_pred],'color',c1,'LineWidth',4);
    link2_pred = line([x1_pred x2_pred],[y1_pred y2_pred ],'color',c2,'LineWidth',4);
end

T = t;
str = strcat(num2str(T)+"/",num2str(T(end)));
timestamp = text(1.6,-0.1,str,'HorizontalAlignment','right');
axis([-3 3 -3 3])

for i = 1:N
    
th1 = x(i,1);th2 = x(i,2);
x1 = l1*sin(th1);y1 = -l1*cos(th1);
x2 = x1 + l2*sin(th1+th2);y2 = y1 - l2*cos(th1+th2);

set(link1,'xdata',[0 x1],'ydata',[0 y1]);
set(link2,'xdata',[x1 x2],'ydata',[y1 y2]);

if(Np)
    k=i;
    th1_pred = x_pred((k-1)*Np+1,1);th2_pred  = x_pred((k-1)*Np+1,2);
    x1_pred = l1*sin(th1_pred);y1_pred = -l1*cos(th1_pred);
    x2_pred = x1_pred + l2*sin(th1_pred+th2_pred);
    y2_pred = y1_pred - l2*cos(th1_pred+th2_pred);
    
    set(link1_pred ,'xdata',[0 x1_pred ],'ydata',[0 y1_pred ]);
    set(link2_pred ,'xdata',[x1_pred  x2_pred ],'ydata',[y1_pred  y2_pred]);
    pause(0.02)
    drawnow
end

T = t(i);
str = strcat(num2str(T)+"/",num2str(t(end)));
set(timestamp,'String',str);

pause(0.05);
drawnow
end