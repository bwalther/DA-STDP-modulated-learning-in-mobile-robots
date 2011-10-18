function drawObstacle(wrapped, Ob,xmax,ymax)

x = Ob.x-Ob.radius;
y = Ob.y-Ob.radius;
w = Ob.radius*2;
h = Ob.radius*2;
lw=0.5;

if Ob.touched
   color='r'; 
elseif Ob.type==1
    color='g';
else
    color='b';
end

if Ob.nearest
    lw=1;
end

rectangle('Position',[x y w h], ...
    'Curvature',[1 1],'FaceColor',color,'LineWidth',lw);

if wrapped
    % Draw nine circles - to account for wrapping on torus
    rectangle('Position',[x y-ymax w h], ...
        'Curvature',[1 1],'FaceColor',color,'LineWidth',lw)
    
    rectangle('Position',[x y+ymax w h], ...
        'Curvature',[1 1],'FaceColor',color,'LineWidth',lw)
    
    rectangle('Position',[x+xmax y w h], ...
        'Curvature',[1 1],'FaceColor',color,'LineWidth',lw)
    
    rectangle('Position',[x+xmax y-ymax w h], ...
        'Curvature',[1 1],'FaceColor',color,'LineWidth',lw)
    
    rectangle('Position',[x+xmax y+ymax w h], ...
        'Curvature',[1 1],'FaceColor',color,'LineWidth',lw)
    
    rectangle('Position',[x-xmax y w h], ...
        'Curvature',[1 1],'FaceColor',color,'LineWidth',lw)
    
    rectangle('Position',[x-xmax y-ymax w h], ...
        'Curvature',[1 1],'FaceColor',color,'LineWidth',lw)
    
    rectangle('Position',[x-xmax y+ymax w h], ...
        'Curvature',[1 1],'FaceColor',color,'LineWidth',lw)
    
end
end
