function drawSonarSensor(x,y,th,SL,SR,params)


    plot_arc(th+params.Robot.sonarStartAngle,th+pi/2,x,y,params.Robot.sonarRange);
    plot_arc(th-params.Robot.sonarStartAngle,th-pi/2,x,y,params.Robot.sonarRange);
%     alpha=(th+pi/2)-(th+params.Robot.sonarStartAngle);
%     x1=x+cos(alpha/2)*params.Robot.sonarRange/2;
%     y1=y+sin(alpha/2)*params.Robot.sonarRange/2;
%     text(x1,y1,num2str(SL));
%     alpha2=(th-pi/2)-(th-params.Robot.sonarStartAngle);
%     x2=x+cos(alpha2/2)*params.Robot.sonarRange/2;
%     y2=y+sin(alpha2/2)*params.Robot.sonarRange/2;
%     text(x2,y2,num2str(SR));

%   rectangle('Position',[x y w h], ...
%       'Curvature',[1 1],'FaceColor',color);
%   
%    
%    rectangle('Position',[x y-ymax w h], ...
%       'Curvature',[1 1],'FaceColor',color)
%   
%    rectangle('Position',[x y+ymax w h], ...
%       'Curvature',[1 1],'FaceColor',color)
%    
%    rectangle('Position',[x+xmax y w h], ...
%       'Curvature',[1 1],'FaceColor',color)
%    
%    rectangle('Position',[x+xmax y-ymax w h], ...
%       'Curvature',[1 1],'FaceColor',color)
%   
%    rectangle('Position',[x+xmax y+ymax w h], ...
%       'Curvature',[1 1],'FaceColor',color)
%    
%    rectangle('Position',[x-xmax y w h], ...
%       'Curvature',[1 1],'FaceColor',color)
%    
%    rectangle('Position',[x-xmax y-ymax w h], ...
%       'Curvature',[1 1],'FaceColor',color)
%   
%    rectangle('Position',[x-xmax y+ymax w h], ...
%       'Curvature',[1 1],'FaceColor',color)   
end