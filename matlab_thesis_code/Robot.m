classdef Robot<handle
    
 
    properties(GetAccess= 'public', SetAccess= 'public')
        Rmax; % peak motor firing rate in Hz
        Umin; % minimum wheel velocity in cm per ms
        Umax; % maximum wheel velocity in cm per ms
    end
    properties(GetAccess= 'protected', SetAccess= 'private')
        posAbs;     % Position in absolute coordinates
                    % Format: vector of doubles, [x y]
        velAbs;     % Velocity in absolute coordinates
                    % Format: vector of doubles, [x y]
        thAbs;      % Yaw angle relative to positive x-axis
                    % Format: double (-pi < thAbs <= pi)
        wAbs;       % Angular velocity
                    % Format: double (positive counter-clockwise)
                    
        dt; % robot timestep
        FI; % Scaling Factor for Incoming current
    end
    properties(GetAccess= 'public', SetAccess= 'protected')
        shape=ROBOTSHAPE.NONE; 
        radius=2;
        % trajectory trace co-ordinates
        xTraj=[];
        yTraj=[];
        wTraj=[]; % bearing 0 = East, pi/2 = North
        worldTraj=cell(1,1); % dynamic world
    end
    methods
        function obj=Robot(params)
            obj.posAbs=[params.xstart params.ystart];
            obj.velAbs= [0 0]; % Assume robot is stationary to start
            obj.thAbs= params.thstart; % initial orientation in radians
            obj.wAbs= 0;
            obj.xTraj(1)=params.xstart;
            obj.yTraj(1)=params.ystart;
            obj.wTraj(1) = params.thstart; % initial orientation in radians
            obj.dt = params.dt;
            obj.Rmax = params.Rmax;
            obj.Umin = params.Umin;
            obj.Umax = obj.Umin+obj.Umin/3;
            obj.FI=params.FI;
        end
        function resetTraj(obj)
            obj.xTraj=[obj.posAbs(1)];
            obj.yTraj=[obj.posAbs(2)];
            obj.wTraj=[obj.thAbs]; % bearing 0 = East, pi/2 = North
        end
        
    % State Manipulator Functions
        function state= getState(obj)
        % state = getState(obj)
        % Extracts current state properties for the simulation program
        %
        % Input:
        % obj - Instance of class CreateRobot
        %
        % Output:
        % state - Vector of doubles [x y th v w], values of current state
            
            % Extract variables
            x= obj.posAbs(1);
            y= obj.posAbs(2);
            th= obj.thAbs;
            v= obj.velAbs;
            w= obj.wAbs;
            
            % Put in output format
            state= [x y th v w];
        end
        
        function initWorld(obj,Env)
            obj.worldTraj{1}=Env;
        end
        
        function setStateSimple(obj,pAbs,tAbs)
        % setState(obj,state)
        % Imports new state properties from the simulation program
        %
        % Input:
        % obj - Instance of class CreateRobot
        % state - Vector of doubles [x y th v w], values of new state
            
            % Update robot object
            obj.posAbs= pAbs;
            obj.thAbs= tAbs;
            obj.xTraj(end)=pAbs(1);
            obj.yTraj(end)=pAbs(2);
            obj.wTraj(end)=tAbs;
        end
        
        function setState(obj,state)
        % setState(obj,state)
        % Imports new state properties from the simulation program
        %
        % Input:
        % obj - Instance of class CreateRobot
        % state - Vector of doubles [x y th v w], values of new state
            
            % Update robot object
            obj.posAbs= state(1:2);
            obj.thAbs= state(3);
            obj.velAbs= state(4:5);
            obj.wAbs= state(6);
        end
        
       
        
        function updatePosition(obj,IL,IR,wrapped,xmax,ymax,Env)
            x1= obj.posAbs(1);
            y1= obj.posAbs(2);
            w1= obj.thAbs;
%             % scaling to enable sharper turns
%             diff=IL-IR;
%             IL=IL+diff/2;
%             IR=IR-diff/2;
            UL=obj.Umin+(min(1,IL/obj.Rmax))*(obj.Umax-obj.Umin);
            UR=obj.Umin+(min(1,IR/obj.Rmax))*(obj.Umax-obj.Umin);
            BL=min(max(obj.Umin,UL),obj.Umax);
            BR=min(max(obj.Umin,UR),obj.Umax);
%             UL = (obj.Umin/obj.Umax+IL/obj.Rmax*(1-obj.Umin/obj.Umax)); % velocity of left wheel
%             UR = (obj.Umin/obj.Umax+IR/obj.Rmax*(1-obj.Umin/obj.Umax)); % velocity of left wheel
%             
%             BL = UL*obj.Umax; % distance moved by left wheel
%             BR = UR*obj.Umax; % distance moved by right wheel
            
            A = 1; % axle length
            
            B = (BL+BR)/2; % distance moved by centre of axle
            C = BR-BL;
            
            dx = B*cos(w1); % change in x-co-ordinate
            dy = B*sin(w1); % change in y co-ordinate
            dw = atan2(C,A); % change in orientation
            
            x2 = x1+obj.dt*dx;
            y2 = y1+obj.dt*dy;
            w2 = w1+obj.dt*dw;
            
            % Wrap x and y axes - robot moves on a torus
            if wrapped
                if x2 > xmax
                    x2 = x2-xmax;
                end
                if y2 > ymax
                    y2 = y2-ymax;
                end
                if x2 < 0
                    x2 = xmax+x2;
                end
                if y2 < 0
                    y2 = ymax+y2;
                end
            elseif xmax
                bump=false;
                if x2 > xmax
                    bump=true;
                    x2 = x2-x2-xmax;
                end
                if y2 > ymax
                    bump=true;
                    y2 = y2-y2-ymax;
                end
                if x2 < 0
                    bump=true;
                    x2 = abs(x2);
                end
                if y2 < 0
                    bump=true;
                    y2 = abs(y2);
                end
                if bump
                    if rand > 0.5
                        w2=w2+pi/2;
                    else
                        w2=w2-pi/2;
                    end
                end
            end
            
            % Keep orientation in range 0 to 2pi
            w2 = phmod(w2);
            if w2 < 0
                w2 = 2*pi+w2;
            end
                        % DEBUG
            % disp(['x2:' num2str(x2) ' y2:' num2str(y2) 'th:' num2str(w2) ' UL:' num2str(UL) ' UR:' num2str(UR)]);
            obj.setState([x2 y2 w2 obj.velAbs obj.wAbs]);
            
            obj.xTraj(end+1)=obj.posAbs(1);
            obj.yTraj(end+1)=obj.posAbs(2);
            obj.wTraj(end+1)=obj.thAbs;
            
            if ~nargin > 6
                obj.worldTraj{end+1}=Env;
            end
        end
    end
    
end
    
