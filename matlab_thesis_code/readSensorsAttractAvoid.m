function [SVALUES Env] = readSensorsAttractAvoid(Env,x,y,w,params,sensors)



SVALUES=zeros(2,6);

InRangeObs=cell(0,0);
obIdx=1;
nearestOb=[0 0];
touchedOb=[0 0];

for i = 1:length(Env.Obs)
   Env.Obs(i).nearest=false;
   Env.Obs(i).touched=false;
   Ob = Env.Obs(i);
   if Ob.type==1
       sensIdx=1;
   else
       sensIdx=2;
   end
   
   x2 = Ob.x;
   y2 = Ob.y;
   
   if Env.wrapped
       % Find shortest x distance on torus
       if abs(x2+Env.xmax-x) < abs(x2-x)
          x2 = x2+Env.xmax;
       elseif abs(x2-Env.xmax-x) < abs(x2-x)
          x2 = x2-Env.xmax;
       end

       % Find shortest y distance on torus
       if abs(y2+Env.ymax-y) < abs(y2-y)
          y2 = y2+Env.ymax;
       elseif abs(y2-Env.ymax-y) < abs(y2-y)
          y2 = y2-Env.ymax;
       end
   end
   
   dx = x2-x;
   dy = y2-y;
   z = sqrt(dx^2+dy^2); % distance from robot to object centre
   objectBorder = z-Ob.radius;
   
    if objectBorder < params.Robot.sonarRange   % object is close

        
        
      v = atan2(dy,dx); %+ times*pi; % bearing of object wrt robot
      if ( v<0)
          v = v+2*pi;
      end
      
      dw = v-w; % angular difference between robot's heading and object
      % normalise angle
      if(dw > pi)
        dw = dw - 2*pi ;
      elseif(dw < -pi)
        dw = 2*pi + dw; 
      end
      
      %%DEBUG
%       if params.logtrace
%       disp(['Object is in range. Th Robot:' num2str(w*(180/pi)) ... 
%           ' Angle of object:' num2str(v*(180/pi)) ' angle difference:' num2str(dw*(180/pi)) ...
%           ' dist to obj:' num2str(objectBorder) ]);
%       end
      
      % Stimulus strength. (depends on distance to object boundary), the
      % closer the stronger
      % IF NEGATIVE WILL BE IGNORED, SINCE OUT OF RANGE
      S = (params.Robot.sonarRange-(objectBorder))/params.Robot.sonarRange;
      
         if (dw > params.Robot.sonarStartAngle && dw < pi/2)
             %%DEBUG
             % centre of object is to left
             if ( z > Ob.radius )
                SVALUES(sensIdx,sensors.SL) = max(S,SVALUES(sensIdx,sensors.SL)); % use nearest object
             else
                 SVALUES(sensIdx,sensors.TL) = max(S,SVALUES(sensIdx,sensors.TL));
             end
         elseif (dw < -params.Robot.sonarStartAngle && dw > -pi/2)
             %%DEBUG
             % centre of object is to right
             if ( z > Ob.radius )
                SVALUES(sensIdx,sensors.SR) = max(S,SVALUES(sensIdx,sensors.SR));
             else 
                  SVALUES(sensIdx,sensors.TR) = max(S,SVALUES(sensIdx,sensors.TR));
             end
         elseif (dw >= - params.Robot.sonarStartAngle && dw <= params.Robot.sonarStartAngle)
             %DEBUG
             % centre of object is ahead
             if ( z > Ob.radius )
                SVALUES(sensIdx,sensors.SC) = max(S,SVALUES(sensIdx,sensors.SC));
             else
                  SVALUES(sensIdx,sensors.TC) = max(S,SVALUES(sensIdx,sensors.TC));
             end
         end
         
   
         if ismember(S,SVALUES(sensIdx,:))
             nearestOb(sensIdx)=i;
             Env.Obs(i).dw=dw;
             Env.Obs(i).objectBorder=objectBorder;
             InRangeObs{obIdx}=Ob;
             obIdx=obIdx+1;
         end
         if ismember(S,[SVALUES(sensIdx,sensors.TL), SVALUES(sensIdx,sensors.TR), SVALUES(sensIdx,sensors.TC)])
             touchedOb(sensIdx)=i;
             InRangeObs{obIdx}=Ob;
             obIdx=obIdx+1;
         end
    end
      
      
end

if nearestOb(1)
    Env.Obs(nearestOb(1)).nearest=true;
end
if nearestOb(2)
    Env.Obs(nearestOb(2)).nearest=true;
end
if touchedOb(1)
    Env.Obs(touchedOb(1)).touched=true;
end
if touchedOb(2)
    Env.Obs(touchedOb(2)).touched=true;
end    




%% add noise to the sensor readings? noiseAvg=0, noiseStDev=0.1 
% SL=SL+noiseAvg+noiseStDev*randn;
% SR=SR+noiseAvg+noiseStDev*randn;
