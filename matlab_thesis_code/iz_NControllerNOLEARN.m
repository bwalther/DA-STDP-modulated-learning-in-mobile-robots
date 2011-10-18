classdef iz_NControllerNOLEARN<handle
    
    properties(GetAccess= 'protected', SetAccess= 'private')

        % network
        M=200;                 % number of synapses per neuron
        D=1;                   % maximal conduction delay
        Ns1=50; Ns2=50;
        % there will be different allocation in case of attractor avoidance
        Ns3=100; Ns4=100;       % right green and blue
        %
        Ne1=400; Ne2=400;
        Ni1=100; Ni2=100;
        Ns=0;Ne=0;Ni=0;N=0;

        % indices of layers
        SL=[]; SR=[]; ML=[]; MR=[]; IL=[]; IR=[];
        % for combined attract avoid (green and blue resources)
        GSL=[]; BSL=[]; GSR=[]; BSR=[];

        % connectivity variables
        delays=[]; post=[];
        s=[];
        
        % voltage, refractory variable, neuron firings (reset after each
        % step)
        v = [];  u = []; firings=[];
        % iz parameters
        a=[];  b=[]; c=[]; d=[];
        
        % configuration param
        combinedAttractAvoid=false;
        
        % stats
        lastfirings=[];
        HZPERGRP=[];        
    end
    methods (Access='private')
        
        function buildProperties(obj,attrAvoid)
            if attrAvoid
                obj.Ns1=100; obj.Ns2=100;       % left green and blue
                obj.Ns3=100; obj.Ns4=100;       % right green and blue
                obj.Ne1=400; obj.Ne2=400;
                obj.Ni1=100; obj.Ni2=100;
                obj.Ns=obj.Ns1+obj.Ns2+obj.Ns3+obj.Ns4;
            else
                obj.Ns=obj.Ns1+obj.Ns2;
            end
            obj.Ne=obj.Ne1+obj.Ne2;
            obj.Ni=obj.Ni1+obj.Ni2;
            obj.N=obj.Ns+obj.Ne+obj.Ni;
            
            % config for heterogenity; goal: stability, no overtraining (too much spikings in
            % Motor layer after learning,  WTA
            re=rand(obj.Ns+obj.Ne,1); ri=rand(obj.Ni,1);
            obj.a=[0.02*ones(obj.Ns,1); 0.2*ones(obj.Ne,1); 1*ones(obj.Ni,1) ]; % 0.001 (for iz Grp)
            obj.b=[0.2*ones(obj.Ns,1); 0.2*ones(obj.Ne,1);  0.25-0.05*ri ]; % 0.25-0.05*ri, 0.237*
            obj.c=[-65+15*re.^2; -65*ones(obj.Ni,1)]; % -65+15*re.^2
            obj.d=[   8*ones(obj.Ns,1);   8*ones(obj.Ne,1);   2*ones(obj.Ni,1) ];
            
            % indices of layers
            if obj.combinedAttractAvoid
                obj.GSL=1:obj.Ns1;
                obj.BSL=obj.Ns1+1:obj.Ns1+obj.Ns2;
                obj.SL=1:obj.Ns1+obj.Ns2;
                obj.GSR=obj.Ns1+obj.Ns2+1:obj.Ns1+obj.Ns2+obj.Ns3;
                obj.BSR=obj.Ns1+obj.Ns2+obj.Ns3+1:obj.Ns;
                obj.SR=obj.Ns1+obj.Ns2+1:obj.Ns;
            else
                obj.SL=1:obj.Ns1;
                obj.SR=obj.Ns1+1:obj.Ns;
            end
            obj.ML=obj.Ns+1:obj.Ns+obj.Ne1;
            obj.MR=obj.Ns+obj.Ne1+1:obj.Ns+obj.Ne;
            obj.IL=obj.Ns+obj.Ne+1:obj.Ns+obj.Ne+obj.Ni1;
            obj.IR=obj.Ns+obj.Ne+obj.Ni1+1:obj.N;
            
            sex=rand(obj.Ne,obj.M).*(rand(obj.Ne,obj.M)>0.5);
            obj.s=[2*rand(obj.Ns,obj.M);rand(obj.Ne,obj.M);-2*rand(obj.Ni,obj.M)];
            obj.v = -65*ones(obj.N,1);                   
            obj.u = 0.2.*obj.v;         
            obj.firings=[-obj.D 0];
 
        end
        
        function initConnectivity(obj,connectBehaviour)
            if connectBehaviour==CONNECTBEHAVIOUR.ATTRACTAVOID
                sensorCons=[randi([obj.MR(1), obj.MR(end)],[length(obj.GSL), obj.M]); % GSL->MR
                    randi([obj.ML(1), obj.ML(end)],[length(obj.BSL), obj.M]); % BSL->ML
                    randi([obj.ML(1), obj.ML(end)],[length(obj.GSR), obj.M]); % GSR->ML
                    randi([obj.MR(1), obj.MR(end)],[length(obj.BSR), obj.M])]; % BSR->MR
            elseif connectBehaviour==CONNECTBEHAVIOUR.RANDOM
                sensorCons=[randi([obj.ML(1), obj.MR(end)],[obj.Ns, obj.M])]; % S*->M*
            elseif connectBehaviour==CONNECTBEHAVIOUR.ATTRACT
                sensorCons=[randi([obj.MR(1), obj.MR(end)],[length(obj.SL), obj.M]); % SL->MR
                    randi([obj.ML(1), obj.ML(end)],[length(obj.SR), obj.M])]; % SR->ML
                
            elseif connectBehaviour==CONNECTBEHAVIOUR.AVOID
                sensorCons=[randi([obj.ML(1), obj.ML(end)],[length(obj.SL), obj.M]); % SL->ML
                    randi([obj.MR(1), obj.MR(end)],[length(obj.SR), obj.M])]; % SR->MR
                
            else
                error('connect behaviour not recognised');
            end
            mRec= ceil(0.2/obj.M); % recurrent proecting to own
            mRec=70; % alternatively fixed value to have some drift
            %mRec=0; % no recurrent connections
            obj.post=[sensorCons;
                randi([obj.ML(1), obj.ML(end)],[length(obj.ML), mRec]), randi([obj.IR(1), obj.IR(end)],[length(obj.ML), obj.M-mRec]);    % ML->ML (for some self sustained activity) & ML->IR (WTA)
                randi([obj.MR(1), obj.MR(end)],[length(obj.MR), mRec]), randi([obj.IL(1), obj.IL(end)],[length(obj.MR), obj.M-mRec]); % MR->MR (") & MR->IL (WTA)
                
                randi([obj.ML(1), obj.ML(end)],[length(obj.IL), obj.M/2]), randi([obj.IR(1), obj.IR(end)],[length(obj.IL), obj.M/2]) ;% IL ->ML & IL -> IR (WTA)
                randi([obj.MR(1), obj.MR(end)],[length(obj.IR), obj.M/2]), randi([obj.IL(1), obj.IL(end)],[length(obj.IR), obj.M/2]) ];% IR ->MR & IR -> IL (WTA)
            for i=1:obj.N
                if i<=obj.Ns+obj.Ne
                    for j=1:obj.D
                        obj.delays{i,j}=obj.M/obj.D*(j-1)+(1:obj.M/obj.D);
                    end;
                else
                    obj.delays{i,1}=1:obj.M;
                end;
            end;
            
        end
               

    end
    
    methods

        function obj=iz_NControllerNOLEARN(params)
            obj.combinedAttractAvoid=params.attrAvoid;
            buildProperties(obj,params.attrAvoid);
            initConnectivity(obj,params.connectBehaviour);
        end
        
        function data=topologyInfo(obj)
            if obj.combinedAttractAvoid
                data.GSL=obj.GSL;
                data.BSL=obj.BSL;
                data.GSR=obj.GSR;
                data.BSR=obj.BSR;
            else
                data.SL=obj.SL;
                data.SR=obj.SR;
            end
            data.ML=obj.ML;
            data.MR=obj.MR;
            data.IL=obj.IL;
            data.IR=obj.IR;
            data.N=obj.N;
        end
        function data=serializePlotData(obj)
                data.HZPERGRP=obj.HZPERGRP;
                data.lastfirings=obj.lastfirings;
        end
        function [network]=serializeNetwork(obj)
            network.M=obj.M; 
            network.D=obj.D;
            network.Ns1=obj.Ns1; network.Ns2=obj.Ns2;
            network.Ne1=obj.Ne1; network.Ne2=obj.Ne2;
            network.Ni1=obj.Ni1; network.Ni2=obj.Ni2;
            
            network.a=obj.a;
            network.b=obj.b;
            network.c=obj.c;
            network.d=obj.d;
            
            network.s=obj.s;
            
            % connectivity
            network.post=obj.post;
            network.delays=obj.delays;

        end
        function [retHZPERGROUP lastfirings]=simulationStep(obj,dsec,IExt)

            for t=1:100
                I=13*(rand(obj.N,1)-0.5)+IExt;
        
                fired = find(obj.v>=30);                % indices of fired neurons
                obj.v(fired)=obj.c(fired);
                obj.u(fired)=obj.u(fired)+obj.d(fired);


                obj.firings=[obj.firings;t*ones(length(fired),1),fired];
                k=size(obj.firings,1);
                while obj.firings(k,1)>t-obj.D
                    del=obj.delays{obj.firings(k,2),t-obj.firings(k,1)+1};
                    ind = obj.post(obj.firings(k,2),del);
                    I(ind)=I(ind)+obj.s(obj.firings(k,2), del)';
                    k=k-1;
                end;
                
                dt=0.25;
                k=1/dt;
                stepfired=[];
                for ei=1:k
                    obj.v=obj.v+dt*((0.04*obj.v+5).*obj.v+140-obj.u+I);    
                    obj.u=obj.u+dt*(obj.a.*(obj.b.*obj.v-obj.u));            
                    stepfired = [stepfired;find(obj.v>=30)];                
                    obj.v(stepfired)=31;
                end
                
            end;
            
            CURFIRINGS=sort(obj.firings(:,2));
            if obj.combinedAttractAvoid
                % -1 since we add this in the beginning of the iteration
                obj.HZPERGRP(dsec,1)=(sum(CURFIRINGS<(obj.GSL(end)+1))-1)/obj.Ns1*10;
                obj.HZPERGRP(dsec,2)=sum(CURFIRINGS>=obj.BSL(1) & CURFIRINGS<=obj.BSL(end))/obj.Ns2*10;
                obj.HZPERGRP(dsec,3)=sum(CURFIRINGS>=obj.GSR(1) & CURFIRINGS<=obj.GSR(end))/obj.Ns3*10;
                obj.HZPERGRP(dsec,4)=sum(CURFIRINGS>=obj.BSR(1) & CURFIRINGS<=obj.BSR(end))/obj.Ns4*10;
                obj.HZPERGRP(dsec,5)=sum(CURFIRINGS>=obj.ML(1) & CURFIRINGS<=obj.ML(end))/obj.Ne1*10;
                obj.HZPERGRP(dsec,6)=sum(CURFIRINGS>=obj.MR(1) & CURFIRINGS<=obj.MR(end))/obj.Ne2*10;
                obj.HZPERGRP(dsec,7)=sum(CURFIRINGS>=obj.IL(1) & CURFIRINGS<=obj.IL(end))/obj.Ni1*10;
                obj.HZPERGRP(dsec,8)=sum(CURFIRINGS>=obj.IR(1) & CURFIRINGS<=obj.IR(end))/obj.Ni2*10;
            else
                % -1 since we add this in the beginning of the iteration
                obj.HZPERGRP(dsec,1)=(sum(CURFIRINGS<(obj.SL(end)+1))-1)/obj.Ns1*10;
                obj.HZPERGRP(dsec,2)=sum(CURFIRINGS>obj.SL(end) & CURFIRINGS<obj.SR(end)+1)/obj.Ns2*10;
                obj.HZPERGRP(dsec,3)=sum(CURFIRINGS>obj.SR(end) & CURFIRINGS<obj.ML(end)+1)/obj.Ne1*10;
                obj.HZPERGRP(dsec,4)=sum(CURFIRINGS>obj.ML(end) & CURFIRINGS<obj.MR(end)+1)/obj.Ne2*10;
                obj.HZPERGRP(dsec,5)=sum(CURFIRINGS>obj.MR(end) & CURFIRINGS<obj.IL(end)+1)/obj.Ni1*10;
                obj.HZPERGRP(dsec,6)=sum(CURFIRINGS>obj.IL(end) & CURFIRINGS<obj.IR(end)+1)/obj.Ni2*10;
            end
            retHZPERGROUP=obj.HZPERGRP;
            
            ind = find(obj.firings(:,1) > 101-obj.D);
            lastfirings=obj.firings;
            obj.lastfirings=obj.firings;
            obj.firings=[-obj.D 0;obj.firings(ind,1)-100,obj.firings(ind,2)];
            
            
            
        end
        
        
    end
    
    
end