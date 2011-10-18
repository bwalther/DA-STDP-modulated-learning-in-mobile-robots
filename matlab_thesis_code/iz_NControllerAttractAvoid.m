classdef iz_NControllerAttractAvoid<handle
    
    properties(GetAccess= 'protected', SetAccess= 'private')
        
        % figures
        f1Contr=0;guiOn=false;
        firingHist=zeros(1,2);
        firingHistTscale=0;
        logtrace=false;
        dsec=1;
        ads=false;stdpsym=false;stdpvals=[];STDPwrapover=2;
        timerSim;
        
        M=200;                 % number of synapses per neuron
        D=1;                   % maximal conduction delay
        Ns1=100; Ns2=100;       % left green and blue
        Ns3=100; Ns4=100;       % right green and blue
        Ne1=400; Ne2=400;
        Ni1=100; Ni2=100;
        Ns=0;Ne=0;Ni=0;N=0;
        
        % indices of layers
        GSL=[]; BSL=[]; SL=[]; GSR=[]; BSR=[]; SR=[]; ML=[]; MR=[]; IL=[]; IR=[];

        % connectivity variables
        delays=[]; post=[]; pre=[]; aux=[];
        % connection indices 
        cGSLML=[]; cGSLMR=[]; cGSRML=[]; cGSRMR =[];
        cBSLML=[]; cBSLMR=[]; cBSRML=[]; cBSRMR =[];
        
         % maximal synaptic strength % synaptic weights % their derivatives
        sm=4;  s=[]; sd=[]; 
        % sum of weights sensor to motor layer, used for synaptic scaling:
        smTM=0;
        
        % stdp traces for each synapse % level of dopamine above the baseline
        STDP = []; DA=0;
        % dopamine increases for reward and punishments
        attrDA=0.5;punDA=-0.2;
        
        % voltage, refractory variable, neuron firings (reset after each
        % step
        v = [];  u = []; firings=[];
        % iz parameters
        a=[];  b=[]; c=[]; d=[];
        
        % stats
        weightMean=[];
        HZPERGRP=[];  
        lastfirings=[];
        daStats=[];
    end
    methods (Access='private')
        
        function buildTopology(obj)
            % sizes
            obj.Ns=obj.Ns1+obj.Ns2+obj.Ns3+obj.Ns4;
            obj.Ne=obj.Ne1+obj.Ne2;
            obj.Ni=obj.Ni1+obj.Ni2;
            obj.N=obj.Ns+obj.Ne+obj.Ni;
            
            % indices of layers
            obj.GSL=1:obj.Ns1;
            obj.BSL=obj.Ns1+1:obj.Ns1+obj.Ns2;
            obj.SL=1:obj.Ns1+obj.Ns2;
            
            obj.GSR=obj.Ns1+obj.Ns2+1:obj.Ns1+obj.Ns2+obj.Ns3;
            obj.BSR=obj.Ns1+obj.Ns2+obj.Ns3+1:obj.Ns;
            obj.SR=obj.Ns1+obj.Ns2+1:obj.Ns;
            
            obj.ML=obj.Ns+1:obj.Ns+obj.Ne1;
            obj.MR=obj.Ns+obj.Ne1+1:obj.Ns+obj.Ne;
            obj.IL=obj.Ns+obj.Ne+1:obj.Ns+obj.Ne+obj.Ni1;
            obj.IR=obj.Ns+obj.Ne+obj.Ni1+1:obj.N;
        end
        
        function buildProperties(obj)
            obj.buildTopology;
            
            re=rand(obj.Ns+obj.Ne,1); ri=rand(obj.Ni,1);
            obj.a=[0.02*ones(obj.Ns,1); 0.2*ones(obj.Ne,1); 0.1*ones(obj.Ni,1) ]; % 0.001 (for iz Grp), 1 (sm)
            obj.b=[0.2*ones(obj.Ns,1); 0.2*ones(obj.Ne,1);  0.25-0.05*ri ]; % 0.25-0.05*ri, 0.237*
            obj.c=[-65+15*re.^2; -65*ones(obj.Ni,1)]; % -65+15*re.^2
            obj.d=[   8*ones(obj.Ns,1);   8*ones(obj.Ne,1);   2*ones(obj.Ni,1) ];

            % used to controll number of connections. See initialisation of
            % pre.
            sex=rand(obj.Ne,obj.M).*(rand(obj.Ne,obj.M)>0.0);
            obj.s=[rand(obj.Ns,obj.M);sex;-1*rand(obj.Ni,obj.M)];
            obj.smTM=sum(sum(obj.s(1:obj.Ns,:)));
            obj.sd=zeros(obj.N,obj.M);                      % their derivatives
            
            % rows: neuron numbers, col: time 1-1000, value: decaying stdp over time
            % stored per neuron
            obj.STDP = zeros(obj.N,101+obj.D);
            obj.v = -65*ones(obj.N,1);                      % initial values
            obj.u = 0.2.*obj.v;                             % initial values
            obj.firings=[-obj.D 0];                         % spike timings
            
            
        end
        
        function initConnectivity(obj)
            % POST: FROM-TO sparse 
            mRec= ceil(0.2/obj.M); % recurrent proecting to own
            mRec=20; % this is a value that worked
            mRec=0; % this is a conservative value
            % directed stuff
            obj.post=[
                randi([obj.ML(1), obj.ML(end)],[length(obj.GSL)/2, obj.M]); % GSL->ML
                randi([obj.MR(1), obj.MR(end)],[length(obj.GSL)/2, obj.M]); % GSL->MR
                
                randi([obj.ML(1), obj.ML(end)],[length(obj.BSL)/2, obj.M]); % BSL->ML
                randi([obj.MR(1), obj.MR(end)],[length(obj.BSL)/2, obj.M]); % BSL->MR
                
                randi([obj.ML(1), obj.ML(end)],[length(obj.GSR)/2, obj.M]); % GSR->ML
                randi([obj.MR(1), obj.MR(end)],[length(obj.GSR)/2, obj.M]); % GSR->MR
                
                randi([obj.ML(1), obj.ML(end)],[length(obj.BSR)/2, obj.M]); % BSR->ML
                randi([obj.MR(1), obj.MR(end)],[length(obj.BSR)/2, obj.M]); % BSR->MR
                
                randi([obj.ML(1), obj.ML(end)],[length(obj.ML), mRec]), randi([obj.IR(1), obj.IR(end)],[length(obj.ML), obj.M-mRec]);    % ML->ML (for some self sustained activity) & ML->IR (WTA)
                randi([obj.MR(1), obj.MR(end)],[length(obj.MR), mRec]), randi([obj.IL(1), obj.IL(end)],[length(obj.MR), obj.M-mRec]); % MR->MR (") & MR->IL (WTA)
                
                randi([obj.ML(1), obj.ML(end)],[length(obj.IL), obj.M/2]), randi([obj.IR(1), obj.IR(end)],[length(obj.IL), obj.M/2]) ;% IL ->ML & IL -> IR (WTA)
                randi([obj.MR(1), obj.MR(end)],[length(obj.IR), obj.M/2]), randi([obj.IL(1), obj.IL(end)],[length(obj.IR), obj.M/2]) ];% IR ->MR & IR -> IL (WTA)
            
            % PRE: TO-FROM (~100)
            for i=1:obj.N
                if i<=obj.Ns+obj.Ne
                    for j=1:obj.D
                        obj.delays{i,j}=obj.M/obj.D*(j-1)+(1:obj.M/obj.D);
                    end;
                else
                    obj.delays{i,1}=1:obj.M;
                end;
                obj.pre{i}=find(obj.post==i&obj.s>0);
                % used for STDP indexing matching to pre
                obj.aux{i}=obj.N*(obj.D-1-ceil(ceil(obj.pre{i}/obj.N)/(obj.M/obj.D)))+1+mod(obj.pre{i}-1,obj.N); 
            end;
            
            % ConIndexes btw. Sensor and Excitatory layers (indexes post)
            matSize=size(obj.post);
            for z=1:obj.Ns1
                for z2=1:obj.M
                    for z3=1:obj.Ne1
                        % green sensor left -*
                        if obj.post(obj.GSL(z),z2)==obj.ML(z3)
                            obj.cGSLML=[obj.cGSLML,sub2ind(matSize,obj.GSL(z),z2)];
                        end
                        if obj.post(obj.GSL(z),z2)==obj.MR(z3)
                            obj.cGSLMR=[obj.cGSLMR,sub2ind(matSize,obj.GSL(z),z2)];
                        end
                        % blue sensor left-*
                        if obj.post(obj.BSL(z),z2)==obj.ML(z3)
                            obj.cBSLML=[obj.cBSLML,sub2ind(matSize,obj.BSL(z),z2)];
                        end
                        if obj.post(obj.BSL(z),z2)==obj.MR(z3)
                            obj.cBSLMR=[obj.cBSLMR,sub2ind(matSize,obj.BSL(z),z2)];
                        end
                        
                        % green sensor right -*
                        if obj.post(obj.GSR(z),z2)==obj.ML(z3)
                            obj.cGSRML=[obj.cGSRML,sub2ind(matSize,obj.GSR(z),z2)];
                        end
                        if obj.post(obj.GSR(z),z2)==obj.MR(z3)
                            obj.cGSRMR=[obj.cGSRMR,sub2ind(matSize,obj.GSR(z),z2)];
                        end
                        % blue sensor right -*
                        if obj.post(obj.BSR(z),z2)==obj.ML(z3)
                            obj.cBSRML=[obj.cBSRML,sub2ind(matSize,obj.BSR(z),z2)];
                        end
                        if obj.post(obj.BSR(z),z2)==obj.MR(z3)
                            obj.cBSRMR=[obj.cBSRMR,sub2ind(matSize,obj.BSR(z),z2)];
                        end
                        
                    end
                end
            end
            
        end
        

        function initFigures(obj)
            obj.f1Contr=figure(10);
        end
    end
    
    methods
        function assignProperties(obj,network)
            obj.M=network.M; 
            obj.D=network.D; 
            obj.Ns1=network.Ns1; obj.Ns2=network.Ns2;obj.Ns3=network.Ns3;obj.Ns4=network.Ns4;
            obj.Ne1=network.Ne1; obj.Ne2=network.Ne2;
            obj.Ni1=network.Ni1; obj.Ni2=network.Ni2;
            
            
            obj.a=network.a;
            obj.b=network.b;
            obj.c=network.c;
            obj.d=network.d;
            
            obj.buildTopology;
            
            obj.s=network.s;
            obj.sm=network.sm;
            obj.sd=zeros(obj.N,obj.M);
            
            obj.STDP = zeros(obj.N,100+obj.STDPwrapover);
            obj.v = -65*ones(obj.N,1); 
            obj.u = 0.2.*obj.v; 
            obj.firings=[-obj.D 0];
            
            % connectivity
            obj.post=network.post;
            obj.pre=network.pre;
            obj.aux=network.aux;
            obj.delays=network.delays;
            obj.cGSLML=network.cGSLML;
            obj.cGSLMR=network.cGSLMR;
            obj.cGSRML=network.cGSRML;
            obj.cGSRMR=network.cGSRMR;
            obj.cBSLML=network.cBSLML;
            obj.cBSLMR=network.cBSLMR;
            obj.cBSRML=network.cBSRML;
            obj.cBSRMR=network.cBSRMR;
            
            obj.smTM=40000;%sum(sum(obj.s(1:obj.Ns,:)));
            
        end
        function obj=iz_NControllerAttractAvoid(params)
            obj.logtrace=params.logtrace;            
            obj.guiOn=params.guiOn;
            obj.ads=params.ads;
            obj.stdpsym=params.stdpsym;
            if obj.stdpsym
                obj.stdpvals=symSTDPValues;
                obj.STDPwrapover=length(obj.stdpvals);
            else
                obj.STDPwrapover=1+obj.D;
            end
            
            if params.assignNetwork
                assignProperties(obj,params.network);
            else
                buildProperties(obj);
                initConnectivity(obj);
            end
            if params.guiOn
                initFigures(obj);
                % Set up timer to control simulation updates
                obj.timerSim= timer;
                obj.timerSim.BusyMode= 'drop';
                obj.timerSim.ExecutionMode= 'fixedSpacing';
                %%%%%%%%%%Change?%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                obj.timerSim.Name= 'simStats';    % To specify timer for deletion
                obj.timerSim.ObjectVisibility= 'on';
                %%%%%%%%%%Change?%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                obj.timerSim.Period= 10;
                obj.timerSim.StartDelay=2;
                obj.timerSim.TasksToExecute= inf;
                obj.timerSim.TimerFcn= {@obj.updatePlot,obj};
                start(obj.timerSim);
                
            end
        end
        
        function [mGSL mBSL mGSR mBSR mML mMR mIL mIR mN]=topologyInfo(obj)
            mGSL=obj.GSL;
            mBSL=obj.BSL;
            mGSR=obj.GSR;
            mBSR=obj.BSR;
            mML=obj.ML;
            mMR=obj.MR;
            mIL=obj.IL;
            mIR=obj.IR;
            mN=obj.N;
        end
        function [data]=serializePlotData(obj)
            data.weightMeanSnapshot=obj.getWeightMeans;
            data.weightMean=obj.weightMean;
            data.HZPERGRP=obj.HZPERGRP;
            data.firings=obj.firingHist;
            data.s=obj.s;
            data.lastfirings=obj.lastfirings;
            data.dsec=obj.dsec;
        end
        function wmeans=getWeightMeans(obj)
            wmeans(1)=mean(obj.s(obj.cGSLML));
            wmeans(2)=mean(obj.s(obj.cGSLMR));
            
            wmeans(3)=mean(obj.s(obj.cBSLML));
            wmeans(4)=mean(obj.s(obj.cBSLMR));
            
            wmeans(5)=mean(obj.s(obj.cGSRML));
            wmeans(6)=mean(obj.s(obj.cGSRMR));
            
            wmeans(7)=mean(obj.s(obj.cBSRML));
            wmeans(8)=mean(obj.s(obj.cBSRMR));
        end
        function [network]=serializeNetwork(obj)
            network.M=obj.M; 
            network.D=obj.D;
            network.Ns1=obj.Ns1; network.Ns2=obj.Ns2;
            network.Ns3=obj.Ns3; network.Ns4=obj.Ns4;
            network.Ne1=obj.Ne1; network.Ne2=obj.Ne2;
            network.Ni1=obj.Ni1; network.Ni2=obj.Ni2;
            
            network.a=obj.a;
            network.b=obj.b;
            network.c=obj.c;
            network.d=obj.d;
            
            network.s=obj.s;
            network.sm=obj.sm;
            
            % connectivity
            network.post=obj.post;
            network.pre=obj.pre;
            network.aux=obj.aux;
            network.delays=obj.delays;
            network.cGSLML=obj.cGSLML;
            network.cGSLMR=obj.cGSLMR;
            network.cGSRML=obj.cGSRML;
            network.cGSRMR=obj.cGSRMR;
            network.cBSLML=obj.cBSLML;
            network.cBSLMR=obj.cBSLMR;
            network.cBSRML=obj.cBSRML;
            network.cBSRMR=obj.cBSRMR;
        end
        function [retHZPERGROUP]=simulationStep(obj,dsec,rw,pun,IExt)
            obj.dsec=dsec;
            for t=1:100
                I=13*(rand(obj.N,1)-0.5)+IExt;
                fired = find(obj.v>=30);                % indices of fired neurons
                obj.v(fired)=obj.c(fired);
                obj.u(fired)=obj.u(fired)+obj.d(fired);
                if obj.stdpsym
                    obj.STDP(fired,t+obj.D:t+obj.D+50-1)=repmat(obj.stdpvals,length(fired),1);
                else
                    obj.STDP(fired,t+obj.D)=0.01; % TRICK: note a incoming spike to the FUTURE!!! t+D!!!!!!!!
                end
                for k=1:length(fired)
                    obj.sd(obj.pre{fired(k)})=obj.sd(obj.pre{fired(k)}) +obj.STDP(obj.N*t+obj.aux{fired(k)}); % weight increase
                end;
                obj.firings=[obj.firings;t*ones(length(fired),1),fired];
                k=size(obj.firings,1);
                while obj.firings(k,1)>t-obj.D
                    del=obj.delays{obj.firings(k,2),t-obj.firings(k,1)+1};
                    ind = obj.post(obj.firings(k,2),del);
                    I(ind)=I(ind)+obj.s(obj.firings(k,2), del)';
                    if obj.stdpsym
                        obj.sd(obj.firings(k,2),del)=obj.sd(obj.firings(k,2),del) +obj.STDP(ind,t+obj.D)';
                    else
                        obj.sd(obj.firings(k,2),del)=obj.sd(obj.firings(k,2),del) -1.1*obj.STDP(ind,t+obj.D)'; % weight decrease
                    end
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
                
                
                if ~obj.stdpsym
                    obj.STDP(:,t+obj.D+1)=0.95*obj.STDP(:,t+obj.D);     % tau = 20 ms
                end
                obj.DA=obj.DA*0.995;
                if (mod(t,10)==0)
                     obj.s(1:obj.Ns+obj.Ne,:)=max(0,min(obj.sm,obj.s(1:obj.Ns+obj.Ne,:)+(obj.DA+0.002)*obj.sd(1:obj.Ns+obj.Ne,:)));
                     if obj.ads
                         % synaptic scaling:
                         obj.s(1:obj.Ns,:)=obj.s(1:obj.Ns,:).*obj.smTM/sum(sum(obj.s(1:obj.Ns,:)));
                     end
                    obj.sd=0.99*obj.sd;
                end;
                
                if any(rw==100*dsec+t)
                    obj.DA=obj.DA+obj.attrDA;
                end
                if any(pun==100*dsec+t)
                    obj.DA=obj.DA+obj.punDA;
                end
                
            end;
            
            CURFIRINGS=sort(obj.firings(:,2));
            % -1 since we add this in the beginning of the iteration
            obj.HZPERGRP(dsec,1)=(sum(CURFIRINGS<(obj.GSL(end)+1))-1)/obj.Ns1*10;
            obj.HZPERGRP(dsec,2)=sum(CURFIRINGS>=obj.BSL(1) & CURFIRINGS<=obj.BSL(end))/obj.Ns2*10;
            obj.HZPERGRP(dsec,3)=sum(CURFIRINGS>=obj.GSR(1) & CURFIRINGS<=obj.GSR(end))/obj.Ns3*10;
            obj.HZPERGRP(dsec,4)=sum(CURFIRINGS>=obj.BSR(1) & CURFIRINGS<=obj.BSR(end))/obj.Ns4*10;
            obj.HZPERGRP(dsec,5)=sum(CURFIRINGS>=obj.ML(1) & CURFIRINGS<=obj.ML(end))/obj.Ne1*10;
            obj.HZPERGRP(dsec,6)=sum(CURFIRINGS>=obj.MR(1) & CURFIRINGS<=obj.MR(end))/obj.Ne2*10;
            obj.HZPERGRP(dsec,7)=sum(CURFIRINGS>=obj.IL(1) & CURFIRINGS<=obj.IL(end))/obj.Ni1*10;
            obj.HZPERGRP(dsec,8)=sum(CURFIRINGS>=obj.IR(1) & CURFIRINGS<=obj.IR(end))/obj.Ni2*10;
            retHZPERGROUP=obj.HZPERGRP;
            

            
%             if obj.guiOn
% %                 obj.firingHist=[obj.firingHist; obj.firings(:,1)+obj.firingHistTscale*100, obj.firings(:,2)];
% %                 obj.firingHistTscale=obj.firingHistTscale+1;
%                  obj.updatePlot(0,0,obj);
%             end

            endIdx=size(obj.weightMean,1)+1;
            obj.weightMean(endIdx:obj.dsec,1)=mean(obj.s(obj.cGSLML));
            obj.weightMean(endIdx:obj.dsec,2)=mean(obj.s(obj.cGSLMR));
            
            obj.weightMean(endIdx:obj.dsec,3)=mean(obj.s(obj.cBSLML));
            obj.weightMean(endIdx:obj.dsec,4)=mean(obj.s(obj.cBSLMR));
            
            obj.weightMean(endIdx:obj.dsec,5)=mean(obj.s(obj.cGSRML));
            obj.weightMean(endIdx:obj.dsec,6)=mean(obj.s(obj.cGSRMR));
            
            obj.weightMean(endIdx:obj.dsec,7)=mean(obj.s(obj.cBSRML));
            obj.weightMean(endIdx:obj.dsec,8)=mean(obj.s(obj.cBSRMR));
            
            obj.STDP(:,1:obj.STDPwrapover)=obj.STDP(:,101:100+obj.STDPwrapover);
            ind = find(obj.firings(:,1) > 101-obj.D);
            obj.lastfirings=obj.firings;
            obj.firings=[-obj.D 0;obj.firings(ind,1)-100,obj.firings(ind,2)];
            
            
            
        end
        function delete(this)
            disp('Delete method called');
            delete(this.timerSim);
        end
    end
    methods(Static) 
        function updatePlot(timerSim,event,obj)
          %  if obj.firingHistTscale>0
            % obj is user supplied arg to this class
            endIdx=size(obj.weightMean,1)+1;
            obj.weightMean(endIdx:obj.dsec,1)=mean(obj.s(obj.cGSLML));
            obj.weightMean(endIdx:obj.dsec,2)=mean(obj.s(obj.cGSLMR));
            
            obj.weightMean(endIdx:obj.dsec,3)=mean(obj.s(obj.cBSLML));
            obj.weightMean(endIdx:obj.dsec,4)=mean(obj.s(obj.cBSLMR));
            
            obj.weightMean(endIdx:obj.dsec,5)=mean(obj.s(obj.cGSRML));
            obj.weightMean(endIdx:obj.dsec,6)=mean(obj.s(obj.cGSRMR));
            
            obj.weightMean(endIdx:obj.dsec,7)=mean(obj.s(obj.cBSRML));
            obj.weightMean(endIdx:obj.dsec,8)=mean(obj.s(obj.cBSRMR));
            
            
                set(0,'CurrentFigure',obj.f1Contr);
                subplot(3,2,1:2)
                Hz=length(obj.lastfirings(:,2))/100/obj.N*1000;
                plot(obj.lastfirings(:,1),obj.lastfirings(:,2),'.');
                title(['Firings: ' num2str(Hz) 'Hz']);
                axis([0 100 0 obj.N]);
                
%                 set(0,'CurrentFigure',obj.f1Contr);
%                 subplot(3,2,1:2)
%                 Hz=length(obj.firings(:,2))/100/obj.N*1000;
%                 plot(obj.firings(:,1),obj.firings(:,2),'.');
%                 title(['Firings: ' num2str(Hz) 'Hz']);
%                 axis([0 100 0 obj.N]);
                
%                 set(0,'CurrentFigure',obj.f1Contr);
%                 subplot(3,2,1:2)
%                 Hz=length(obj.firingHist(:,2))/100/obj.N*1000;
%                 plot(obj.firingHist(:,1),obj.firingHist(:,2),'.');
%                 title(['Firings: ' num2str(Hz) 'Hz']);
%                 axis([0 obj.firingHistTscale*100 0 obj.N]);
                
                
%                 obj.firingHistTscale=0;
%                 obj.firingHist=[];
                
                %subplot(3,2,2)
                %hist(obj.s(find(obj.s>0)),obj.sm*(0.01:0.01:1)); % only excitatory synapses
                
                subplot(3,2,3);
                plot(1:obj.dsec,obj.weightMean(:,1:2) );
                legend('GSL-ML','GSL-MR','Location','NorthWest');
                title('Mean Weights GSL-*');
                ylabel('weight');
                
                subplot(3,2,4);
                plot(1:obj.dsec,obj.weightMean(:,3:4) );
                legend('BSL-ML','BSL-MR','Location','NorthWest');
                title('Mean Weights BSL-*');
                ylabel('weight');
                
                subplot(3,2,5);
                plot(1:obj.dsec,obj.weightMean(:,5:6) );
                legend('GSR-ML','GSR-MR','Location','NorthWest');
                title('Mean Weights GSR-*');
                ylabel('weight');
                
                subplot(3,2,6);
                plot(1:obj.dsec,obj.weightMean(:,7:8) );
                legend('BSR-ML','BSR-MR','Location','NorthWest');
                title('Mean Weights BSR-*');
                ylabel('weight');             
                
                drawnow;
                % ---- end plot ------
            %end
        end
        
    end
    
    
end