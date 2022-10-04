classdef czRobot < handle
    properties
        %> Robot model
        model; model_pos;
        
        %> workspace
        workspace = [-0.6 0.6 -0.6 0.6 0 1.1];   
      
    end
    
    methods%% Class for IRB robot simulation

        function self = czRobot(~)
            clf
            self.GetIRBRobot();
           % self.PlotAndColourRobot();%robot,workspace);

            drawnow
        end

        %% GetIRBRobot
        % Given a name (optional), create and return a IRB robot model
        function GetIRBRobot(self)
            
            L(1) = Link([0     0.085      0       pi/2     0]); % base
            L(2) = Link([0      0       0.099      0      0]);
            L(3) = Link([0      0         0       -pi/2    0]);
            L(4) = Link([0     0.1        0       pi/2     0]);
            L(5) = Link([0      0         0       -pi/2    0]);
            L(6) = Link([0     0.05       0         0	   0]);
            % Incorporate joint limits
            L(1).qlim = [-360 360]*pi/180;
            L(2).qlim = [-360 360]*pi/180;
            L(3).qlim = [-360 360]*pi/180;
            L(4).qlim = [-360 360]*pi/180;
            L(5).qlim = [-360 360]*pi/180;
            L(6).qlim = [-360 360]*pi/180;  
% 
            L(2).offset = pi/2;
            L(3).offset = -pi;
            
            self.model = SerialLink(L,'name','IRB');
            scale = 0.5;                                                    
            q = zeros(1,6);                                                   
            self.model.plot(q,'workspace',self.workspace,'scale',scale)
%             
            self.model_pos = [0,0,0];
            pos = makehgtform('translate',[self.model_pos]);
            self.model.base = self.model.base * pos;
            
            self.model.teach
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
%                 if linkIndex == 0
%                     [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['ur3link_0.ply'],'tri'); %#ok<AGROW> 
%                 else
%                     [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['IRB_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>   
%                 end        
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['IRB_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW> 
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end        
    end
end
