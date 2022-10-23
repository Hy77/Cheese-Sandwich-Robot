classdef IRB < handle
	properties
		%> Robot model
		model; model_pos;
		%>
		workspace = [-2 2 -2 2 0 2];
		
		%> Flag to indicate if gripper is used
		useGripper = false;
        

	end
	
	methods%% Class for IRB robot simulation
		function self = IRB()
			self.GetIRBRobot();
			% robot =
			self.PlotAndColourRobot();%robot,workspace);
			
		end
		
		%% GetIRBCuteRobot
		% Given a name (optional), create and return a IRB robot model
		function GetIRBRobot(self)
			%     if nargin < 1
			% Create a unique name (ms timestamp after 1ms pause)
			pause(0.001);
			name = ['IRB',datestr(now,'yyyymmddTHHMMSSFFF')];
			%     end
            L1 = Link([0     0.248      0       pi/2     0]);
            L2 = Link([0      0       0.22       0       0]);
            L3 = Link([0      0         0      -pi/2     0]);
            L4 = Link([0     -0.295     0       pi/2     0]);
            L5 = Link([0      0         0      -pi/2     0]);
            L6 = Link([0     -0.059     0        0	   0]);
            
%             L1.qlim = [-360 360]*pi/180;
%             L2.qlim = [-122 135]*pi/180;
%             L3.qlim = [-40  235]*pi/180;
%             L4.qlim = [-360 360]*pi/180;
%             L5.qlim = [-135 150]*pi/180;
%             L6.qlim = [-360 360]*pi/180;
            
			
            L2.offset = pi/2;
            L4.offset = pi;
            L5.offset = pi/2;
           
            
			self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',name);
                        self.model_pos = [-0.18,0.5,0.775]; % set robot base's position
             pos = makehgtform('translate',[self.model_pos]); % Rotate/Move robot to the correct orientation
             self.model.base = self.model.base * pos;
            
		end
		%% PlotAndColourRobot
		% Given a robot index, add the glyphs (vertices and faces) and
		% colour them in if data is available
		function PlotAndColourRobot(self)%robot,workspace)
			for linkIndex = 0:self.model.n
				if self.useGripper && linkIndex == self.model.n
					[ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['IRB_',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
				else
					[ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['IRB_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
				end
				self.model.faces{linkIndex+1} = faceData;
				self.model.points{linkIndex+1} = vertexData;
			end
			
			% Display robot
			self.model.plot3d(zeros(1,self.model.n), 'noarrow', 'workspace', self.workspace);
			if isempty(findobj(get(gca, 'Children'), 'Type', 'Light'))
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
		
		%% Advanced Teach Section to allow robot movement with inverse kinematics
		function ModelIKinematics(self, qMatrix, robot)
			numSteps = size(qMatrix);
			numSteps = numSteps(1);
			for i = 1:numSteps

				animate(robot.model, qMatrix(i, :));

			end
		end
		

		%% Intrerpolate joint angles (lab4)for advanced teach
		function qMatrix = PathIK(self, q0, Posq1, numSteps)
			s = lspb(0, 1, numSteps);
			qMatrix = zeros(numSteps, 6); %based on the number of joints for the robot
			for  i = 1:numSteps
				qMatrix(i,:) = q0 + s(i) * (Posq1 - q0);
			end
        end

	end
end
