classdef czRobot < handle
    properties
        %> Robot model
        IRB; IRB_pos; UR3; UR3_pos; cp_board; cp_board1; table; table1;
        kitchen; kitchen1; fridge; fridge1; bread1; bread1s; bread2; bread2s;
        bread3; bread3s; bread4; bread4s; bread5; bread5s; basket; basket1;
        gp_base; gp_base1; gp_fg1;gp_fg1s; gp_fg2;gp_fg2s; gp_fg3;gp_fg3s;
        c_block; c_block1; c_blocks; c_block2; c_slice; c_slice1;
        
        %> workspace
        workspace = [-2 2 -2 2 0 2];
        
    end
    
    methods%% Class for kitchen robot simulation
        
        function self = czRobot(~)
            clf
            self.GetIRBRobot();
            self.GetUR3Robot();
            self.PlotAndColourRobot();
            drawnow
        end
        
        %% GetIRBRobot
        function GetIRBRobot(self)
            % DH parameters
            L_IRB(1) = Link([0     0.248      0       pi/2     0]); % base
            L_IRB(2) = Link([0      0       0.22       0       0]);
            L_IRB(3) = Link([0      0         0      -pi/2     0]);
            L_IRB(4) = Link([0     -0.295     0       pi/2     0]);
            L_IRB(5) = Link([0      0         0      -pi/2     0]);
            L_IRB(6) = Link([0     -0.059     0        0	   0]);
            % Incorporate joint limits
            L_IRB(1).qlim = [-360 360]*pi/180;
            L_IRB(2).qlim = [-122 135]*pi/180;
            L_IRB(3).qlim = [-40  235]*pi/180;
            L_IRB(4).qlim = [-360 360]*pi/180;
            L_IRB(5).qlim = [-135 150]*pi/180;
            L_IRB(6).qlim = [-360 360]*pi/180;
            % Offset
            L_IRB(2).offset = pi/2;
            L_IRB(4).offset = pi;
            L_IRB(5).offset = pi/2;
            
            self.IRB = SerialLink(L_IRB,'name','IRB');
            self.IRB_pos = [-0.18,0.5,0.775]; % set robot base's position
            pos = makehgtform('translate',[self.IRB_pos]); % Rotate/Move robot to the correct orientation
            self.IRB.base = self.IRB.base * pos;
        end
        
        %% GetUR3Robot
        function GetUR3Robot(self)
            
            % DH parameters
            L_UR3(1) = Link([0      0.1519      0        pi/2      0]); % base
            L_UR3(2) = Link([0      0       -0.24365       0       0]);
            L_UR3(3) = Link([0      0       -0.21325       0       0]);
            L_UR3(4) = Link([0      0.11235     0        pi/2      0]);
            L_UR3(5) = Link([0      0.08535     0       -pi/2	   0]);
            L_UR3(6) = Link([0      0.092      0          0       0]);
            % Incorporate joint limits
            L_UR3(1).qlim = [-360 360]*pi/180;
            L_UR3(2).qlim = [-360 360]*pi/180;
            L_UR3(3).qlim = [-205 100]*pi/180;
            L_UR3(4).qlim = [-360 360]*pi/180;
            L_UR3(5).qlim = [-360 360]*pi/180;
            L_UR3(6).qlim = [-360 360]*pi/180;
            % Offset
            L_UR3(2).offset = -pi/2;
            L_UR3(3).offset = -pi/2;
            L_UR3(4).offset = -pi;
            L_UR3(5).offset = pi;
            
            self.UR3 = SerialLink(L_UR3,'name','UR3');
            self.UR3_pos = [-0.18,-0.5,0.775]; % set robot base's position
            pos = makehgtform('translate',[self.UR3_pos]); % Rotate/Move robot to the correct orientation
            self.UR3.base = self.UR3.base * pos;
        end
        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available
        function PlotAndColourRobot(self)%robot,workspace)
            % colour the IRB robot
            for linkIndex = 0:self.IRB.n
                [ faceData, vertexData, plyData1{linkIndex + 1} ] = plyread(['IRB_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.IRB.faces{linkIndex + 1} = faceData;
                self.IRB.points{linkIndex + 1} = vertexData;
            end
            % colour the UR3 robot
            for linkIndex = 0:self.UR3.n
                [ faceData, vertexData, plyData2{linkIndex + 1} ] = plyread(['ur3link_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.UR3.faces{linkIndex + 1} = faceData;
                self.UR3.points{linkIndex + 1} = vertexData;
            end
            
            % colour the chopping board
            [f,v,data] = plyread('Chopping_board.ply','tri');
            self.cp_board.VertexCount = size(v,1);
            self.cp_board.midPoint = sum(v)/self.cp_board.VertexCount; % find the midPoint of the cp_board
            self.cp_board.baseVerts = v - repmat(self.cp_board.midPoint,self.cp_board.VertexCount,1);  % find the vertex of the cp_board
            self.cp_board.basePose = eye(4);
            self.cp_board.vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255; % set cp_board's colour
            self.cp_board1 = trisurf(f,v(:,1),v(:,2),v(:,3),'FaceVertexCData', self.cp_board.vertexColours,'EdgeColor','none','EdgeLighting','none'); % plot the cp_board all the faces & colour
            forwardTR = makehgtform('translate',[-0.15,0.15,0.79]); % set the origin/start point of the cp_board
            rotateTRy = makehgtform('yrotate',(pi/2));
            rotateTRx = makehgtform('xrotate',(pi/2));
            self.cp_board.basePose = self.cp_board.basePose * forwardTR * rotateTRy * rotateTRx; % let the cp_board move to the specified position
            updatedPoints = [self.cp_board.basePose * [self.cp_board.baseVerts,ones(self.cp_board.VertexCount,1)]']'; % get the new position
            self.cp_board1.Vertices = updatedPoints(:,1:3); % updated the cp_board's location
            hold on
            
            % colour the table
            [ft,vt,datat] = plyread('table.ply','tri');
            self.table.VertexCount = size(vt,1);
            self.table.midPoint = sum(vt)/self.table.VertexCount; % find the midPoint of the table
            self.table.baseVerts = vt - repmat(self.table.midPoint,self.table.VertexCount,1);  % find the vertex of the table
            self.table.basePose = eye(4);
            self.table.vertexColours = [datat.vertex.red, datat.vertex.green, datat.vertex.blue] / 255; % set table's colour
            self.table1 = trisurf(ft,vt(:,1),vt(:,2),vt(:,3),'FaceVertexCData', self.table.vertexColours,'EdgeColor','none','EdgeLighting','none'); % plot the table all the faces & colour
            forwardTR = makehgtform('translate',[0,0,0.417]); % set the origin/start point of the table 0.08
            rotateTRx = makehgtform('xrotate',(pi/2));
            self.table.basePose = self.table.basePose * forwardTR * rotateTRx; % let the table move to the specified position
            updatedPoints = [self.table.basePose * [self.table.baseVerts,ones(self.table.VertexCount,1)]']'; % get the new position
            self.table1.Vertices = updatedPoints(:,1:3); % updated the table's location
            hold on
            
            % colour the basket
            [fbck,vgbck,databck] = plyread('basket.ply','tri');
            self.basket.VertexCount = size(vgbck,1);
            self.basket.midPoint = sum(vgbck)/self.basket.VertexCount; % find the midPoint of the basket
            self.basket.baseVerts = vgbck - repmat(self.basket.midPoint,self.basket.VertexCount,1);  % find the vertex of the basket
            self.basket.basePose = eye(4);
            self.basket.vertexColours = [databck.vertex.red, databck.vertex.green, databck.vertex.blue] / 255; % set basket's colour
            self.basket1 = trisurf(fbck,vgbck(:,1),vgbck(:,2),vgbck(:,3),'FaceVertexCData', self.basket.vertexColours,'EdgeColor','none','EdgeLighting','none'); % plot the basket all the faces & colour
            forwardTR = makehgtform('translate',[0.27,-0.6,0.82]); % set the origin/start point of the basket
            rotateTRz = makehgtform('zrotate',(pi/2));
            self.basket.basePose = self.basket.basePose * forwardTR * rotateTRz; % let the basket move to the specified position
            updatedPoints = [self.basket.basePose * [self.basket.baseVerts,ones(self.basket.VertexCount,1)]']'; % get the new position
            self.basket1.Vertices = updatedPoints(:,1:3); % updated the basket's location
            hold on
            
            % colour the gripper_base
            [fgb,vgb,datagb] = plyread('gripper_base.ply','tri');
            self.gp_base.VertexCount = size(vgb,1);
            self.gp_base.midPoint = sum(vgb)/self.gp_base.VertexCount; % find the midPoint of the gp_base
            self.gp_base.baseVerts = vgb - repmat(self.gp_base.midPoint,self.gp_base.VertexCount,1);  % find the vertex of the gp_base
            self.gp_base.basePose = eye(4);
            self.gp_base.vertexColours = [datagb.vertex.red, datagb.vertex.green, datagb.vertex.blue] / 255; % set gp_base's colour
            self.gp_base1 = trisurf(fgb,vgb(:,1),vgb(:,2),vgb(:,3),'FaceVertexCData', self.gp_base.vertexColours,'EdgeColor','none','EdgeLighting','none'); % plot the gp_base all the faces & colour
            forwardTR = makehgtform('translate',[0.033,-0.5203,1.085]); % set the origin/start point of the gp_base
            rotateTRx = makehgtform('xrotate',(-pi/2));
            self.gp_base.basePose = self.gp_base.basePose * forwardTR * rotateTRx; % let the gp_base move to the specified position
            updatedPoints = [self.gp_base.basePose * [self.gp_base.baseVerts,ones(self.gp_base.VertexCount,1)]']'; % get the new position
            self.gp_base1.Vertices = updatedPoints(:,1:3); % updated the gp_base's location
            hold on
            
            % colour the gripper_fingers
            [fgf,vgf,datagf] = plyread('gripper_finger_v2.ply','tri');
            % colour the gripper_finger1
            self.gp_fg1.VertexCount = size(vgf,1);
            self.gp_fg1.midPoint = sum(vgf)/self.gp_fg1.VertexCount; % find the midPoint of the gp_fg1
            self.gp_fg1.baseVerts = vgf - repmat(self.gp_fg1.midPoint,self.gp_fg1.VertexCount,1);  % find the vertex of the gp_fg1
            self.gp_fg1.basePose = eye(4);
            self.gp_fg1.vertexColours = [datagf.vertex.red, datagf.vertex.green, datagf.vertex.blue] / 255; % set gp_fg1's colour
            %self.gp_fg1s = trisurf(fgf,vgf(:,1),vgf(:,2),vgf(:,3),'EdgeColor','interp','EdgeLighting','flat'); % plot the gp_fg1 all the faces & colour
            self.gp_fg1s = trisurf(fgf,vgf(:,1),vgf(:,2),vgf(:,3),'FaceVertexCData', self.gp_fg1.vertexColours,'EdgeColor','none','EdgeLighting','none'); % plot the gp_fg1 all the faces & colour
            forwardTR = makehgtform('translate',[0.033,-0.5203,1.085]); % set the origin/start point of the gp_fg1 0.033,-0.5,1.11
            self.gp_fg1.basePose = self.gp_fg1.basePose * forwardTR; % let the gp_fg1 move to the specified position
            updatedPoints = [self.gp_fg1.basePose * [self.gp_fg1.baseVerts,ones(self.gp_fg1.VertexCount,1)]']'; % get the new position
            self.gp_fg1s.Vertices = updatedPoints(:,1:3); % updated the gp_fg1's location
            hold on
            % colour the gripper_finger2
            self.gp_fg2.VertexCount = size(vgf,1);
            self.gp_fg2.midPoint = sum(vgf)/self.gp_fg2.VertexCount; % find the midPoint of the gp_fg2
            self.gp_fg2.baseVerts = vgf - repmat(self.gp_fg2.midPoint,self.gp_fg2.VertexCount,1);  % find the vertex of the gp_fg2
            self.gp_fg2.basePose = eye(4);
            self.gp_fg2.vertexColours = [datagf.vertex.red, datagf.vertex.green, datagf.vertex.blue] / 255; % set gp_fg2's colour
            self.gp_fg2s = trisurf(fgf,vgf(:,1),vgf(:,2),vgf(:,3),'FaceVertexCData', self.gp_fg2.vertexColours,'EdgeColor','none','EdgeLighting','none'); % plot the gp_fg2 all the faces & colour
            forwardTR = makehgtform('translate',[0.033,-0.5203,1.085]); % set the origin/start point of the gp_fg2 0.054,-0.5,1.07
            rotateTRy = makehgtform('yrotate',(2*pi/3));
            self.gp_fg2.basePose = self.gp_fg2.basePose * forwardTR * rotateTRy; % let the gp_fg2 move to the specified position
            updatedPoints = [self.gp_fg2.basePose * [self.gp_fg2.baseVerts,ones(self.gp_fg2.VertexCount,1)]']'; % get the new position
            self.gp_fg2s.Vertices = updatedPoints(:,1:3); % updated the gp_fg2's location
            hold on
            % colour the gripper_finger3
            self.gp_fg3.VertexCount = size(vgf,1);
            self.gp_fg3.midPoint = sum(vgf)/self.gp_fg3.VertexCount; % find the midPoint of the gp_fg3
            self.gp_fg3.baseVerts = vgf - repmat(self.gp_fg3.midPoint,self.gp_fg3.VertexCount,1);  % find the vertex of the gp_fg3
            self.gp_fg3.basePose = eye(4);
            self.gp_fg3.vertexColours = [datagf.vertex.red, datagf.vertex.green, datagf.vertex.blue] / 255; % set gp_fg3's colour
            self.gp_fg3s = trisurf(fgf,vgf(:,1),vgf(:,2),vgf(:,3),'FaceVertexCData', self.gp_fg3.vertexColours,'EdgeColor','none','EdgeLighting','none'); % plot the gp_fg3 all the faces & colour
            forwardTR = makehgtform('translate',[0.033,-0.5203,1.085]); % set the origin/start point of the gp_fg3 0.011,-0.5,1.07
            rotateTRy = makehgtform('yrotate',(-2*pi/3));
            self.gp_fg3.basePose = self.gp_fg3.basePose * forwardTR * rotateTRy; % let the gp_fg3 move to the specified position
            updatedPoints = [self.gp_fg3.basePose * [self.gp_fg3.baseVerts,ones(self.gp_fg3.VertexCount,1)]']'; % get the new position
            self.gp_fg3s.Vertices = updatedPoints(:,1:3); % updated the gp_fg3's location
            hold on
            
            % colour the breads
            [fb,vb,datab] = plyread('bread.ply','tri');
            % colour the bread1
            self.bread1.VertexCount = size(vb,1);
            self.bread1.midPoint = sum(vb)/self.bread1.VertexCount; % find the midPoint of the bread1
            self.bread1.baseVerts = vb - repmat(self.bread1.midPoint,self.bread1.VertexCount,1);  % find the vertex of the bread1
            self.bread1.basePose = eye(4);
            self.bread1.vertexColours = [datab.vertex.red, datab.vertex.green, datab.vertex.blue] / 255; % set bread1's colour
            self.bread1s = trisurf(fb,vb(:,1),vb(:,2),vb(:,3),'FaceVertexCData', self.bread1.vertexColours,'EdgeColor','none','EdgeLighting','none'); % plot the bread1 all the faces & colour
            forwardTR = makehgtform('translate',[0.1850,-0.53,0.87]); % set the origin/start point of the bread1
            rotateTRy = makehgtform('yrotate',(-pi/2));
            self.bread1.basePose = self.bread1.basePose * forwardTR * rotateTRy; % let the bread1 move to the specified position
            updatedPoints = [self.bread1.basePose * [self.bread1.baseVerts,ones(self.bread1.VertexCount,1)]']'; % get the new position
            self.bread1s.Vertices = updatedPoints(:,1:3); % updated the bread1's location
            hold on
            % colour the bread2
            self.bread2.VertexCount = size(vb,1);
            self.bread2.midPoint = sum(vb)/self.bread2.VertexCount; % find the midPoint of the bread2
            self.bread2.baseVerts = vb - repmat(self.bread2.midPoint,self.bread2.VertexCount,1);  % find the vertex of the bread2
            self.bread2.basePose = eye(4);
            self.bread2.vertexColours = [datab.vertex.red, datab.vertex.green, datab.vertex.blue] / 255; % set bread2's colour
            self.bread2s = trisurf(fb,vb(:,1),vb(:,2),vb(:,3),'FaceVertexCData', self.bread2.vertexColours,'EdgeColor','none','EdgeLighting','none'); % plot the bread2 all the faces & colour
            forwardTR = makehgtform('translate',[0.2075,-0.53,0.87]); % set the origin/start point of the bread2
            rotateTRy = makehgtform('yrotate',(-pi/2));
            self.bread2.basePose = self.bread2.basePose * forwardTR * rotateTRy; % let the bread2 move to the specified position
            updatedPoints = [self.bread2.basePose * [self.bread2.baseVerts,ones(self.bread2.VertexCount,1)]']'; % get the new position
            self.bread2s.Vertices = updatedPoints(:,1:3); % updated the bread2's location
            hold on
            % colour the bread3
            self.bread3.VertexCount = size(vb,1);
            self.bread3.midPoint = sum(vb)/self.bread3.VertexCount; % find the midPoint of the bread3
            self.bread3.baseVerts = vb - repmat(self.bread3.midPoint,self.bread3.VertexCount,1);  % find the vertex of the bread3
            self.bread3.basePose = eye(4);
            self.bread3.vertexColours = [datab.vertex.red, datab.vertex.green, datab.vertex.blue] / 255; % set bread3's colour
            self.bread3s = trisurf(fb,vb(:,1),vb(:,2),vb(:,3),'FaceVertexCData', self.bread3.vertexColours,'EdgeColor','none','EdgeLighting','none'); % plot the bread3 all the faces & colour
            forwardTR = makehgtform('translate',[0.23,-0.53,0.87]); % set the origin/start point of the bread3
            rotateTRy = makehgtform('yrotate',(-pi/2));
            self.bread3.basePose = self.bread3.basePose * forwardTR * rotateTRy; % let the bread3 move to the specified position
            updatedPoints = [self.bread3.basePose * [self.bread3.baseVerts,ones(self.bread3.VertexCount,1)]']'; % get the new position
            self.bread3s.Vertices = updatedPoints(:,1:3); % updated the bread3's location
            hold on
            % colour the bread4
            self.bread4.VertexCount = size(vb,1);
            self.bread4.midPoint = sum(vb)/self.bread4.VertexCount; % find the midPoint of the bread4
            self.bread4.baseVerts = vb - repmat(self.bread4.midPoint,self.bread4.VertexCount,1);  % find the vertex of the bread4
            self.bread4.basePose = eye(4);
            self.bread4.vertexColours = [datab.vertex.red, datab.vertex.green, datab.vertex.blue] / 255; % set bread4's colour
            self.bread4s = trisurf(fb,vb(:,1),vb(:,2),vb(:,3),'FaceVertexCData', self.bread4.vertexColours,'EdgeColor','none','EdgeLighting','none'); % plot the bread4 all the faces & colour
            forwardTR = makehgtform('translate',[0.2525,-0.53,0.87]); % set the origin/start point of the bread4
            rotateTRy = makehgtform('yrotate',(-pi/2));
            self.bread4.basePose = self.bread4.basePose * forwardTR * rotateTRy; % let the bread4 move to the specified position
            updatedPoints = [self.bread4.basePose * [self.bread4.baseVerts,ones(self.bread4.VertexCount,1)]']'; % get the new position
            self.bread4s.Vertices = updatedPoints(:,1:3); % updated the bread4's location
            hold on
            % colour the bread5
            self.bread5.VertexCount = size(vb,1);
            self.bread5.midPoint = sum(vb)/self.bread5.VertexCount; % find the midPoint of the bread5
            self.bread5.baseVerts = vb - repmat(self.bread5.midPoint,self.bread5.VertexCount,1);  % find the vertex of the bread5
            self.bread5.basePose = eye(4);
            self.bread5.vertexColours = [datab.vertex.red, datab.vertex.green, datab.vertex.blue] / 255; % set bread5's colour
            self.bread5s = trisurf(fb,vb(:,1),vb(:,2),vb(:,3),'FaceVertexCData', self.bread5.vertexColours,'EdgeColor','none','EdgeLighting','none'); % plot the bread5 all the faces & colour
            forwardTR = makehgtform('translate',[0.275,-0.53,0.87]); % set the origin/start point of the bread5
            rotateTRy = makehgtform('yrotate',(-pi/2));
            self.bread5.basePose = self.bread5.basePose * forwardTR * rotateTRy; % let the bread5 move to the specified position
            updatedPoints = [self.bread5.basePose * [self.bread5.baseVerts,ones(self.bread5.VertexCount,1)]']'; % get the new position
            self.bread5s.Vertices = updatedPoints(:,1:3); % updated the bread5's location
            hold on
            
            % colour the c_block
            [fcb,vcb,datacb] = plyread('cheese_block.ply','tri');
            self.c_block.VertexCount = size(vcb,1);
            self.c_block.midPoint = sum(vcb)/self.c_block.VertexCount; % find the midPoint of the c_block
            self.c_block.baseVerts = vcb - repmat(self.c_block.midPoint,self.c_block.VertexCount,1);  % find the vertex of the c_block
            self.c_block.basePose = eye(4);
            self.c_block.vertexColours = [datacb.vertex.red, datacb.vertex.green, datacb.vertex.blue] / 255; % set c_block's colour
            self.c_block1 = trisurf(fcb,vcb(:,1),vcb(:,2),vcb(:,3),'FaceVertexCData', self.c_block.vertexColours,'EdgeColor','none','EdgeLighting','none'); % plot the c_block all the faces & colour
            forwardTR = makehgtform('translate',[0.275,-0.685,0.87]); % set the origin/start point of the c_block
            self.c_block.basePose = self.c_block.basePose * forwardTR; % let the c_block move to the specified position
            updatedPoints = [self.c_block.basePose * [self.c_block.baseVerts,ones(self.c_block.VertexCount,1)]']'; % get the new position
            self.c_block1.Vertices = updatedPoints(:,1:3); % updated the c_block's location
            hold on
            
            % colour the c_block2
            [fcb,vcb,datacb] = plyread('cheese_block2.ply','tri');
            self.c_blocks.VertexCount = size(vcb,1);
            self.c_blocks.midPoint = sum(vcb)/self.c_blocks.VertexCount; % find the midPoint of the c_blocks
            self.c_blocks.baseVerts = vcb - repmat(self.c_blocks.midPoint,self.c_blocks.VertexCount,1);  % find the vertex of the c_blocks
            self.c_blocks.basePose = eye(4);
            self.c_blocks.vertexColours = [datacb.vertex.red, datacb.vertex.green, datacb.vertex.blue] / 255; % set c_blocks's colour
            self.c_block2 = trisurf(fcb,vcb(:,1),vcb(:,2),vcb(:,3),'FaceVertexCData', self.c_blocks.vertexColours,'EdgeColor','none','EdgeLighting','none'); % plot the c_blocks all the faces & colour
            forwardTR = makehgtform('translate',[0,0,0.5]); % set the origin/start point of the c_blocks
            self.c_blocks.basePose = self.c_blocks.basePose * forwardTR; % let the c_blocks move to the specified position
            updatedPoints = [self.c_blocks.basePose * [self.c_blocks.baseVerts,ones(self.c_blocks.VertexCount,1)]']'; % get the new position
            self.c_block2.Vertices = updatedPoints(:,1:3); % updated the c_blocks's location
            hold on
            
            % colour the c_slice
            [fcb,vcb,datacb] = plyread('cheese_slice.ply','tri');
            self.c_slice.VertexCount = size(vcb,1);
            self.c_slice.midPoint = sum(vcb)/self.c_slice.VertexCount; % find the midPoint of the c_slice
            self.c_slice.baseVerts = vcb - repmat(self.c_slice.midPoint,self.c_slice.VertexCount,1);  % find the vertex of the c_slice
            self.c_slice.basePose = eye(4);
            self.c_slice.vertexColours = [datacb.vertex.red, datacb.vertex.green, datacb.vertex.blue] / 255; % set c_slice's colour
            self.c_slice1 = trisurf(fcb,vcb(:,1),vcb(:,2),vcb(:,3),'FaceVertexCData', self.c_slice.vertexColours,'EdgeColor','none','EdgeLighting','none'); % plot the c_slice all the faces & colour
            forwardTR = makehgtform('translate',[0.08,0,0.5]); % set the origin/start point of the c_slice
            self.c_slice.basePose = self.c_slice.basePose * forwardTR; % let the c_slice move to the specified position
            updatedPoints = [self.c_slice.basePose * [self.c_slice.baseVerts,ones(self.c_slice.VertexCount,1)]']'; % get the new position
            self.c_slice1.Vertices = updatedPoints(:,1:3); % updated the c_slice's location
            hold on
            
            
            % colour the kitchen
            [fk,vk,datak] = plyread('kitchen.ply','tri');
            self.kitchen.VertexCount = size(vk,1);
            self.kitchen.midPoint = sum(vk)/self.kitchen.VertexCount; % find the midPoint of the kitchen
            self.kitchen.baseVerts = vk - repmat(self.kitchen.midPoint,self.kitchen.VertexCount,1);  % find the vertex of the kitchen
            self.kitchen.basePose = eye(4);
            self.kitchen.vertexColours = [datak.vertex.red, datak.vertex.green, datak.vertex.blue] / 255; % set kitchen's colour
            self.kitchen1 = trisurf(fk,vk(:,1),vk(:,2),vk(:,3),'FaceVertexCData', self.kitchen.vertexColours,'EdgeColor','none','EdgeLighting','none'); % plot the kitchen all the faces & colour
            forwardTR = makehgtform('translate',[-1.6,0.7,1]); % set the origin/start point of the kitchen
            rotateTRx = makehgtform('xrotate',(pi/2));

            self.kitchen.basePose = self.kitchen.basePose * forwardTR * rotateTRx; % let the kitchen move to the specified position
            updatedPoints = [self.kitchen.basePose * [self.kitchen.baseVerts,ones(self.kitchen.VertexCount,1)]']'; % get the new position
            self.kitchen1.Vertices = updatedPoints(:,1:3); % updated the kitchen's location
            hold on
            
            % colour the fridge
            [ff,vf,dataf] = plyread('fridge.ply','tri');
            self.fridge.VertexCount = size(vf,1);
            self.fridge.midPoint = sum(vf)/self.fridge.VertexCount; % find the midPoint of the fridge
            self.fridge.baseVerts = vf - repmat(self.fridge.midPoint,self.fridge.VertexCount,1);  % find the vertex of the fridge
            self.fridge.basePose = eye(4);
            self.fridge.vertexColours = [dataf.vertex.red, dataf.vertex.green, dataf.vertex.blue] / 255; % set fridge's colour
            self.fridge1 = trisurf(ff,vf(:,1),vf(:,2),vf(:,3),'FaceVertexCData', self.fridge.vertexColours,'EdgeColor','none','EdgeLighting','none'); % plot the fridge all the faces & colour
            forwardTR = makehgtform('translate',[-1.6,-1.5,0.97]); % set the origin/start point of the fridge
            rotateTRx = makehgtform('xrotate',(pi/2));
            rotateTRy = makehgtform('yrotate',(pi/2));
            self.fridge.basePose = self.fridge.basePose * forwardTR * rotateTRx * rotateTRy; % let the fridge move to the specified position
            updatedPoints = [self.fridge.basePose * [self.fridge.baseVerts,ones(self.fridge.VertexCount,1)]']'; % get the new position
            self.fridge1.Vertices = updatedPoints(:,1:3); % updated the fridge's location
            hold on
            
            
            % Display IRB robot
            self.IRB.plot3d(zeros(1,self.IRB.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.IRB.delay = 0;
            
            % Display UR3 robot
            self.UR3.plot3d(zeros(1,self.UR3.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.UR3.delay = 0;
            
            % Try to correctly colour the IRB arm (if colours are in ply file data)
            for linkIndex = 0:self.IRB.n
                handle_IRB = findobj('Tag', self.IRB.name);
                h_IRB = get(handle_IRB,'UserData');
                try
                    h_IRB.link(linkIndex+1).Children.FaceVertexCData = [plyData1{linkIndex+1}.vertex.red ...
                        , plyData1{linkIndex+1}.vertex.green ...
                        , plyData1{linkIndex+1}.vertex.blue]/255;
                    h_IRB.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
            
            % Try to correctly colour the UR3 arm (if colours are in ply file data)
            for linkIndex = 0:self.UR3.n
                handle_UR3 = findobj('Tag', self.UR3.name);
                h_UR3 = get(handle_UR3,'UserData');
                try
                    h_UR3.link(linkIndex+1).Children.FaceVertexCData = [plyData2{linkIndex+1}.vertex.red ...
                        , plyData2{linkIndex+1}.vertex.green ...
                        , plyData2{linkIndex+1}.vertex.blue]/255;
                    h_UR3.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_2
                    disp(ME_2);
                    continue;
                end
            end
            
            % colour the environment/floor
            surf([-2,-2;2,2],[-2,2;-2,2],[0.001,0.001;0.01,0.001],'CData',imread('floor.jpg'),'FaceColor','texturemap');
        end
        
    end
end
