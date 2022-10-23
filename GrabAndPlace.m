%% Setup
clear;
clc;
%Creating the Figures
CheeseRobotFigure = figure('Name','Movement of Robot');
cheeseRobot = czRobot;
warning('off')

EstopFigure = figure('Name','Safety GUI');
% GUI Figures
handles.fig=figure(EstopFigure);
handles.pb1= uicontrol('style','pushbutton','position',[100 100 80 40],'callback',@ESTOP_cb,'string','ESTOP');
handles.pb2= uicontrol('style','pushbutton','position',[200 100 80 40],'callback',@Reset_cb,'string','Reset');
handles.pb3= uicontrol('style','pushbutton','position',[300 100 80 40],'callback',@Light_cb,'string','Lightcurtain');
guidata(handles.fig,handles)


% Variable to create dual stage lockout system
locker = 0;
Estop = 0;
reset = 0;

figure(CheeseRobotFigure)
% Collision Variables
% Object in interest a cube at location with side of disired
Obj_center_location = [-0.15,-0.15,.775];% In Chopping boardnear UR3
% Other Points
% Obj_center_location = [0.04,0.5,0.705];% in Table infront of IRB 
% Obj_center_location = [-0.18,0.5,0.775];% Base of IRB
% Obj_center_location = [-0.15,0.15,1.1];% hovering over Chopping board
% Obj_center_location = [-0.15,0.15,.775];% In Chopping board
% Obj_center_location = [-0.15,0.4,1.05];% Middle of IRB
% Obj_center_location = [0.185,-0.53,0.99];% Bread Basket
% Obj_center_location = [-0.2,-0.5,0.775];% Base of UR3
Obj_center_location = [2,2,2];% In middle of noware

[Y,Z] = meshgrid(-.1:0.01:.1,-.1:0.01:.1); % it is the 2 sides of the cube 
X = repmat(.1,size(Y,1),size(Y,2)); % number is for x direction of the cube


oneSideOfCube_h = surf(X,Y,Z);
cubePoints = [X(:),Y(:),Z(:)];
cubePoints = [ cubePoints ...
             ; cubePoints * rotz(pi/2)...
             ; cubePoints * rotz(pi) ...
             ; cubePoints * rotz(3*pi/2) ...
             ; cubePoints * roty(pi/2) ...
             ; cubePoints * roty(-pi/2)];   


%Spawning in the cube
cubePoints = cubePoints + repmat(Obj_center_location,size(cubePoints,1),1);    
cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');

% Collision Bubble specs
IRB_elipse_radius = [0.1,0.1,0.225;  0.1,0.1,0.22;  0.1,0.1,0.1;  0.2,0.1,0.1;   0.1,0.1,0.1;   0.1,0.1,0.1;   0.1,0.1,0.1;]; %the diameter of elipse for each joint
IRB_elipse_center = [0,0,0;  0,0,0;  0,0,0;  0,0,0;   0,0,0;   0,0,0;  0,0,0;] ;%the center offset of elipse for each joint
UR3_elipse_radius = [0.1,0.1,0.225;  0.1,0.1,0.1;  0.1,0.1,0.22;  0.2,0.1,0.1;   0.1,0.1,0.1;   0.1,0.1,0.1;   0.1,0.1,0.1;];%the diameter of elipse for each joint
UR3_elipse_center = [0,0,0;  0,0,0;  0,0,0;  0,0,0;   0,0,0;   0,0,0;  0,0,0;]; %the center offset of elipse for each joint


%% Movement of the Robot
% UR3 Get to the 1st bread
% use q = ans.UR3.getpos() to find q
ori_q = cheeseRobot.UR3.getpos();
ori_q = [0 0 0 0 0 0];
% set steps = 50 ez for demonstration
steps = 50;
% get joint_q by using the ikine
targ_q = cheeseRobot.UR3.ikine(transl([0.185,-0.53,0.99]) * troty(pi) * trotz(pi));
% create the jtraj
bot_bread = jtraj(ori_q, targ_q, steps);

% Collision
if MatrixCollisonDetection(bot_bread,cheeseRobot.UR3,UR3_elipse_radius, UR3_elipse_center,cubePoints) == 1
    locker = 1
    disp('COLLISION')
end

% start the animation
for i = 1:steps

    if Estop == 1
        locker = 1
        Estop = 0;
        reset = 0;
    end
    while locker == 1
        if reset == 1;
            if Estop == 1
                locker = 0;
                Estop = 0;
                reset = 0;
                disp('Unlocked')
            end
        else
            Estop = 0;
        end

         disp('Locked')
         pause (1);
    end






    G_T_cur = cheeseRobot.UR3.fkine(cheeseRobot.UR3.getpos()); % find the ur3's current matrix
    % Gripper base
    cheeseRobot.gp_base.basePose = G_T_cur;
    updatedPoints = [cheeseRobot.gp_base.basePose * [cheeseRobot.gp_base.baseVerts,ones(cheeseRobot.gp_base.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_base1.Vertices = updatedPoints(:,1:3); % updated the gripper base's location
    % Gripper finger 1
    rotateTRx = makehgtform('xrotate',(pi/2));
    cheeseRobot.gp_fg1.basePose = G_T_cur * rotateTRx;
    updatedPoints = [cheeseRobot.gp_fg1.basePose * [cheeseRobot.gp_fg1.baseVerts,ones(cheeseRobot.gp_fg1.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg1s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % Gripper finger 2
    rotateTRx = makehgtform('xrotate',(pi/2));
    rotateTRy = makehgtform('yrotate',(2*pi/3));
    cheeseRobot.gp_fg2.basePose = G_T_cur * rotateTRx * rotateTRy;
    updatedPoints = [cheeseRobot.gp_fg2.basePose * [cheeseRobot.gp_fg2.baseVerts,ones(cheeseRobot.gp_fg2.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg2s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 2's location
    % Gripper finger 3
    rotateTRx = makehgtform('xrotate',(pi/2));
    rotateTRy = makehgtform('yrotate',(-2*pi/3));
    cheeseRobot.gp_fg3.basePose = G_T_cur * rotateTRx * rotateTRy;
    updatedPoints = [cheeseRobot.gp_fg3.basePose * [cheeseRobot.gp_fg3.baseVerts,ones(cheeseRobot.gp_fg3.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg3s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % robot's action
    cheeseRobot.UR3.animate(bot_bread(i,:));
    drawnow();
end


% Gripper Grab -> Gripper finger 1
% UR3 Place the 1st bread
% Place 1st Bread motion Have to pick the bread up alit bit more
% use q = ans.UR3.getpos() to find current q
ori_q = cheeseRobot.UR3.getpos();
% set joint_q -> its just pull the bread up
targ_q = ori_q; targ_q(2:4) = [-0.0202 -0.0703 1.7]; % -> only joint2 3 4 moves => Just "pull" the bread up
bot_bread = jtraj(ori_q, targ_q, steps);

if MatrixCollisonDetection(bot_bread,cheeseRobot.UR3,UR3_elipse_radius, UR3_elipse_center,cubePoints) == 1
    locker = 1
    disp('COLLISION')
end


for i = 1:steps
    
    if Estop == 1
        locker = 1
        Estop = 0;
        reset = 0;
    end
    while locker == 1
        if reset == 1;

            if Estop == 1

                locker = 0;
                Estop = 0;
                reset = 0;
                disp('Unlocked')
            end
        else
            Estop = 0;
        end

         disp('Locked')
         pause (1);
    end



    G_T_cur = cheeseRobot.UR3.fkine(cheeseRobot.UR3.getpos()); % find the ur3's current matrix
    % Gripper base
    cheeseRobot.gp_base.basePose = G_T_cur;
    updatedPoints = [cheeseRobot.gp_base.basePose * [cheeseRobot.gp_base.baseVerts,ones(cheeseRobot.gp_base.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_base1.Vertices = updatedPoints(:,1:3); % updated the gripper base's location
    % Gripper Grab -> Gripper finger 1
    rotateTRx = makehgtform('xrotate',(pi/2.5));
    cheeseRobot.gp_fg1.basePose = G_T_cur * rotateTRx;
    updatedPoints = [cheeseRobot.gp_fg1.basePose * [cheeseRobot.gp_fg1.baseVerts,ones(cheeseRobot.gp_fg1.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg1s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % Gripper finger 2
    cheeseRobot.gp_fg2.basePose = G_T_cur * rpy2tr(295,55,158,'deg');
    updatedPoints = [cheeseRobot.gp_fg2.basePose * [cheeseRobot.gp_fg2.baseVerts,ones(cheeseRobot.gp_fg2.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg2s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 2's location
    % Gripper finger 3
    cheeseRobot.gp_fg3.basePose = G_T_cur * rpy2tr(290,300,200,'deg');
    updatedPoints = [cheeseRobot.gp_fg3.basePose * [cheeseRobot.gp_fg3.baseVerts,ones(cheeseRobot.gp_fg3.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg3s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    %bread1
    br_cur = G_T_cur; br_cur(3,end) = br_cur(3,end) - 0.12; % 1-0.12=0.88
    rotateTRy = makehgtform('yrotate',(pi/2));
    rotateTRx = makehgtform('xrotate',(pi));
    cheeseRobot.bread1.basePose = br_cur * rotateTRy * rotateTRx;
    updatedPoints = [cheeseRobot.bread1.basePose * [cheeseRobot.bread1.baseVerts,ones(cheeseRobot.bread1.VertexCount,1)]']'; % get the new position
    cheeseRobot.bread1s.Vertices = updatedPoints(:,1:3); % updated the bread1's location
    % robot's action
    cheeseRobot.UR3.animate(bot_bread(i,:));
    drawnow();
end



% NOW place the bread  - Gripper close!!!
ori_q = cheeseRobot.UR3.getpos(); % use q = ans.UR3.getpos() to find current q
targ_q = cheeseRobot.UR3.ikine(transl([-0.15,-0.1,1]) * troty(-pi));
bot_bread = jtraj(ori_q, targ_q, steps);
% Collision
if MatrixCollisonDetection(bot_bread,cheeseRobot.UR3,UR3_elipse_radius, UR3_elipse_center,cubePoints) == 1
    locker = 1
    disp('COLLISION')
end

for i = 1:steps
    if Estop == 1
        locker = 1
        Estop = 0;
        reset = 0;
    end
    while locker == 1
        if reset == 1;

            if Estop == 1

                locker = 0;
                Estop = 0;
                reset = 0;
                disp('Unlocked')
            end
        else
            Estop = 0;
        end

         disp('Locked')
         pause (1);
    end
    G_T_cur = cheeseRobot.UR3.fkine(cheeseRobot.UR3.getpos()); % find the ur3's current matrix
    % Gripper base
    cheeseRobot.gp_base.basePose = G_T_cur;
    updatedPoints = [cheeseRobot.gp_base.basePose * [cheeseRobot.gp_base.baseVerts,ones(cheeseRobot.gp_base.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_base1.Vertices = updatedPoints(:,1:3); % updated the gripper base's location
    % Gripper Grab -> Gripper finger 1
    rotateTRx = makehgtform('xrotate',(pi/2.5));
    cheeseRobot.gp_fg1.basePose = G_T_cur * rotateTRx;
    updatedPoints = [cheeseRobot.gp_fg1.basePose * [cheeseRobot.gp_fg1.baseVerts,ones(cheeseRobot.gp_fg1.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg1s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % Gripper finger 2
    cheeseRobot.gp_fg2.basePose = G_T_cur * rpy2tr(295,55,158,'deg');
    updatedPoints = [cheeseRobot.gp_fg2.basePose * [cheeseRobot.gp_fg2.baseVerts,ones(cheeseRobot.gp_fg2.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg2s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 2's location
    % Gripper finger 3
    cheeseRobot.gp_fg3.basePose = G_T_cur * rpy2tr(290,300,200,'deg');
    updatedPoints = [cheeseRobot.gp_fg3.basePose * [cheeseRobot.gp_fg3.baseVerts,ones(cheeseRobot.gp_fg3.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg3s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    %bread1
    br_cur = G_T_cur; br_cur(3,end) = br_cur(3,end) - 0.12; % 1-0.12=0.88
    rotateTRy = makehgtform('yrotate',(pi/2));
    rotateTRx = makehgtform('xrotate',(pi));
    cheeseRobot.bread1.basePose = br_cur * rotateTRy * rotateTRx;
    updatedPoints = [cheeseRobot.bread1.basePose * [cheeseRobot.bread1.baseVerts,ones(cheeseRobot.bread1.VertexCount,1)]']'; % get the new position
    cheeseRobot.bread1s.Vertices = updatedPoints(:,1:3); % updated the bread1's location
    % robot's action
    cheeseRobot.UR3.animate(bot_bread(i,:));
    drawnow();
end

% Get to the cheese block
% use q = ans.UR3.getpos() to find current q
ori_q = cheeseRobot.UR3.getpos();

targ_q = cheeseRobot.UR3.ikine(transl([0.275,-0.685,0.95]) * troty(pi));

cheese_block = jtraj(ori_q, targ_q, steps);
% Collision
if MatrixCollisonDetection(bot_bread,cheeseRobot.UR3,UR3_elipse_radius, UR3_elipse_center,cubePoints) == 1
    locker = 1
    disp('COLLISION')
end

for i = 1:steps
    if Estop == 1
        locker = 1
        Estop = 0;
        reset = 0;
    end
    while locker == 1
        if reset == 1;

            if Estop == 1

                locker = 0;
                Estop = 0;
                reset = 0;
                disp('Unlocked')
            end
        else
            Estop = 0;
        end

         disp('Locked')
         pause (1);
    end


    % bread drop
    if i < 10
        br_cur(3,end) = br_cur(3,end) - 0.008; % 0.88-0.08=0.8
        rotateTRx = makehgtform('xrotate',(pi));
        rotateTRy = makehgtform('yrotate',(pi));
        if i == 10, cheeseRobot.bread1.basePose = br_cur * rotateTRx * rotateTRy;
        else cheeseRobot.bread1.basePose = br_cur;end
        updatedPoints = [cheeseRobot.bread1.basePose * [cheeseRobot.bread1.baseVerts,ones(cheeseRobot.bread1.VertexCount,1)]']'; % get the new position
        cheeseRobot.bread1s.Vertices = updatedPoints(:,1:3); % updated the bread1's location
    end
    G_T_cur = cheeseRobot.UR3.fkine(cheeseRobot.UR3.getpos()); % find the ur3's current matrix
    % Gripper base
    cheeseRobot.gp_base.basePose = G_T_cur;
    updatedPoints = [cheeseRobot.gp_base.basePose * [cheeseRobot.gp_base.baseVerts,ones(cheeseRobot.gp_base.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_base1.Vertices = updatedPoints(:,1:3); % updated the gripper base's location
    % Gripper Grab -> Gripper finger 1 OVEROPEN
    cheeseRobot.gp_fg1.basePose = G_T_cur * rpy2tr(110,0,0,'deg');
    updatedPoints = [cheeseRobot.gp_fg1.basePose * [cheeseRobot.gp_fg1.baseVerts,ones(cheeseRobot.gp_fg1.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg1s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % Gripper finger 2
    cheeseRobot.gp_fg2.basePose = G_T_cur * rpy2tr(240,45,200,'deg');
    updatedPoints = [cheeseRobot.gp_fg2.basePose * [cheeseRobot.gp_fg2.baseVerts,ones(cheeseRobot.gp_fg2.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg2s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 2's location
    % Gripper finger 3
    cheeseRobot.gp_fg3.basePose = G_T_cur * rpy2tr(50,240,325,'deg');
    updatedPoints = [cheeseRobot.gp_fg3.basePose * [cheeseRobot.gp_fg3.baseVerts,ones(cheeseRobot.gp_fg3.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg3s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    
    % robot's action
    cheeseRobot.UR3.animate(cheese_block(i,:));
    drawnow();
end


% Place the cheese block & cheese model change
% Place the cheese block motion Have to pick the the cheese block up alit bit more
% use q = ans.UR3.getpos() to find current q
ori_q = cheeseRobot.UR3.getpos();
% set joint_q -> its just pull the cheese up
targ_q = ori_q; targ_q(2) = -0.4803; targ_q(4) = 1.4034; % -> only joint2 4 moves => Just "pull" the cheese up

cheese_block = jtraj(ori_q, targ_q, steps);
% Collision
if MatrixCollisonDetection(bot_bread,cheeseRobot.UR3,UR3_elipse_radius, UR3_elipse_center,cubePoints) == 1
    locker = 1
    disp('COLLISION')
end

for i = 1:steps
    if Estop == 1
        locker = 1
        Estop = 0;
        reset = 0;
    end
    while locker == 1
        if reset == 1;

            if Estop == 1

                locker = 0;
                Estop = 0;
                reset = 0;
                disp('Unlocked')
            end
        else
            Estop = 0;
        end

         disp('Locked')
         pause (1);
    end


    G_T_cur = cheeseRobot.UR3.fkine(cheeseRobot.UR3.getpos()); % find the ur3's current matrix
    % Gripper base
    cheeseRobot.gp_base.basePose = G_T_cur;
    updatedPoints = [cheeseRobot.gp_base.basePose * [cheeseRobot.gp_base.baseVerts,ones(cheeseRobot.gp_base.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_base1.Vertices = updatedPoints(:,1:3); % updated the gripper base's location
    % Gripper Grab -> Gripper finger 1  bit_OPEN
    cheeseRobot.gp_fg1.basePose = G_T_cur * rpy2tr(100,0,0,'deg');
    updatedPoints = [cheeseRobot.gp_fg1.basePose * [cheeseRobot.gp_fg1.baseVerts,ones(cheeseRobot.gp_fg1.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg1s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % Gripper finger 2
    cheeseRobot.gp_fg2.basePose = G_T_cur * rpy2tr(260,45,200,'deg');
    updatedPoints = [cheeseRobot.gp_fg2.basePose * [cheeseRobot.gp_fg2.baseVerts,ones(cheeseRobot.gp_fg2.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg2s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 2's location
    % Gripper finger 3
    cheeseRobot.gp_fg3.basePose = G_T_cur * rpy2tr(40,260,310,'deg');
    updatedPoints = [cheeseRobot.gp_fg3.basePose * [cheeseRobot.gp_fg3.baseVerts,ones(cheeseRobot.gp_fg3.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg3s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % cheese block
    cb_cur = G_T_cur; cb_cur(3,end) = cb_cur(3,end) - 0.08; % pick the c_block a little bit more 0.97-0.08=0.89
    cheeseRobot.c_block.basePose = cb_cur;
    updatedPoints = [cheeseRobot.c_block.basePose * [cheeseRobot.c_block.baseVerts,ones(cheeseRobot.c_block.VertexCount,1)]']'; % get the new position
    cheeseRobot.c_block1.Vertices = updatedPoints(:,1:3); % updated the c_block's location
    % robot's action
    cheeseRobot.UR3.animate(cheese_block(i,:));
    drawnow();
end

% NOW place the cheese block - Gripper close!!! -> but for cheese block
% should be over-open! because its 'huge'
ori_q = cheeseRobot.UR3.getpos(); % use q = ans.UR3.getpos() to find current q

targ_q = cheeseRobot.UR3.ikine(transl([-0.153,0.0422,0.9076]) * trotx(-pi));

cheese_block = jtraj(ori_q, targ_q, steps);
% Collision
if MatrixCollisonDetection(bot_bread,cheeseRobot.UR3,UR3_elipse_radius, UR3_elipse_center,cubePoints) == 1
    locker = 1
    disp('COLLISION')
end

for i = 1:steps
    if Estop == 1
        locker = 1
        Estop = 0;
        reset = 0;
    end
    while locker == 1
        if reset == 1;

            if Estop == 1

                locker = 0;
                Estop = 0;
                reset = 0;
                disp('Unlocked')
            end
        else
            Estop = 0;
        end

         disp('Locked')
         pause (1);
    end


    G_T_cur = cheeseRobot.UR3.fkine(cheeseRobot.UR3.getpos()); % find the ur3's current matrix
    % Gripper base
    cheeseRobot.gp_base.basePose = G_T_cur;
    updatedPoints = [cheeseRobot.gp_base.basePose * [cheeseRobot.gp_base.baseVerts,ones(cheeseRobot.gp_base.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_base1.Vertices = updatedPoints(:,1:3); % updated the gripper base's location
    % Gripper Grab -> Gripper finger 1  bit_OPEN
    cheeseRobot.gp_fg1.basePose = G_T_cur * rpy2tr(100,0,0,'deg');
    updatedPoints = [cheeseRobot.gp_fg1.basePose * [cheeseRobot.gp_fg1.baseVerts,ones(cheeseRobot.gp_fg1.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg1s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % Gripper finger 2
    cheeseRobot.gp_fg2.basePose = G_T_cur * rpy2tr(260,45,200,'deg');
    updatedPoints = [cheeseRobot.gp_fg2.basePose * [cheeseRobot.gp_fg2.baseVerts,ones(cheeseRobot.gp_fg2.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg2s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 2's location
    % Gripper finger 3
    cheeseRobot.gp_fg3.basePose = G_T_cur * rpy2tr(40,260,310,'deg');
    updatedPoints = [cheeseRobot.gp_fg3.basePose * [cheeseRobot.gp_fg3.baseVerts,ones(cheeseRobot.gp_fg3.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg3s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % c_block
    cb_cur = G_T_cur; cb_cur(3,end) = cb_cur(3,end) - 0.08; % pick the c_block a little bit more 0.97-0.08=0.89
    cheeseRobot.c_block.basePose = cb_cur;
    updatedPoints = [cheeseRobot.c_block.basePose * [cheeseRobot.c_block.baseVerts,ones(cheeseRobot.c_block.VertexCount,1)]']'; % get the new position
    cheeseRobot.c_block1.Vertices = updatedPoints(:,1:3); % updated the c_block's location
    % replace the cheese model
    if i == 50
        % remove c_block
        cb_cur = G_T_cur; cb_cur(1,end) = 0; cb_cur(2,end) = 0;cb_cur(3,end) = 0.5; % pick the c_block a little bit more 0.97-0.08=0.89
        cheeseRobot.c_block.basePose = cb_cur;
        updatedPoints = [cheeseRobot.c_block.basePose * [cheeseRobot.c_block.baseVerts,ones(cheeseRobot.c_block.VertexCount,1)]']'; % get the new position
        cheeseRobot.c_block1.Vertices = updatedPoints(:,1:3); % updated the c_block's location
        % replace with c_block2 and c_slice
        cb_cur = G_T_cur; cb_cur(3,end) = cb_cur(3,end) - 0.08; % pick the c_block a little bit more 0.97-0.08=0.89
        cheeseRobot.c_blocks.basePose = cb_cur;
        updatedPoints = [cheeseRobot.c_blocks.basePose * [cheeseRobot.c_blocks.baseVerts,ones(cheeseRobot.c_blocks.VertexCount,1)]']'; % get the new position
        cheeseRobot.c_block2.Vertices = updatedPoints(:,1:3); % updated the c_block's location
        % c_slice
        cb_cur = G_T_cur; cb_cur(1,end) = cb_cur(1,end) + 0.08; cb_cur(3,end) = cb_cur(3,end) - 0.08; % pick the c_block a little bit more 0.97-0.08=0.89
        cheeseRobot.c_slice.basePose = cb_cur;
        updatedPoints = [cheeseRobot.c_slice.basePose * [cheeseRobot.c_slice.baseVerts,ones(cheeseRobot.c_slice.VertexCount,1)]']'; % get the new position
        cheeseRobot.c_slice1.Vertices = updatedPoints(:,1:3); % updated the c_block's location
    end
    % robot's action
    cheeseRobot.UR3.animate(cheese_block(i,:));
    drawnow();
end

% IRB Cut the cheese block
% Firstly, have to move the knife onto the cheese & collision detection then use q = ans.UR3/IRB.getpos() to find current q
ori_q_IRB = cheeseRobot.IRB.getpos();
ori_q_UR3 = cheeseRobot.UR3.getpos();

targ_q_IRB = cheeseRobot.IRB.ikine(transl([-0.0857,0.0484,0.9635]) * trotz(pi/2));
targ_q_UR3 = ori_q_UR3; targ_q_UR3(1) = 1.8552; % -> only joint1 moves 106degs => deg2rad(106)

cut_cheese = jtraj(ori_q_IRB, targ_q_IRB, steps);
avoid_colli = jtraj(ori_q_UR3, targ_q_UR3, steps); % avoid collision -> UR3 & IRB
% Collision
if MatrixCollisonDetection(bot_bread,cheeseRobot.IRB,IRB_elipse_radius, IRB_elipse_center,cubePoints) == 1
    locker = 1
    disp('COLLISION')
end
if MatrixCollisonDetection(bot_bread,cheeseRobot.UR3,UR3_elipse_radius, UR3_elipse_center,cubePoints) == 1
    locker = 1
    disp('COLLISION')
end


for i = 1:steps
    if Estop == 1
        locker = 1
        Estop = 0;
        reset = 0;
    end
    while locker == 1
        if reset == 1;

            if Estop == 1

                locker = 0;
                Estop = 0;
                reset = 0;
                disp('Unlocked')
            end
        else
            Estop = 0;
        end

         disp('Locked')
         pause (1);
    end


    G_T_cur = cheeseRobot.UR3.fkine(cheeseRobot.UR3.getpos()); % find the ur3's current matrix
    % Gripper base
    cheeseRobot.gp_base.basePose = G_T_cur;
    updatedPoints = [cheeseRobot.gp_base.basePose * [cheeseRobot.gp_base.baseVerts,ones(cheeseRobot.gp_base.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_base1.Vertices = updatedPoints(:,1:3); % updated the gripper base's location
    % Gripper Grab -> Gripper finger 1
    rotateTRx = makehgtform('xrotate',(pi/2.5));
    cheeseRobot.gp_fg1.basePose = G_T_cur * rpy2tr(110,0,0,'deg');
    updatedPoints = [cheeseRobot.gp_fg1.basePose * [cheeseRobot.gp_fg1.baseVerts,ones(cheeseRobot.gp_fg1.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg1s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % Gripper finger 2
    cheeseRobot.gp_fg2.basePose = G_T_cur * rpy2tr(240,45,200,'deg');
    updatedPoints = [cheeseRobot.gp_fg2.basePose * [cheeseRobot.gp_fg2.baseVerts,ones(cheeseRobot.gp_fg2.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg2s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 2's location
    % Gripper finger 3
    cheeseRobot.gp_fg3.basePose = G_T_cur * rpy2tr(50,240,325,'deg');
    updatedPoints = [cheeseRobot.gp_fg3.basePose * [cheeseRobot.gp_fg3.baseVerts,ones(cheeseRobot.gp_fg3.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg3s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % robot's action
    cheeseRobot.UR3.animate(avoid_colli(i,:));
    cheeseRobot.IRB.animate(cut_cheese(i,:));
    drawnow();
end

% NOW can cut the cheese block 0.0011269/0.159994=0.7%
% use q = ans.IRB.getpos() to find current q
ori_q_IRB = cheeseRobot.IRB.getpos();

targ_q_IRB = cheeseRobot.IRB.ikine(transl([-0.0829,0.035,0.9232]) * trotz(pi/2));

cut_cheese = jtraj(ori_q_IRB, targ_q_IRB, steps);
% Collision
if MatrixCollisonDetection(bot_bread,cheeseRobot.IRB,IRB_elipse_radius, IRB_elipse_center,cubePoints) == 1
    locker = 1
    disp('COLLISION')
end


for i = 1:steps
    if Estop == 1
        locker = 1
        Estop = 0;
        reset = 0;
    end
    while locker == 1
        if reset == 1;

            if Estop == 1

                locker = 0;
                Estop = 0;
                reset = 0;
                disp('Unlocked')
            end
        else
            Estop = 0;
        end

         disp('Locked')
         pause (1);
    end


    % robot's action
    cheeseRobot.IRB.animate(cut_cheese(i,:));
    drawnow();
    if i == 50
        % cut the cheese completely
        ori_q_IRB = cheeseRobot.IRB.getpos();
        
        targ_q_IRB = ori_q_IRB; targ_q_IRB(5) = 0.8929; % -> only joint5 moves 51degs => deg2rad(51)
        
        cut_cheese = jtraj(ori_q_IRB, targ_q_IRB, steps);
        
        for j = 1:steps
            % robot's action
            cheeseRobot.IRB.animate(cut_cheese(j,:));
            drawnow();
            if j == 50
                % c_slice
                rotateTRy = makehgtform('yrotate',(pi/2));
                cheeseRobot.c_slice.basePose(1,end) = -0.045;cheeseRobot.c_slice.basePose(3,end) = 0.8;
                cheeseRobot.c_slice.basePose = cheeseRobot.c_slice.basePose * rotateTRy;
                updatedPoints = [cheeseRobot.c_slice.basePose * [cheeseRobot.c_slice.baseVerts,ones(cheeseRobot.c_slice.VertexCount,1)]']'; % get the new position
                cheeseRobot.c_slice1.Vertices = updatedPoints(:,1:3); % updated the c_slice's location
            end
        end
    end
end
% IRB reset & UR3 put cheese back
% use q = ans.IRB/UR3.getpos() to find current q
ori_q_IRB = cheeseRobot.IRB.getpos();
ori_q_UR3 = cheeseRobot.UR3.getpos();


targ_q_IRB = cheeseRobot.IRB.ikine(transl([0.115,0.5,1.184]) * trotz(pi/2));
targ_q_UR3 = ori_q_UR3; targ_q_UR3(1) = 1.7295; % -> only joint1 moves 106degs => deg2rad(106)


UR3_cz_back = jtraj(ori_q_UR3, targ_q_UR3, steps);  % UR3 put cheese back
IRB_reset = jtraj(ori_q_IRB, targ_q_IRB, steps); % IRB reset
% Collision
if MatrixCollisonDetection(bot_bread,cheeseRobot.UR3,UR3_elipse_radius, UR3_elipse_center,cubePoints) == 1
    locker = 1
    disp('COLLISION')
end

if MatrixCollisonDetection(bot_bread,cheeseRobot.IRB,IRB_elipse_radius, IRB_elipse_center,cubePoints) == 1
    locker = 1
    disp('COLLISION')
end



for i = 1:steps
    if Estop == 1
        locker = 1
        Estop = 0;
        reset = 0;
    end
    while locker == 1
        if reset == 1;

            if Estop == 1

                locker = 0;
                Estop = 0;
                reset = 0;
                disp('Unlocked')
            end
        else
            Estop = 0;
        end

         disp('Locked')
         pause (1);
    end


    G_T_cur = cheeseRobot.UR3.fkine(cheeseRobot.UR3.getpos()); % find the ur3's current matrix
    % Grab !!! Gripper base
    cheeseRobot.gp_base.basePose = G_T_cur;
    updatedPoints = [cheeseRobot.gp_base.basePose * [cheeseRobot.gp_base.baseVerts,ones(cheeseRobot.gp_base.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_base1.Vertices = updatedPoints(:,1:3); % updated the gripper base's location
    % Gripper Grab -> Gripper finger 1
    rotateTRx = makehgtform('xrotate',(pi/2.5));
    cheeseRobot.gp_fg1.basePose = G_T_cur * rpy2tr(110,0,0,'deg');
    updatedPoints = [cheeseRobot.gp_fg1.basePose * [cheeseRobot.gp_fg1.baseVerts,ones(cheeseRobot.gp_fg1.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg1s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % Gripper finger 2
    cheeseRobot.gp_fg2.basePose = G_T_cur * rpy2tr(240,45,200,'deg');
    updatedPoints = [cheeseRobot.gp_fg2.basePose * [cheeseRobot.gp_fg2.baseVerts,ones(cheeseRobot.gp_fg2.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg2s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 2's location
    % Gripper finger 3
    cheeseRobot.gp_fg3.basePose = G_T_cur * rpy2tr(50,240,325,'deg');
    updatedPoints = [cheeseRobot.gp_fg3.basePose * [cheeseRobot.gp_fg3.baseVerts,ones(cheeseRobot.gp_fg3.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg3s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % robot's action
    cheeseRobot.UR3.animate(UR3_cz_back(i,:));
    cheeseRobot.IRB.animate(IRB_reset(i,:));
    drawnow();
    % pick the cheese up
    if i == 50
        ori_q_UR3 = cheeseRobot.UR3.getpos();
        targ_q_UR3 = ori_q_UR3; targ_q_UR3(2) = -1.3439; targ_q_UR3(4) = 0.9957;% -> only joint2&4 moves
        UR3_cz_back = jtraj(ori_q_UR3, targ_q_UR3, steps);
        for j = 1:steps
            G_T_cur = cheeseRobot.UR3.fkine(cheeseRobot.UR3.getpos()); % find the ur3's current matrix
            % Grab !!! Gripper base
            cheeseRobot.gp_base.basePose = G_T_cur;
            updatedPoints = [cheeseRobot.gp_base.basePose * [cheeseRobot.gp_base.baseVerts,ones(cheeseRobot.gp_base.VertexCount,1)]']'; % get the new position
            cheeseRobot.gp_base1.Vertices = updatedPoints(:,1:3); % updated the gripper base's location
            % Gripper Grab -> Gripper finger 1  bit_OPEN
            cheeseRobot.gp_fg1.basePose = G_T_cur * rpy2tr(100,0,0,'deg');
            updatedPoints = [cheeseRobot.gp_fg1.basePose * [cheeseRobot.gp_fg1.baseVerts,ones(cheeseRobot.gp_fg1.VertexCount,1)]']'; % get the new position
            cheeseRobot.gp_fg1s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
            % Gripper finger 2
            cheeseRobot.gp_fg2.basePose = G_T_cur * rpy2tr(260,45,200,'deg');
            updatedPoints = [cheeseRobot.gp_fg2.basePose * [cheeseRobot.gp_fg2.baseVerts,ones(cheeseRobot.gp_fg2.VertexCount,1)]']'; % get the new position
            cheeseRobot.gp_fg2s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 2's location
            % Gripper finger 3
            cheeseRobot.gp_fg3.basePose = G_T_cur * rpy2tr(40,260,310,'deg');
            updatedPoints = [cheeseRobot.gp_fg3.basePose * [cheeseRobot.gp_fg3.baseVerts,ones(cheeseRobot.gp_fg3.VertexCount,1)]']'; % get the new position
            cheeseRobot.gp_fg3s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
            % c_block2
            cb_cur = G_T_cur; cb_cur(3,end) = cb_cur(3,end) - 0.08; % pick the c_block a little bit more 0.97-0.08=0.89
            cheeseRobot.c_blocks.basePose = cb_cur;
            updatedPoints = [cheeseRobot.c_blocks.basePose * [cheeseRobot.c_blocks.baseVerts,ones(cheeseRobot.c_blocks.VertexCount,1)]']'; % get the new position
            cheeseRobot.c_block2.Vertices = updatedPoints(:,1:3); % updated the c_block's location
            % robot's action
            cheeseRobot.UR3.animate(UR3_cz_back(j,:));
            drawnow();
            if j == 50 % put cheese back
                ori_q_UR3 = cheeseRobot.UR3.getpos();
                targ_q_UR3 = cheeseRobot.UR3.ikine(transl([0.2059,-0.6742,1.0865]) * troty(pi)  * trotx(pi/180));
                UR3_cz_back = jtraj(ori_q_UR3, targ_q_UR3, steps);
                for m = 1:steps
                    G_T_cur = cheeseRobot.UR3.fkine(cheeseRobot.UR3.getpos()); % find the ur3's current matrix
                    % Grab !!! Gripper base
                    cheeseRobot.gp_base.basePose = G_T_cur;
                    updatedPoints = [cheeseRobot.gp_base.basePose * [cheeseRobot.gp_base.baseVerts,ones(cheeseRobot.gp_base.VertexCount,1)]']'; % get the new position
                    cheeseRobot.gp_base1.Vertices = updatedPoints(:,1:3); % updated the gripper base's location
                    % Gripper Grab -> Gripper finger 1  bit_OPEN
                    cheeseRobot.gp_fg1.basePose = G_T_cur * rpy2tr(100,0,0,'deg');
                    updatedPoints = [cheeseRobot.gp_fg1.basePose * [cheeseRobot.gp_fg1.baseVerts,ones(cheeseRobot.gp_fg1.VertexCount,1)]']'; % get the new position
                    cheeseRobot.gp_fg1s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
                    % Gripper finger 2
                    cheeseRobot.gp_fg2.basePose = G_T_cur * rpy2tr(260,45,200,'deg');
                    updatedPoints = [cheeseRobot.gp_fg2.basePose * [cheeseRobot.gp_fg2.baseVerts,ones(cheeseRobot.gp_fg2.VertexCount,1)]']'; % get the new position
                    cheeseRobot.gp_fg2s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 2's location
                    % Gripper finger 3
                    cheeseRobot.gp_fg3.basePose = G_T_cur * rpy2tr(40,260,310,'deg');
                    updatedPoints = [cheeseRobot.gp_fg3.basePose * [cheeseRobot.gp_fg3.baseVerts,ones(cheeseRobot.gp_fg3.VertexCount,1)]']'; % get the new position
                    cheeseRobot.gp_fg3s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
                    % c_block2
                    cb_cur = G_T_cur; cb_cur(3,end) = cb_cur(3,end) - 0.08; % pick the c_block a little bit more 0.97-0.08=0.89
                    cheeseRobot.c_blocks.basePose = cb_cur;
                    updatedPoints = [cheeseRobot.c_blocks.basePose * [cheeseRobot.c_blocks.baseVerts,ones(cheeseRobot.c_blocks.VertexCount,1)]']'; % get the new position
                    cheeseRobot.c_block2.Vertices = updatedPoints(:,1:3); % updated the c_block's location
                    % robot's action
                    cheeseRobot.UR3.animate(UR3_cz_back(m,:));
                    drawnow();
                    if m == 50 % put cheese back
                        ori_q_UR3 = cheeseRobot.UR3.getpos();
                        targ_q_UR3 = ori_q_UR3; targ_q_UR3(2) = -1.7453; targ_q_UR3(4) = 0.7896; % -> only joint2 4 moves => Just "put" the cheese down
                        UR3_cz_back = jtraj(ori_q_UR3, targ_q_UR3, steps);
                        for n = 1:steps
                            G_T_cur = cheeseRobot.UR3.fkine(cheeseRobot.UR3.getpos()); % find the ur3's current matrix
                            % Grab !!! Gripper base
                            cheeseRobot.gp_base.basePose = G_T_cur;
                            updatedPoints = [cheeseRobot.gp_base.basePose * [cheeseRobot.gp_base.baseVerts,ones(cheeseRobot.gp_base.VertexCount,1)]']'; % get the new position
                            cheeseRobot.gp_base1.Vertices = updatedPoints(:,1:3); % updated the gripper base's location
                            % Gripper Grab -> Gripper finger 1  bit_OPEN
                            cheeseRobot.gp_fg1.basePose = G_T_cur * rpy2tr(100,0,0,'deg');
                            updatedPoints = [cheeseRobot.gp_fg1.basePose * [cheeseRobot.gp_fg1.baseVerts,ones(cheeseRobot.gp_fg1.VertexCount,1)]']'; % get the new position
                            cheeseRobot.gp_fg1s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
                            % Gripper finger 2
                            cheeseRobot.gp_fg2.basePose = G_T_cur * rpy2tr(260,45,200,'deg');
                            updatedPoints = [cheeseRobot.gp_fg2.basePose * [cheeseRobot.gp_fg2.baseVerts,ones(cheeseRobot.gp_fg2.VertexCount,1)]']'; % get the new position
                            cheeseRobot.gp_fg2s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 2's location
                            % Gripper finger 3
                            cheeseRobot.gp_fg3.basePose = G_T_cur * rpy2tr(40,260,310,'deg');
                            updatedPoints = [cheeseRobot.gp_fg3.basePose * [cheeseRobot.gp_fg3.baseVerts,ones(cheeseRobot.gp_fg3.VertexCount,1)]']'; % get the new position
                            cheeseRobot.gp_fg3s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
                            % c_block2
                            cb_cur = G_T_cur; cb_cur(3,end) = cb_cur(3,end) - 0.08; % pick the c_block a little bit more 0.97-0.08=0.89
                            cheeseRobot.c_blocks.basePose = cb_cur;
                            updatedPoints = [cheeseRobot.c_blocks.basePose * [cheeseRobot.c_blocks.baseVerts,ones(cheeseRobot.c_blocks.VertexCount,1)]']'; % get the new position
                            cheeseRobot.c_block2.Vertices = updatedPoints(:,1:3); % updated the c_block's location
                            % robot's action
                            cheeseRobot.UR3.animate(UR3_cz_back(n,:));
                            drawnow();
                        end
                    end
                end
            end
        end
    end
end
% UR3 get to the cheese slice
% use q = ans.UR3.getpos() to find current q
ori_q = cheeseRobot.UR3.getpos();

targ_q = ori_q; targ_q(1) = 1.4661;targ_q(2) = -1.591;targ_q(3) = 1.6921;targ_q(4) = 1.5436;
% or targ_q = cheeseRobot.UR3.ikine(transl([-0.0095 0.0328 0.8581]) * rpy2tr(-180,0,0,'deg') );

get_slice = jtraj(ori_q, targ_q, steps);
% Collision
if MatrixCollisonDetection(bot_bread,cheeseRobot.UR3,UR3_elipse_radius, UR3_elipse_center,cubePoints) == 1
    locker = 1
    disp('COLLISION')
end

for i = 1:steps
    if Estop == 1
        locker = 1
        Estop = 0;
        reset = 0;
    end
    while locker == 1
        if reset == 1;

            if Estop == 1

                locker = 0;
                Estop = 0;
                reset = 0;
                disp('Unlocked')
            end
        else
            Estop = 0;
        end

         disp('Locked')
         pause (1);
    end


    G_T_cur = cheeseRobot.UR3.fkine(cheeseRobot.UR3.getpos()); % find the ur3's current matrix
    % Gripper base
    cheeseRobot.gp_base.basePose = G_T_cur;
    updatedPoints = [cheeseRobot.gp_base.basePose * [cheeseRobot.gp_base.baseVerts,ones(cheeseRobot.gp_base.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_base1.Vertices = updatedPoints(:,1:3); % updated the gripper base's location
    % Gripper Grab -> Gripper finger 1
    rotateTRx = makehgtform('xrotate',(pi/2));
    cheeseRobot.gp_fg1.basePose = G_T_cur * rotateTRx;
    updatedPoints = [cheeseRobot.gp_fg1.basePose * [cheeseRobot.gp_fg1.baseVerts,ones(cheeseRobot.gp_fg1.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg1s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % Gripper finger 2
    rotateTRx = makehgtform('xrotate',(pi/2));
    rotateTRy = makehgtform('yrotate',(2*pi/3));
    cheeseRobot.gp_fg2.basePose = G_T_cur * rotateTRx * rotateTRy;
    updatedPoints = [cheeseRobot.gp_fg2.basePose * [cheeseRobot.gp_fg2.baseVerts,ones(cheeseRobot.gp_fg2.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg2s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 2's location
    % Gripper finger 3
    rotateTRx = makehgtform('xrotate',(pi/2));
    rotateTRy = makehgtform('yrotate',(-2*pi/3));
    cheeseRobot.gp_fg3.basePose = G_T_cur * rotateTRx * rotateTRy;
    updatedPoints = [cheeseRobot.gp_fg3.basePose * [cheeseRobot.gp_fg3.baseVerts,ones(cheeseRobot.gp_fg3.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg3s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % robot's action
    cheeseRobot.UR3.animate(get_slice(i,:));
    drawnow();
end
% UR3 place the cheese slice onto the 1st bread
% use q = ans.UR3.getpos() to find current q
ori_q = cheeseRobot.UR3.getpos();
%ori_q = [ 1.4661   -1.5910    1.6921    1.5436   -1.5880    1.4192];

%targ_q = cheeseRobot.UR3.ikine(transl([-0.1561 -0.0971 0.9193]) * rpy2tr(-0.3,-0.939,-68.32,'deg'));
% x=atan2(r32,r33);y=atan2(r21,r11);z=atan2(-r31,(r11/cos(y))); but its not working!!!
targ_q = [1.7977   -0.5516   -0.0646    2.1886   -1.5880    1.4192];

get_slice = jtraj(ori_q, targ_q, steps);
% Collision
if MatrixCollisonDetection(bot_bread,cheeseRobot.UR3,UR3_elipse_radius, UR3_elipse_center,cubePoints) == 1
    locker = 1
    disp('COLLISION')
end

for i = 1:steps
    if Estop == 1
        locker = 1
        Estop = 0;
        reset = 0;
    end
    while locker == 1
        if reset == 1;

            if Estop == 1

                locker = 0;
                Estop = 0;
                reset = 0;
                disp('Unlocked')
            end
        else
            Estop = 0;
        end

         disp('Locked')
         pause (1);
    end


    G_T_cur = cheeseRobot.UR3.fkine(cheeseRobot.UR3.getpos()); % find the ur3's current matrix
    % Gripper base
    cheeseRobot.gp_base.basePose = G_T_cur;
    updatedPoints = [cheeseRobot.gp_base.basePose * [cheeseRobot.gp_base.baseVerts,ones(cheeseRobot.gp_base.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_base1.Vertices = updatedPoints(:,1:3); % updated the gripper base's location
    % Gripper Grab -> Gripper finger 1
    rotateTRx = makehgtform('xrotate',(pi/2.5));
    cheeseRobot.gp_fg1.basePose = G_T_cur * rotateTRx;
    updatedPoints = [cheeseRobot.gp_fg1.basePose * [cheeseRobot.gp_fg1.baseVerts,ones(cheeseRobot.gp_fg1.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg1s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % Gripper finger 2
    cheeseRobot.gp_fg2.basePose = G_T_cur * rpy2tr(295,55,158,'deg');
    updatedPoints = [cheeseRobot.gp_fg2.basePose * [cheeseRobot.gp_fg2.baseVerts,ones(cheeseRobot.gp_fg2.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg2s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 2's location
    % Gripper finger 3
    cheeseRobot.gp_fg3.basePose = G_T_cur * rpy2tr(290,300,200,'deg');
    updatedPoints = [cheeseRobot.gp_fg3.basePose * [cheeseRobot.gp_fg3.baseVerts,ones(cheeseRobot.gp_fg3.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg3s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % c_slice
    rotateTRy = makehgtform('yrotate',(-pi/2));
    cheeseRobot.c_slice.basePose = G_T_cur;cheeseRobot.c_slice.basePose(3,end) = cheeseRobot.c_slice.basePose(3,end) - 0.05;
    if i > 20
        cheeseRobot.c_slice.basePose(3,end) = cheeseRobot.c_slice.basePose(3,end) - 0.03;
        cheeseRobot.c_slice.basePose = cheeseRobot.c_slice.basePose;
    else
        cheeseRobot.c_slice.basePose(1,end) = cheeseRobot.c_slice.basePose(1,end) - 0.04;
        cheeseRobot.c_slice.basePose = cheeseRobot.c_slice.basePose * rotateTRy;
    end
    updatedPoints = [cheeseRobot.c_slice.basePose * [cheeseRobot.c_slice.baseVerts,ones(cheeseRobot.c_slice.VertexCount,1)]']'; % get the new position
    cheeseRobot.c_slice1.Vertices = updatedPoints(:,1:3); % updated the c_slice's location
    
    % c_slice drop
    if i == 50
        for j = 1:10
            cz_cur = G_T_cur;
            cz_cur(3,end) = cz_cur(3,end) - 0.1; % 0.88-0.08=0.8
            rotateTRy = makehgtform('yrotate',(pi/2));
            if j == 10, cheeseRobot.c_slice.basePose = cz_cur * rotateTRy;
            else cheeseRobot.c_slice.basePose = br_cur;end
            updatedPoints = [cheeseRobot.c_slice.basePose * [cheeseRobot.c_slice.baseVerts,ones(cheeseRobot.c_slice.VertexCount,1)]']'; % get the new position
            cheeseRobot.c_slice1.Vertices = updatedPoints(:,1:3); % updated the c_slice's location
        end
    end
    
    % robot's action
    cheeseRobot.UR3.animate(get_slice(i,:));
    drawnow();
end
% UR3 get to the last bread
% use q = ans.UR3.getpos() to find current q
ori_q = cheeseRobot.UR3.getpos();

targ_q = cheeseRobot.UR3.ikine(transl([0.2075,-0.53,0.99]) * rpy2tr(0,180,180,'deg'));

get_slice = jtraj(ori_q, targ_q, steps);
% Collision
if MatrixCollisonDetection(bot_bread,cheeseRobot.UR3,UR3_elipse_radius, UR3_elipse_center,cubePoints) == 1
    locker = 1
    disp('COLLISION')
end

for i = 1:steps

    if Estop == 1
        locker = 1
        Estop = 0;
        reset = 0;
    end
    while locker == 1
        if reset == 1;

            if Estop == 1

                locker = 0;
                Estop = 0;
                reset = 0;
                disp('Unlocked')
            end
        else
            Estop = 0;
        end

         disp('Locked')
         pause (1);
    end



    G_T_cur = cheeseRobot.UR3.fkine(cheeseRobot.UR3.getpos()); % find the ur3's current matrix
    % Gripper base
    cheeseRobot.gp_base.basePose = G_T_cur;
    updatedPoints = [cheeseRobot.gp_base.basePose * [cheeseRobot.gp_base.baseVerts,ones(cheeseRobot.gp_base.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_base1.Vertices = updatedPoints(:,1:3); % updated the gripper base's location
    % Gripper Grab -> Gripper finger 1
    rotateTRx = makehgtform('xrotate',(pi/2));
    cheeseRobot.gp_fg1.basePose = G_T_cur * rotateTRx;
    updatedPoints = [cheeseRobot.gp_fg1.basePose * [cheeseRobot.gp_fg1.baseVerts,ones(cheeseRobot.gp_fg1.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg1s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % Gripper finger 2
    rotateTRx = makehgtform('xrotate',(pi/2));
    rotateTRy = makehgtform('yrotate',(2*pi/3));
    cheeseRobot.gp_fg2.basePose = G_T_cur * rotateTRx * rotateTRy;
    updatedPoints = [cheeseRobot.gp_fg2.basePose * [cheeseRobot.gp_fg2.baseVerts,ones(cheeseRobot.gp_fg2.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg2s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 2's location
    % Gripper finger 3
    rotateTRx = makehgtform('xrotate',(pi/2));
    rotateTRy = makehgtform('yrotate',(-2*pi/3));
    cheeseRobot.gp_fg3.basePose = G_T_cur * rotateTRx * rotateTRy;
    updatedPoints = [cheeseRobot.gp_fg3.basePose * [cheeseRobot.gp_fg3.baseVerts,ones(cheeseRobot.gp_fg3.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg3s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % robot's action
    cheeseRobot.UR3.animate(get_slice(i,:));
    drawnow();
end
% UR3 place the last beard onto the cheese slice
% Place 1st Bread motion Have to pick the bread up alit bit more
% use q = ans.UR3.getpos() to find current q
ori_q = cheeseRobot.UR3.getpos();
% set joint_q -> its just pull the bread up
targ_q = ori_q; targ_q(2:4) = [0.1222 3.1416 1.4739]; % -> only joint2 3 4 moves => Just "pull" the bread up

top_bread = jtraj(ori_q, targ_q, steps);
% Collision
if MatrixCollisonDetection(bot_bread,cheeseRobot.UR3,UR3_elipse_radius, UR3_elipse_center,cubePoints) == 1
    locker = 1
    disp('COLLISION')
end

for i = 1:steps
    if Estop == 1
        locker = 1
        Estop = 0;
        reset = 0;
    end
    while locker == 1
        if reset == 1;

            if Estop == 1

                locker = 0;
                Estop = 0;
                reset = 0;
                disp('Unlocked')
            end
        else
            Estop = 0;
        end

         disp('Locked')
         pause (1);
    end



    G_T_cur = cheeseRobot.UR3.fkine(cheeseRobot.UR3.getpos()); % find the ur3's current matrix
    % Gripper base
    cheeseRobot.gp_base.basePose = G_T_cur;
    updatedPoints = [cheeseRobot.gp_base.basePose * [cheeseRobot.gp_base.baseVerts,ones(cheeseRobot.gp_base.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_base1.Vertices = updatedPoints(:,1:3); % updated the gripper base's location
    % Gripper Grab -> Gripper finger 1
    rotateTRx = makehgtform('xrotate',(pi/2.5));
    cheeseRobot.gp_fg1.basePose = G_T_cur * rotateTRx;
    updatedPoints = [cheeseRobot.gp_fg1.basePose * [cheeseRobot.gp_fg1.baseVerts,ones(cheeseRobot.gp_fg1.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg1s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % Gripper finger 2
    cheeseRobot.gp_fg2.basePose = G_T_cur * rpy2tr(295,55,158,'deg');
    updatedPoints = [cheeseRobot.gp_fg2.basePose * [cheeseRobot.gp_fg2.baseVerts,ones(cheeseRobot.gp_fg2.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg2s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 2's location
    % Gripper finger 3
    cheeseRobot.gp_fg3.basePose = G_T_cur * rpy2tr(290,300,200,'deg');
    updatedPoints = [cheeseRobot.gp_fg3.basePose * [cheeseRobot.gp_fg3.baseVerts,ones(cheeseRobot.gp_fg3.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg3s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    %bread2
    br_cur = G_T_cur; br_cur(3,end) = br_cur(3,end) - 0.12; % 1-0.12=0.88
    rotateTRy = makehgtform('yrotate',(pi/2));
    rotateTRx = makehgtform('xrotate',(pi));
    cheeseRobot.bread2.basePose = br_cur * rotateTRy * rotateTRx;
    updatedPoints = [cheeseRobot.bread2.basePose * [cheeseRobot.bread2.baseVerts,ones(cheeseRobot.bread2.VertexCount,1)]']'; % get the new position
    cheeseRobot.bread2s.Vertices = updatedPoints(:,1:3); % updated the bread1's location
    % robot's action
    cheeseRobot.UR3.animate(top_bread(i,:));
    drawnow();
end

% NOW place the bread  - Gripper close!!!
ori_q = cheeseRobot.UR3.getpos(); % use q = ans.UR3.getpos() to find current q
targ_q = cheeseRobot.UR3.ikine(transl([-0.15,-0.1,1.05]) * trotx(pi));
top_bread = jtraj(ori_q, targ_q, steps);
% Collision
if MatrixCollisonDetection(bot_bread,cheeseRobot.UR3,UR3_elipse_radius, UR3_elipse_center,cubePoints) == 1
    locker = 1
    disp('COLLISION')
end

for i = 1:steps
    if Estop == 1
        locker = 1
        Estop = 0;
        reset = 0;
    end
    while locker == 1
        if reset == 1;

            if Estop == 1

                locker = 0;
                Estop = 0;
                reset = 0;
                disp('Unlocked')
            end
        else
            Estop = 0;
        end

         disp('Locked')
         pause (1);
    end


    G_T_cur = cheeseRobot.UR3.fkine(cheeseRobot.UR3.getpos()); % find the ur3's current matrix
    % Gripper base
    cheeseRobot.gp_base.basePose = G_T_cur;
    updatedPoints = [cheeseRobot.gp_base.basePose * [cheeseRobot.gp_base.baseVerts,ones(cheeseRobot.gp_base.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_base1.Vertices = updatedPoints(:,1:3); % updated the gripper base's location
    % Gripper Grab -> Gripper finger 1
    rotateTRx = makehgtform('xrotate',(pi/2.5));
    cheeseRobot.gp_fg1.basePose = G_T_cur * rotateTRx;
    updatedPoints = [cheeseRobot.gp_fg1.basePose * [cheeseRobot.gp_fg1.baseVerts,ones(cheeseRobot.gp_fg1.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg1s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % Gripper finger 2
    cheeseRobot.gp_fg2.basePose = G_T_cur * rpy2tr(295,55,158,'deg');
    updatedPoints = [cheeseRobot.gp_fg2.basePose * [cheeseRobot.gp_fg2.baseVerts,ones(cheeseRobot.gp_fg2.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg2s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 2's location
    % Gripper finger 3
    cheeseRobot.gp_fg3.basePose = G_T_cur * rpy2tr(290,300,200,'deg');
    updatedPoints = [cheeseRobot.gp_fg3.basePose * [cheeseRobot.gp_fg3.baseVerts,ones(cheeseRobot.gp_fg3.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg3s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    %bread1
    br_cur = G_T_cur; br_cur(3,end) = br_cur(3,end) - 0.12; % 1-0.12=0.88
    rotateTRy = makehgtform('yrotate',(pi/2));
    rotateTRx = makehgtform('xrotate',(pi));
    cheeseRobot.bread2.basePose = br_cur * rotateTRy * rotateTRx;
    updatedPoints = [cheeseRobot.bread2.basePose * [cheeseRobot.bread2.baseVerts,ones(cheeseRobot.bread2.VertexCount,1)]']'; % get the new position
    cheeseRobot.bread2s.Vertices = updatedPoints(:,1:3); % updated the bread1's location
    % robot's action
    cheeseRobot.UR3.animate(top_bread(i,:));
    drawnow();
end
% UR3 reset
% reset UR3
ori_q = cheeseRobot.UR3.getpos(); % use q = ans.UR3.getpos() to find current q

targ_q = [0 0 0 0 0 0];

top_bread = jtraj(ori_q, targ_q, steps);
% Collision
if MatrixCollisonDetection(bot_bread,cheeseRobot.UR3,UR3_elipse_radius, UR3_elipse_center,cubePoints) == 1
    locker = 1
    disp('COLLISION')
end

for i = 1:steps
    if Estop == 1
        locker = 1
        Estop = 0;
        reset = 0;
    end
    while locker == 1
        if reset == 1;

            if Estop == 1

                locker = 0;
                Estop = 0;
                reset = 0;
                disp('Unlocked')
            end
        else
            Estop = 0;
        end

         disp('Locked')
         pause (1);
    end


    % bread drop
    if i < 10
        br_cur(3,end) = br_cur(3,end) - 0.011; % 0.88-0.08=0.8
        rotateTRx = makehgtform('xrotate',(pi));
        rotateTRy = makehgtform('yrotate',(pi));
        if i == 10, cheeseRobot.bread2.basePose = br_cur * rotateTRx * rotateTRy;
        else cheeseRobot.bread2.basePose = br_cur;end
        updatedPoints = [cheeseRobot.bread2.basePose * [cheeseRobot.bread2.baseVerts,ones(cheeseRobot.bread2.VertexCount,1)]']'; % get the new position
        cheeseRobot.bread2s.Vertices = updatedPoints(:,1:3); % updated the bread1's location
    end
    G_T_cur = cheeseRobot.UR3.fkine(cheeseRobot.UR3.getpos()); % find the ur3's current matrix
    % Gripper base
    cheeseRobot.gp_base.basePose = G_T_cur;
    updatedPoints = [cheeseRobot.gp_base.basePose * [cheeseRobot.gp_base.baseVerts,ones(cheeseRobot.gp_base.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_base1.Vertices = updatedPoints(:,1:3); % updated the gripper base's location
    % Gripper finger 1
    rotateTRx = makehgtform('xrotate',(pi/2));
    cheeseRobot.gp_fg1.basePose = G_T_cur * rotateTRx;
    updatedPoints = [cheeseRobot.gp_fg1.basePose * [cheeseRobot.gp_fg1.baseVerts,ones(cheeseRobot.gp_fg1.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg1s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % Gripper finger 2
    rotateTRx = makehgtform('xrotate',(pi/2));
    rotateTRy = makehgtform('yrotate',(2*pi/3));
    cheeseRobot.gp_fg2.basePose = G_T_cur * rotateTRx * rotateTRy;
    updatedPoints = [cheeseRobot.gp_fg2.basePose * [cheeseRobot.gp_fg2.baseVerts,ones(cheeseRobot.gp_fg2.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg2s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 2's location
    % Gripper finger 3
    rotateTRx = makehgtform('xrotate',(pi/2));
    rotateTRy = makehgtform('yrotate',(-2*pi/3));
    cheeseRobot.gp_fg3.basePose = G_T_cur * rotateTRx * rotateTRy;
    updatedPoints = [cheeseRobot.gp_fg3.basePose * [cheeseRobot.gp_fg3.baseVerts,ones(cheeseRobot.gp_fg3.VertexCount,1)]']'; % get the new position
    cheeseRobot.gp_fg3s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
    % robot's action
    cheeseRobot.UR3.animate(top_bread(i,:));
    drawnow();
end


%% Function Dump
% Lock
function ESTOP_cb(~,~)
    modified_value = 1;% setting the variable to send back
    assignin('base','Estop', modified_value); % sending the variable back from callback
end

function Reset_cb(~,~)
    modified_value = 1;% setting the variable to send back
    assignin('base','reset', modified_value);% sending the variable back from callback
end

function Light_cb(~,~)
    modified_value = 1;% setting the variable to send back
    assignin('base','locker', modified_value);% sending the variable back from callback
end

% Gripper_Release
function gp_release(coor)
% Gripper base
cheeseRobot.gp_base.basePose = coor;
updatedPoints = [cheeseRobot.gp_base.basePose * [cheeseRobot.gp_base.baseVerts,ones(cheeseRobot.gp_base.VertexCount,1)]']'; % get the new position
cheeseRobot.gp_base1.Vertices = updatedPoints(:,1:3); % updated the gripper base's location
% Gripper finger 1
rotateTRx = makehgtform('xrotate',(pi/2));
cheeseRobot.gp_fg1.basePose = coor * rotateTRx;
updatedPoints = [cheeseRobot.gp_fg1.basePose * [cheeseRobot.gp_fg1.baseVerts,ones(cheeseRobot.gp_fg1.VertexCount,1)]']'; % get the new position
cheeseRobot.gp_fg1s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
% Gripper finger 2
rotateTRx = makehgtform('xrotate',(pi/2));
rotateTRy = makehgtform('yrotate',(2*pi/3));
cheeseRobot.gp_fg2.basePose = coor * rotateTRx * rotateTRy;
updatedPoints = [cheeseRobot.gp_fg2.basePose * [cheeseRobot.gp_fg2.baseVerts,ones(cheeseRobot.gp_fg2.VertexCount,1)]']'; % get the new position
cheeseRobot.gp_fg2s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 2's location
% Gripper finger 3
rotateTRx = makehgtform('xrotate',(pi/2));
rotateTRy = makehgtform('yrotate',(-2*pi/3));
cheeseRobot.gp_fg3.basePose = coor * rotateTRx * rotateTRy;
updatedPoints = [cheeseRobot.gp_fg3.basePose * [cheeseRobot.gp_fg3.baseVerts,ones(cheeseRobot.gp_fg3.VertexCount,1)]']'; % get the new position
cheeseRobot.gp_fg3s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
end

function SafeOrNSafe = MatrixCollisonDetection(qCPMatrix,robot,robotelipsesradius, robotelipsecenters,objectpoints)
    SafeOrNSafe = 2;
    for i = 1 : size(qCPMatrix,1)
        if CollisionPath(qCPMatrix(i,:),robot,robotelipsesradius, robotelipsecenters,objectpoints) == 1
            SafeOrNSafe = 1;
        elseif SafeOrNSafe ~= 1
            SafeOrNSafe = 0;
        end   
    end
end

function booleancollisiondetected = CollisionPath(qCP,robot,robotelipsesradius, robotelipsecenters,objectpoints)%,otherrobot    If get time
    booleancollisiondetected = 2; % if returns a 2 at end something went horribly wrong return should be 1 or 0
    tr = zeros(4,4,robot.n+1);
    tr(:,:,1) = robot.base;
    L = robot.links;
  
    for i = 1 : robot.n   % kinematics for each joint in a matrix in 3D space
        tr(:,:,i+1) = tr(:,:,i) * L(i).A(qCP(i));
    end
    
    for i = 1: size(tr,3) % Testing for collision in each ellisiode
        objectpointsAndOnes = [inv(tr(:,:,i)) * [objectpoints,ones(size(objectpoints,1),1)]']';
        objectpointsCurrentlimb = objectpointsAndOnes(:,1:3); %  This is the difference between the arm joint in calcualtion and the box location and then the moxed is moved relative to it
        algebraicDist = GetAlgebraicDist(objectpointsCurrentlimb, robotelipsecenters(i,:), robotelipsesradius(i,:));
        pointsInside = find(algebraicDist < 1);
        if pointsInside > 0
            booleancollisiondetected = 1;
        elseif booleancollisiondetected ~= 1;
            booleancollisiondetected = 0;
        end    
    end
end



function algebraicDist = GetAlgebraicDist(points, centerPoint, radii) % just fuction to detect where the point lies relative to the radius of a ellispoide
algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
              + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
              + ((points(:,3)-centerPoint(3))/radii(3)).^2;
end