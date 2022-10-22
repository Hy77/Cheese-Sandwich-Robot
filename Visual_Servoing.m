cheeseRobot = czRobot;
%% setup environment
% bread
forwardTR = transl([-0.15,-0.1,0.8]);
cheeseRobot.bread1.basePose = forwardTR;
updatedPoints = [cheeseRobot.bread1.basePose * [cheeseRobot.bread1.baseVerts,ones(cheeseRobot.bread1.VertexCount,1)]']'; % get the new position
cheeseRobot.bread1s.Vertices = updatedPoints(:,1:3); % updated the bread1's location
% cheeseblock
forwardTR = transl([-0.153,0.0422,0.84]);
cheeseRobot.c_block.basePose = forwardTR;
updatedPoints = [cheeseRobot.c_block.basePose * [cheeseRobot.c_block.baseVerts,ones(cheeseRobot.c_block.VertexCount,1)]']'; % get the new position
cheeseRobot.c_block1.Vertices = updatedPoints(:,1:3); % updated the c_block's location
% grippers_base_fingers
forwardTR = transl([-0.153, 0.0422, 0.9076]) * trotx(pi);
cheeseRobot.gp_base.basePose = forwardTR;
updatedPoints = [cheeseRobot.gp_base.basePose * [cheeseRobot.gp_base.baseVerts,ones(cheeseRobot.gp_base.VertexCount,1)]']'; % get the new position
cheeseRobot.gp_base1.Vertices = updatedPoints(:,1:3); % updated the gripper base's location
% Gripper Grab -> Gripper finger 1
cheeseRobot.gp_fg1.basePose = forwardTR * rpy2tr(110,0,0,'deg');
updatedPoints = [cheeseRobot.gp_fg1.basePose * [cheeseRobot.gp_fg1.baseVerts,ones(cheeseRobot.gp_fg1.VertexCount,1)]']'; % get the new position
cheeseRobot.gp_fg1s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
% Gripper finger 2
cheeseRobot.gp_fg2.basePose = forwardTR * rpy2tr(240,45,200,'deg');
updatedPoints = [cheeseRobot.gp_fg2.basePose * [cheeseRobot.gp_fg2.baseVerts,ones(cheeseRobot.gp_fg2.VertexCount,1)]']'; % get the new position
cheeseRobot.gp_fg2s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 2's location
% Gripper finger 3
cheeseRobot.gp_fg3.basePose = forwardTR * rpy2tr(50,240,325,'deg');
updatedPoints = [cheeseRobot.gp_fg3.basePose * [cheeseRobot.gp_fg3.baseVerts,ones(cheeseRobot.gp_fg3.VertexCount,1)]']'; % get the new position
cheeseRobot.gp_fg3s.Vertices = updatedPoints(:,1:3); % updated the gripper finger 1's location
% set up UR3
UR3_q = [1.7295 -1.5509 1.8747 1.2470 -1.5708 0.1587]; % on the cheeseblock
cheeseRobot.UR3.animate(UR3_q);
% set up IRB
IRB_q = [-1.5708; -0.6728; 0.6720; 0; 0; 0];
cheeseRobot.IRB.animate(IRB_q');
%% camera & image & points setup
% Create image target (points in the image plane)
pStar = [755 245 320 680; 570 570 275 275];

%Create 3D points
P=[-0.23, -0.23, -0.13, -0.13;
    0.1179, 0.0179, 0.1179, 0.0179;
    1.01, 1.01, 1.01, 1.01];

% Add the camera
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
    'resolution', [1000 1000], 'centre', [500 500],'name', 'IRBcamera');

% frame rate
fps = 25;

%Define values
%gain of the controler
lambda = 0.03;
%depth of the IBVS
depth = mean (P(1,:));
%% initialize simulation
% plot camera and points
Tc0 = cheeseRobot.IRB.fkine(IRB_q) * troty(pi); % get current location - EE
cam.T = Tc0;

% Display points in 3D and the camera
cam.plot_camera('Tcam',Tc0, 'label','scale',0.06);
plot_sphere(P, 0.01, 'b')
%% display in graph
%Project points to the image
p = cam.plot(P, 'Tcam', Tc0); % Put the points seen by the camera into the image

%camera view and plotting
cam.clf()
cam.plot(pStar, '*'); % create the camera view
cam.hold(true);
cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view
pause(2)
cam.hold(true);
cam.plot(P);    % show initial view
%% loop of the visual servoing
ksteps = 0;
while true
    ksteps = ksteps + 1;
    
    % compute the view of the camera
    uv = cam.plot(P);
    % compute image plane error as a column
    e = pStar-uv;   % feature error
    e = e(:);
    
    % compute the current Image Jacobian
    J = cam.visjac_p(uv, depth );
    
    % compute the velocity of camera in camera frame
    try
        v = lambda * pinv(J) * e;
    catch
        status = -1;
        return
    end
    
    %compute robot's Jacobian and inverse
    J2 = cheeseRobot.IRB.jacobn(IRB_q);
    Jinv = pinv(J2);
    % get joint velocities
    qp = Jinv*v;
    
    %Maximum angular velocity cannot exceed 180 degrees/s
    ind=find(qp>pi);
    if ~isempty(ind)
        qp(ind)=pi;
    end
    ind=find(qp<-pi);
    if ~isempty(ind)
        qp(ind)=-pi;
    end
    
    %Update joints
    q = IRB_q + (1/fps)*qp;
    cheeseRobot.IRB.animate(q');
    
    %Get camera location
    Tc = cheeseRobot.IRB.fkine(q) * troty(pi);
    cam.T = Tc;
    
    drawnow
    
    pause(1/fps)
    
    if ~isempty(200) && (ksteps > 200)
        break;
    end
    
    %update current joint position
    IRB_q = q;
end %loop finishes