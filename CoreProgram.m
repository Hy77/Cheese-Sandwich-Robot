%% Clear Matlab
clear;
clc;
clf;

% Note in following in quotation mars this section below
% "%---------------------------------------------------"
%Symbolises code used for debugging and Should be commented out for final
%version. Also if the quotation marks appears at start of line the code
%the following code until the next mark is testing /debugging code

%% Setup section
CheeseRobots = czRobot;







%% testthingsss
%Lab6

q0test = [0,0,0,0,0,0] %testingvairable


%% Collision Variables
% Object in interest a cube at location with side of disired
Obj_center_location = [-0.15,0.4,1.05];% Middle of IRB


% test other points
% Obj_center_location = [0.04,0.5,0.705];% in Table infront of IRB %---------------------------------------------------
% Obj_center_location = [-0.18,0.5,0.775];% Base of IRB%---------------------------------------------------
% Obj_center_location = [-0.15,0.15,1.1];% hovering over Chopping board%---------------------------------------------------


% Obj_center_location = [-0.2,-0.5,0.775];% Base of UR3%---------------------------------------------------


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

% cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'r.');  
% Code From lab not nessary for this

%Spawning in the cube
cubePoints = cubePoints + repmat(Obj_center_location,size(cubePoints,1),1);    
cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
axis equal



% % Just in case when needing to delete cube%---------------------------------------------------
% try delete(cubeAtOigin_h); end;%---------------------------------------------------


% Collision Bubble specs
IRB_elipse_radius = [0.1,0.1,0.225;  0.1,0.1,0.22;  0.1,0.1,0.1;  0.2,0.1,0.1;   0.1,0.1,0.1;   0.1,0.1,0.1;   0.1,0.1,0.1;]; %the diameter of elipse for each joint
IRB_elipse_center = [0,0,0;  0,0,0;  0,0,0;  0,0,0;   0,0,0;   0,0,0;  0,0,0;] ;%the center offset of elipse for each joint
UR3_elipse_radius = [0.1,0.1,0.225;  0.1,0.1,0.1;  0.1,0.1,0.22;  0.2,0.1,0.1;   0.1,0.1,0.1;   0.1,0.1,0.1;;   0.1,0.1,0.1;;];%the diameter of elipse for each joint
UR3_elipse_center = [0,0,0;  0,0,0;  0,0,0;  0,0,0;   0,0,0;   0,0,0;  0,0,0;]; %the center offset of elipse for each joint


%---------------------------------------------------
% % test Large elipse
% IRB_elipse_radius = [2,1,1;  2,1,1;  2,1,1;  2,1,1;   2,1,1;   2,1,1;   2,1,1;]; %the diameter of elipse for each joint
% IRB_elipse_center = [0,0,0;  0,0,0;  0,0,0;  0,0,0;   0,0,0;   0,0,0;  0,0,0;] ;%the center offset of elipse for each joint
% UR3_elipse_radius = [2,1,1;  2,1,1;  2,1,1;  2,1,1;   2,1,1;   2,1,1;   2,1,1;];%the diameter of elipse for each joint
% UR3_elipse_center = [0,0,0;  0,0,0;  0,0,0;  0,0,0;   0,0,0;   0,0,0;  0,0,0;]; %the center offset of elipse for each joint
%---------------------------------------------------


%% Collision Test Variables 


% % test robots to see position of collision not needed for actual Collision
%
% % Collison IRB Setup
% L_IRB_1 = Link('d',0.225,'a',0,'alpha',pi/2,'qlim',[-2*pi 2*pi]);
% L_IRB_2 = Link('d',0,'a',0.22,'alpha',0,'qlim',[-122 135]*pi/180, 'offset', pi/2);
% L_IRB_3 = Link('d',0,'a',0,'alpha',-pi/2,'qlim',[-220  50]*pi/180);
% L_IRB_4 = Link('d',-0.295,'a',0,'alpha',pi/2,'qlim',[-360 360]*pi/180,'offset', pi);
% L_IRB_5 = Link('d',0,'a',0,'alpha',-pi/2,'qlim',[-135 150]*pi/180,'offset', pi/2);
% L_IRB_6 = Link('d',-0.097,'a',-0.027,'alpha',0,'qlim',[-360 360]*pi/180);
% 
% IRBCollision = SerialLink([L_IRB_1 L_IRB_2 L_IRB_3 L_IRB_4 L_IRB_5 L_IRB_6],'name','IRBCollision');  
% Collision_IRB_pos = [-0.18,0.5,0.775]; % set robot base's position
% pos = makehgtform('translate',[Collision_IRB_pos]); % Rotate/Move robot to the correct orientation
% IRBCollision.base = IRBCollision.base * pos;
% 
%             
% % Collison UR3 Setup          
% L_UR3_1 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',[-360 360]*pi/180);
% L_UR3_2 = Link('d',0,'a',-0.24365,'alpha',0,'qlim',[-360 360]*pi/180,'offset', -pi/2);
% L_UR3_3 = Link('d',0,'a',-0.21325,'alpha',0,'qlim',[-205 28]*pi/180,'offset', -pi/2);
% L_UR3_4 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',[-360 360]*pi/180,'offset', -pi) ;
% L_UR3_5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',[-360 360]*pi/180,'offset', pi)   ;
% L_UR3_6 = Link('d',0.0819,'a',0,'alpha',0,'qlim',[-360 360]*pi/180)    ;
%   
% UR3Collision = SerialLink([L_UR3_1 L_UR3_2 L_UR3_3 L_UR3_4 L_UR3_5 L_UR3_6],'name','UR3Collision');  
% Collision_UR3_pos = [-0.18,-0.5,0.775]; % set robot base's position
% pos = makehgtform('translate',[Collision_UR3_pos]); % Rotate/Move robot to the correct orientation
% UR3Collision.base = UR3Collision.base * pos;
% 
% 
% 
% hold on
% IRBCollision.plot(q0test)
% UR3Collision.plot(q0test)    

%% Collision Test
% % % % % try delete(cubeAtOigin_h); end; %---------------------------------------------------
% % % % %Test for compact at original cube location
% % % % q0test = [pi/2,pi/1.5,-2.3*pi,0,0,0]%---------------------------------------------------
% % % % % IRBCollision.plot(q0test)%---------------------------------------------------
% % % % % 
% % % % % q0test = CheeseRobots.IRB.ikine(trvec2tform([-0.15,0.15,0.79]))%---------------------------------------------------
% % % % % IRBCollision.plot(q0test) %---------------------------------------------------
% % % % CheeseRobots.IRB.animate(q0test);%---------------------------------------------------
% % % % 
% % % % testcollision = CollisionPath (q0test,CheeseRobots.IRB,IRB_elipse_radius, IRB_elipse_center,cubePoints)

ori_q = [0 0 0 0 0 0];
% set steps = 50 ez for demonstration
steps = 50;
% get joint_q by using ikine
targ_q = CheeseRobots.UR3.ikine(transl([0.185,-0.53,0.99]) * troty(pi) * trotz(pi));
% create jtraj
q0matrix = jtraj(ori_q, targ_q, steps);

test22222 = MatrixCollisonDetection(q0matrix,CheeseRobots.UR3,UR3_elipse_radius, UR3_elipse_center,cubePoints)
% CheeseRobots.UR3.animate(q0matrix(43,:));%---------------------------------------------------
% testtr999 = IRBCollision.fkine(q0test)%---------------------------------------------------
% testtr000= CheeseRobots.IRB.fkine(q0test)%---------------------------------------------------

%% Setup Robotic Links
function SafeOrNSafe = MatrixCollisonDetection(qCPMatrix,robot,robotelipsesradius, robotelipsecenters,objectpoints)
    SafeOrNSafe = 2;
    for i = 1 : size(qCPMatrix,1)
%        test = CollisionPath(qCPMatrix(i,:),robot,robotelipsesradius, robotelipsecenters,objectpoints)%---------------------------------------------------
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
%     cube_h = plot3(objectpoints(:,1),objectpoints(:,2),objectpoints(:,3),'y.'); %---------------------------------------------------
    
    for i = 1: size(tr,3) % Testing for collision in each ellisiode
        objectpointsAndOnes = [inv(tr(:,:,i)) * [objectpoints,ones(size(objectpoints,1),1)]']';
        objectpointsCurrentlimb = objectpointsAndOnes(:,1:3); %  This is the difference between the arm joint in calcualtion and the box location and then the moxed is moved relative to it
%         cube_h = plot3(objectpointsCurrentlimb(:,1),objectpointsCurrentlimb(:,2),objectpointsCurrentlimb(:,3),'r.');% test for where the object is in 3d enviroment %---------------------------------------------------


        algebraicDist = GetAlgebraicDist(objectpointsCurrentlimb, robotelipsecenters(i,:), robotelipsesradius(i,:));
        pointsInside = find(algebraicDist < 1);
        if pointsInside > 0
            booleancollisiondetected = 1;
        elseif booleancollisiondetected ~= 1;
            booleancollisiondetected = 0;
        end    

        % display(['2.10: There are ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);% test to see waht joints are supposed to collide%---------------------------------------------------
    end
end



function algebraicDist = GetAlgebraicDist(points, centerPoint, radii) % just fuction to detect where the point lies relative to the radius of a ellispoide
algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
              + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
              + ((points(:,3)-centerPoint(3))/radii(3)).^2;
end