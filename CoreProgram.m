%% Clear Matlab
clear;
clc;
clf;

%% Main Loop 
CheeseRobots = czRobot;




% 
% qMatrix = jtraj(q1,q2,steps);
%
% s = lspb(0,1,steps);
% qMatrix = nan(steps,6);
% for i = 1:steps
%    qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
% end



%% testthingsss
%Lab6

q0test = [0,0,0,0,0,0] %testingvairable



% % testing ellisoid things

% centerPoint = [0,0,0];
% radii = [3,2,1];
% [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
% view(3);
% hold on;
% ellipsoidAtOrigin_h = surf(X,Y,Z);
% alpha(0.1);




%% Collision Setup

% Object in interest a cube at location with side of disired
Obj_center_location = [-0.15,0.4,1.05];

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

% % Just in case when needing to delete cube
% try delete(cubeAtOigin_h); end;



IRB_elipse_radius = [0.2,0.1,0.1;  0.2,0.1,0.1;  0.2,0.1,0.1;  0.2,0.1,0.1;   0.2,0.1,0.1;   0.2,0.1,0.1;   0.2,0.1,0.1;]; %the diameter of elipse for each joint
IRB_elipse_center = [0,0,0;  0,0,0;  0,0,0;  0,0,0;   0,0,0;   0,0,0;  0,0,0;] ;%the center offset of elipse for each joint
UR3_elipse_radius = [0.2,0.1,0.1;  0.2,0.1,0.1;  0.2,0.1,0.1;  0.2,0.1,0.1;   0.2,0.1,0.1;   0.2,0.1,0.1;   0.2,0.1,0.1;];%the diameter of elipse for each joint
UR3_elipse_center = [0,0,0;  0,0,0;  0,0,0;  0,0,0;   0,0,0;   0,0,0;  0,0,0;]; %the center offset of elipse for each joint


% Collison IRB Setup
L_IRB_1 = Link('d',0.225,'a',0,'alpha',pi/2,'qlim',[-2*pi 2*pi]);
L_IRB_2 = Link('d',0,'a',0.22,'alpha',0,'qlim',[-122 135]*pi/180, 'offset', pi/2);
L_IRB_3 = Link('d',0,'a',0,'alpha',-pi/2,'qlim',[-220  50]*pi/180);
L_IRB_4 = Link('d',-0.295,'a',0,'alpha',pi/2,'qlim',[-360 360]*pi/180,'offset', pi);
L_IRB_5 = Link('d',0,'a',0,'alpha',-pi/2,'qlim',[-135 150]*pi/180,'offset', pi/2);
L_IRB_6 = Link('d',-0.097,'a',-0.027,'alpha',0,'qlim',[-360 360]*pi/180);

IRBCollision = SerialLink([L_IRB_1 L_IRB_2 L_IRB_3 L_IRB_4 L_IRB_5 L_IRB_6],'name','IRBCollision');  
Collision_IRB_pos = [-0.18,0.5,0.775]; % set robot base's position
pos = makehgtform('translate',[Collision_IRB_pos]); % Rotate/Move robot to the correct orientation
IRBCollision.base = IRBCollision.base * pos;

            
% Collison UR3 Setup          
L_UR3_1 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',[-360 360]*pi/180);
L_UR3_2 = Link('d',0,'a',-0.24365,'alpha',0,'qlim',[-360 360]*pi/180,'offset', -pi/2);
L_UR3_3 = Link('d',0,'a',-0.21325,'alpha',0,'qlim',[-205 28]*pi/180,'offset', -pi/2);
L_UR3_4 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',[-360 360]*pi/180,'offset', -pi) ;
L_UR3_5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',[-360 360]*pi/180,'offset', pi)   ;
L_UR3_6 = Link('d',0.0819,'a',0,'alpha',0,'qlim',[-360 360]*pi/180)    ;
  
UR3Collision = SerialLink([L_UR3_1 L_UR3_2 L_UR3_3 L_UR3_4 L_UR3_5 L_UR3_6],'name','UR3Collision');  
Collision_UR3_pos = [-0.18,-0.5,0.775]; % set robot base's position
pos = makehgtform('translate',[Collision_UR3_pos]); % Rotate/Move robot to the correct orientation
UR3Collision.base = UR3Collision.base * pos;



hold on
IRBCollision.plot(q0test)
UR3Collision.plot(q0test)    

%% Collision Test
testcollision = UR3CollisionPath (q0test,IRBCollision,IRB_elipse_radius, IRB_elipse_center,cubePoints)

%% Setup Robotic Links
function booleancollisiondetected = UR3CollisionPath(qCP,robot,robotelipsesradius, robotelipsecenters,objectpoints)%,otherrobot    If get time
    booleancollisiondetected = 2
    tr = zeros(4,4,robot.n+1);
    tr(:,:,1) = robot.base;
    L = robot.links;
    for i = 1 : robot.n
        tr(:,:,i+1) = tr(:,:,i) * trotz(qCP(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
    end
    
    % Go through each ellipsoid
    for i = 1: size(tr,3)
        test1 = inv(tr(:,:,i))
        test2 = [objectpoints,ones(size(objectpoints,1),1)]'

        objectpointsAndOnes = [inv(tr(:,:,i)) * [objectpoints,ones(size(objectpoints,1),1)]']'
        objectpoints = objectpointsAndOnes(:,1:3); %looks correct moved by appropriate amount
        algebraicDist = GetAlgebraicDist(objectpoints, robotelipsecenters(i,:), robotelipsesradius(i,:));
        pointsInside = find(algebraicDist < 1);
        if pointsInside > 0
            booleancollisiondetected = 1;
        elseif booleancollisiondetected ~= 1;
            booleancollisiondetected = 0;
        end    

        display(['2.10: There are ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);
    end
end



function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)

algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
              + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
              + ((points(:,3)-centerPoint(3))/radii(3)).^2;
end