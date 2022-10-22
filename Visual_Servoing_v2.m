%% camera & image & points setup
% Create image target (points in the image plane)
pStar = [662 362 362 662; 362 362 662 662];

%Create 3D points
P=[1.2,1.2,1.2,1.2;
    0.75,0.25,0.25,0.75;
    1.493,1.493,0.993,0.993];

r = czRobot;

%Initial pose
q0 = [0; 0; 0; 0; -pi/2; 0];

% Add the camera
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
    'resolution', [1024 1024], 'centre', [512 512],'name', 'IRBcamera');

% frame rate
fps = 25;

%gain of the controler
lambda = 0.025;
%depth of the IBVS
depth = mean (P(1,:));
%% initialize simulation
%Display IRB
Tc0 = r.IRB.fkine(q0)*troty(pi); % get current location - EE
r.IRB.animate(q0'); % animate it
drawnow

% plot camera and points
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
    
    J = cam.visjac_p(uv, depth);
    % compute the velocity of camera in camera frame
    try
        v = lambda * pinv(J) * e;
    catch
        status = -1;
        return
    end
    fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);
    
    %compute robot's Jacobian and inverse
    J2 = r.IRB.jacobn(q0);
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
    q = q0 - (1/fps)*qp;
    r.IRB.animate(q');
    
    %Get camera location
    Tc = r.IRB.fkine(q)*troty(pi);
    cam.T = Tc;
    drawnow
    
    pause(1/fps)
    
    if ~isempty(200) && (ksteps > 200)
        break;
    end
    
    %update current joint position
    q0 = q;
end %loop finishes