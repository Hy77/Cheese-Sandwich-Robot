function ATeachUR3(robot, varargin)
    %-------------------------------
    % parameters for teach panel
	bgcol = [80 250 80]/255;
	height = 0.06; 
    % height of slider rows
    %-------------------------------
    
	%% handle options
	opt.deg = true;
	opt.mode = {'xyz', 'joints'};
    opt.orientation = {'rpy', 'eul', 'approach'};
    opt.callback = [];    
    [opt,args] = tb_optparse(opt, varargin);
	
    % stash some options into the persistent object
	handles.orientation = opt.orientation;
    handles.callback = opt.callback;
    handles.opt = opt;
	handles.mode = opt.mode;
	
    % we need to have qlim set to finite values for a prismatic joint
	qlim = robot.model.qlim;
	if any(isinf(qlim))
		error('RTB:teach:badarg', 'Must define joint coordinate limits for prismatic axes, set qlim properties for prismatic Links');
	end
	
	if isempty(args)
		q = [];
	else
		q = args{1};
	end
	
	% set up scale factor, from actual limits in radians/metres to display units
	qscale = ones(robot.model.n,1);
	for j = 1:robot.model.n
		L = robot.model.links(j);
		if opt.deg && L.isrevolute
			qscale(j) = 180/pi;
		end
	end
	handles.qscale = qscale;
	handles.robot = robot;
	
    
   %% install the panel at the side of the figure (Teach Panel)
   
	%find the right figure to put it in
	c = findobj(gca, 'Tag', robot.model.name);
	if isempty(c)
		c = findobj(0, 'Tag', robot.model.name);
		if isempty(c)
            % doesn't exist in current axes, look wider
			robot.model.plot(zeros(1, robot.model.n));
			ax = gca;
        else
            % found it in current axes
			ax = get(c(1), 'Parent');% get first axis holding the robot
		end
	else
		ax = gca;
	end
	
		handles.fig = get(ax, 'Parent');  % get the figure that holds the axis
		
         % shrink the current axes to make room
         %   [l b w h]
		set(ax, 'Outerposition', [0.25 0 0.70 1])
		
		handles.curax = ax;
		
		% create the panel itself
		panel = uipanel(handles.fig, ...
			'Title', 'ATeachUR3', ...
			'BackGroundColor', bgcol, ...
			'Position', [0 0 .25 1]);
		set(panel, 'Units', 'pixels'); % stop automatic resizing
		handles.panel = panel;
		set(handles.fig, 'Units', 'pixels');
        
		set(handles.fig, 'ResizeFcn', @(src,event) resize_callback(robot.model, handles));
		
		
		%---- get the current robot state
		if isempty(q)
			rhandles = findobj('Tag', robot.model.name);
			% find the graphical element of this name
			if isempty(rhandles)
                % check to see if there are any graphical robots of this name
				error('RTB:teach:badarg', 'No graphical robot of this name found');
            end
            % get the info from its Userdata
			info = get(rhandles(1), 'UserData');
            
			% the handle contains current joint angles (set by plot)
			if ~isempty(info.q)
				q = info.q;
			end
		else
			robot.model.plot(q);
		end

		handles.q = q;
		T6 = robot.model.fkine(q);
		
		%% Sliders for Advanced Teach
		
        NumSlider= 3; %number of inverse kinematic sliders (has to be independant of joint number)
		XYZtag = ['x', 'y', 'z']; %character string for the sliders, where the charater position is associted the the slider number
        IKmove = transl(T6);
		
		for j = 1:NumSlider
			% slider label
			uicontrol(panel, 'Style', 'text', ...
				'Units', 'normalized', ...
				'BackgroundColor', bgcol, ...
				'Position', [0 height*(NumSlider - j + 2) 0.15 height], ...
				'FontUnits', 'normalized', ...
				'FontSize', 0.5, ...
				'String', sprintf('%c', XYZtag(1, j))); 
			
			% slider itself
			IKINEreach = [[-.722 -1.042 .925]; [.362 .042 1.47]]; %The reach limit for each robot based on its global position 
            
            %MUST BE ADJUSTED IF THE BASE OF THE ROBOT IS RELOCATED
            %>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
            
			handles.slider(j) = uicontrol(panel, 'Style', 'slider', ...
				'Units', 'normalized', ...
				'Position', [0.15 height*(NumSlider - j + 2) 0.65 height], ... %Distance between the sliders
				'Min', IKINEreach(1, j), ... %based on robot effective work space instead of join qlims
				'Max', IKINEreach(2, j), ...
				'Value', IKmove(j, 1), ...
				'Tag', sprintf('Slider%c', XYZtag(1, j)));
			
			% text box showing slider value, also editable
			handles.edit(j) = uicontrol(panel, 'Style', 'edit', ...
				'Units', 'normalized', ...
				'Position', [0.80 height*(NumSlider - j + 2)+.01 0.20 0.9*height], ...
				'BackgroundColor', bgcol, ...
				'String', num2str(IKmove(j, 1), 3), ...
				'HorizontalAlignment', 'left', ...
				'FontUnits', 'normalized', ...
				'FontSize', 0.4, ...
				'Tag', sprintf('Edit%c', XYZtag(1, j))); 
        end
	
        
        
	%% Existing Fkine sliders
	%---- now make the sliders
		n = robot.model.n;
		for j = 1:n
			% slider label
			uicontrol(panel, 'Style', 'text', ...
				'Units', 'normalized', ...
				'BackgroundColor', bgcol, ...
				'Position', [0 height*(n - j + 5) 0.15 height], ... %had to move the sliders to make room for ikine sliders
				'FontUnits', 'normalized', ...
				'FontSize', 0.5, ...
				'String', sprintf('q%d', j));
			
			% slider itself
			q(j) = max( qlim(j,1), min( qlim(j,2), q(j) ) ); % clip to range
			handles.slider(j + 3) = uicontrol(panel, 'Style', 'slider', ...
				'Units', 'normalized', ...
				'Position', [0.15 height*(n - j + 5) 0.65 height], ...
				'Min', qlim(j,1), ...
				'Max', qlim(j,2), ...
				'Value', q(j), ...
				'Tag', sprintf('Slider%d', j));
			
			% text box showing slider value, also editable
			handles.edit(j + 3) = uicontrol(panel, 'Style', 'edit', ...
				'Units', 'normalized', ...
				'Position', [0.80 height*(n - j + 5)+.01 0.20 0.9*height], ...
				'BackgroundColor', bgcol, ...
				'String', num2str(qscale(j)*q(j), 5), ...
				'HorizontalAlignment', 'left', ...
				'FontUnits', 'normalized', ...
				'FontSize', 0.4, ...
				'Tag', sprintf('Edit%d', j));
		end
		
%% ---- set up the position display box
    
    % X
    uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.05 1-height 0.2 0.7*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'HorizontalAlignment', 'left', ...
        'String', 'x:');
    
    handles.t6.t(1) = uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-height 0.6 0.7*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', T6(1, 4)), ...
        'Tag', 'T6');
    
    % Y
    uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.05 1-1.7*height 0.2 0.7*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', 'y:');
    
    handles.t6.t(2) = uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-1.7*height 0.6 0.7*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', T6(2, 4)));
    
    % Z
    uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.05 1-2.4*height 0.2 0.7*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', 'z:');
    
    handles.t6.t(3) = uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-2.4* height 0.6 0.7*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', T6(3, 4)));
    
    % Orientation
    switch opt.orientation
        case 'approach'
            labels = {'ax:', 'ay:', 'az:'};
        case 'eul'
            labels = {[char(hex2dec('3c6')) ':'], [char(hex2dec('3b8')) ':'], [char(hex2dec('3c8')) ':']}; % phi theta psi
        case'rpy'
            labels = {'R:', 'P:', 'Y:'};
    end
    
    %---- set up the orientation display box

    % AX
    uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.05 1-3.1*height 0.2 0.7*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', labels(1));
    
    handles.t6.r(1) = uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-3.1*height 0.6 0.7*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', T6(1, 3)));
    
    % AY
    uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.05 1-3.8*height 0.2 0.7*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', labels(2));
    
    handles.t6.r(2) = uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-3.8*height 0.6 0.7*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', T6(2, 3)));
    
    % AZ
    uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'BackgroundColor', bgcol, ...
        'Position', [0.05 1-4.5*height 0.2 0.7*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.9, ...
        'HorizontalAlignment', 'left', ...
        'String', labels(3));
    
    handles.t6.r(3) = uicontrol(panel, 'Style', 'text', ...
        'Units', 'normalized', ...
        'Position', [0.3 1-4.5*height 0.6 0.7*height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.8, ...
        'String', sprintf('%.3f', T6(3, 3)));   


    %---- add buttons
    uicontrol(panel, 'Style', 'pushbutton', ...
        'Units', 'normalized', ...
        'Position', [0.80 height*(0)+.01 0.15 height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.7, ...
        'CallBack', @(src,event) quit_callback(robot.model, handles), ...
        'BackgroundColor', 'white', ...
        'ForegroundColor', 'red', ...
        'String', 'X');
    
    % the record button
    handles.record = [];
    if ~isempty(opt.callback)
    uicontrol(panel, 'Style', 'pushbutton', ...
        'Units', 'normalized', ...
        'Position', [0.1 height*(0)+.01 0.30 height], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.6, ...
        'CallBack', @(src,event) record_callback(robot.model, handles), ...
        'BackgroundColor', 'red', ...
        'ForegroundColor', 'white', ...
        'String', 'REC');
	end
    
    %---- now assign the callbacks
	n = robot.model.n + 3; %The Q and X Y Z
    for j = 1:n
        % text edit box
        set(handles.edit(j), ...
            'Interruptible', 'off', ...
            'Callback', @(src,event)ATeachUR3_callback(src, robot.model.name, j, handles));
        
        % slider
        set(handles.slider(j), ...
            'Interruptible', 'off', ...
            'BusyAction', 'queue', ...
            'Callback', @(src,event)ATeachUR3_callback(src, robot.model.name, j, handles));
	end
end
%% ATeachUR3 Callback
function ATeachUR3_callback(src, name, j, handles)

    % called on changes to a slider or to the edit box showing joint coordinate
    %
    % src      the object that caused the event
    % name     name of the robot
    % j        the joint index concerned (1..N)
    % slider   true if the
    
	c2 = findobj('Tag', name);
	
	%find graphical element with name h
	if isempty(c2)
		error('RTB:teach:badarg', 'No graphical robot of this name found');
	end
	
	%Initial Values for the X Y Z position of end effector and manual input of base
	%position
	IKPos0  = [get(handles.slider(1), 'Value'), get(handles.slider(2), 'Value'), get(handles.slider(3), 'Value')];
	IKPos0  = [get(handles.slider(1), 'Value'), get(handles.slider(2), 'Value'), get(handles.slider(3), 'Value')];
    info = get(c2(1), 'UserData');
	%Find current Joint angles
	Qang0 = info.q;
	%update the stored joint coordinates
	set(c2(1), 'UserData', info);
	

	
    %For the Q sliders (Will not work of 3 DOF robots)
	if j >= 4
		switch get(src, 'Style')
			case 'slider'
				newval = get(src, 'Value');
				set(handles.edit(j), 'String', num2str(newval * 180/pi, 3)); %Radians to degrees for joint angle display
			case 'edit'
				newval = str2double(get(src, 'String'));
				set(handles.slider(j), 'Value', newval);
        end
        
		info.q(j - 3) = newval;
        % and save it back to the graphical object
		set(c2(1), 'UserData', info);
        % update all robots of this name
		animate(handles.robot.model, info.q);
    end
     
    
	%For the Ikine Sliders to show coordinates instead of "angles"
	if j < 4 
		switch get(src, 'Style')
			case 'slider'
				newval = get(src, 'Value');
				set(handles.edit(j), 'String', num2str(newval, 3));
			case 'edit'
				newval = str2double(get(src, 'String'));
				set(handles.slider(j), 'Value', newval);
		end
		IKPos1 = IKPos0;
		IKPos1(1, j) = newval;
		Pos1IK = transl(IKPos1);
		Posq1 = handles.robot.model.ikcon(Pos1IK, Qang0);
		qMatrix = handles.robot.PathIK(Qang0, Posq1, 50); %Number of steps for position change
		handles.robot.ModelIKinematics(qMatrix, handles.robot);
		info.q = Posq1;
		set(c2(1), 'UserData', info);
		nQ = size(Posq1);
		nQ = nQ(1, 2);
		for i = 1:nQ
			set(handles.slider(i+3), 'Value', Posq1(1, i));
			set(handles.edit(i+3), 'String', num2str(Posq1(1, i) * 180/pi, 3));
		end
			
    end    
    
    
	% compute the robot tool pose
	T6 = handles.robot.model.fkine(info.q);
    
 
	% convert orientation to desired format
	switch handles.orientation
		case 'approach'
			orient = T6(:,3);    % approach vector
		case 'eul'
			orient = tr2eul(T6, 'setopt', handles.opt);
		case'rpy'
			orient = tr2rpy(T6, 'setopt', handles.opt);
	end
	
	% update the display in the teach window
    for i = 1:3
        set(handles.t6.t(i), 'String', sprintf('%.3f', T6(i, 4)));
        set(handles.t6.r(i), 'String', sprintf('%.3f', orient(i)));
	end
    
    if ~isempty(handles.callback)
        handles.callback(handles.robot, info.q);
    end
    
    %notify(handles.robot, 'Moved');
	
end


function quit_callback(robot, handles)
    set(handles.fig, 'ResizeFcn', '');
    delete(handles.panel);
    set(handles.curax, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1])
end


function resize_callback(robot, handles)

    % come here on figure resize events
    fig = gcbo;   % this figure (whose callback is executing)
    fs = get(fig, 'Position');  % get size of figure
    ps = get(handles.panel, 'Position');  % get position of the panel
    % update dimensions of the axis area
    set(handles.curax, 'Units', 'pixels', ...
        'OuterPosition', [ps(3) 0 fs(3)-ps(3) fs(4)]);
    % keep the panel anchored to the top left corner
    set(handles.panel, 'Position', [1 fs(4)-ps(4) ps(3:4)]);
end
