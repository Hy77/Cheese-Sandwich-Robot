
%Creating the Gui Figure
EstopFigure = figure('Name','Safety GUI')
handles.fig=figure(EstopFigure);
handles.pb1= uicontrol('style','pushbutton','position',[100 100 80 40],'callback',@ESTOP_cb,'string','ESTOP');
handles.pb2= uicontrol('style','pushbutton','position',[200 100 80 40],'callback',@Reset_cb,'string','Reset');
handles.pb3= uicontrol('style','pushbutton','position',[300 100 80 40],'callback',@Light_cb,'string','Lightcurtain');
guidata(handles.fig,handles)


% Variable to create dual stage lockout system
locker = 0;
Estop = 0;
reset = 0;

% Creating figure to store the cheese robot
CheeseRobotFigure = figure('Name','Movement of Robot'))
%INsert Cheese robot starting variables here

for i=0:100
    % Start of the lockout section
    % We Would need to insert this into every for loop for the movement of
    % the robot
    if Estop == 1
        locker = 1
        Estop = 0;
        reset = 0;
    end
    while locker == 1
        if reset == 1;
%             disp('reset = 1')
            if Estop == 1
%                 disp('Estop = 1')
                locker = 0;
                Estop = 0;
                reset = 0;
            end
        else
            Estop = 0;
        end

         disp('Locked')
         pause (1);
    end
    % end of lockout section


    %Insert Code Here that would be the Movement of the ROBOT 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    disp('Unlocked')
    pause(1)
end



%% Function List
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

