function main()
%%

% Setup your variables and stuff
% ...

% Setup input window and flag
figure('Position',[50,800,250,250],'MenuBar','none','Name','Food found input','NumberTitle','off');
set(gcf,'WindowButtonDownFcn',@setFoodFlag); % Mouse click
set(gcf,'KeyPressFcn',@setFoodFlag); % Key press
foodFlag = 0;

% Start main loop
while(1)
    % Do your normal robot control stuff
    % ...
    
    % Check for click on food window
    if foodFlag == 1
        % Do food finding stuff
        % ...
        disp('Food found!');
        
        foodFlag = 0; % Reset flag
    end
    drawnow; % Need this to register button presses
end

% Do cleanup and whatever else
% ...

% This function gets called on click / button press
% Note this is inside main function!
function setFoodFlag(~,~)
    foodFlag = 1;
end

end