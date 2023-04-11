figure 
% Plot the initial position of the robot
plot(xActual(1,1),xActual(2,1),"rpentagram",'LineWidth',1,'MarkerSize',10);

% set axis limits
set(gca,'XLim',[-6 12],'YLim',[-6 14]);

% plot the reference trajectory
% plot(x_ref(1:end),y_ref(1:end),"k-",'LineWidth',2);
hold on;
grid on;
% axis equal;

% parameters c
curve_robot = animatedline('Color','r','LineWidth',2);
curve_ref = animatedline('Color','k','LineWidth',2);

x = xActual(1,:);
y = xActual(2,:);
num_states = size(xActual,2);
num_iter = length(0:0.1:30);

% Plot the initial position of the robot
% plot(xActual(1,1),xActual(2,1),"rpentagram",'LineWidth',1,'MarkerSize',10);

for i = 1:num_iter
    %plot(xActual(1,i),xActual(2,i),'o','MarkerSize',2,'LineWidth',2);
    %pause(0.2)
    if(i <= num_states)
        addpoints(curve_robot,x(i),y(i));
    end 
    addpoints(curve_ref,x_ref(i),y_ref(i));
    drawnow;
    % pause(0.02);
    % movie(i) = getframe;
end
% generate video
% video = VideoWriter('Untuned_MPC','MPEG-4');
% video.FrameRate = 30;
% 
% open(video);
% writeVideo(video,movie);
% close(video);

