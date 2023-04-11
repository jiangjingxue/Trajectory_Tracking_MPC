function [x_ref,y_ref,theta_ref,v_ref,viaPoints,a_ref,w_ref] = generate_route(dt,tsim)

% ---Start of user-controlled section--- % 

% define boundry conditions

pi = [0 0];         % [xi yi] 
vi = 0.1;           % consists of vxi,vyi 
thetai = 90;        % orientation of the starting point, used to compute vxi,vyi,axi,ayi

pf = [6 0];        % [xf yf]
vf = 0.1;
thetaf = 270;

% define viaPoints
viaPoints = [0 , 2, 2, 4, 4, 6
             10,10, 0, 0,10,10
            ];

% define time for the points
tPoints = [0,6,8,14,16,22,24,tsim];

% ----End of user-controlled section---- % 

% Check if time points are valid
if(tPoints(end-1) >= tsim)
    msg = 'Invalid time points';
    error(msg)
end

% Decompose
xi = pi(1);
yi = pi(2);
vxi = vi * cosd(thetai);
vyi = vi * sind(thetai);

xf = pf(1);
yf = pf(2);
vxf = vf * cosd(thetaf);
vyf = vf * sind(thetaf);

xm = viaPoints(1,1:6);
ym = viaPoints(2,1:6);

% pre-compute the coefficients of the polynomials  
xCoefs = calculate_coeffs(xi,vxi,xf,vxf,xm,tPoints);
yCoefs = calculate_coeffs(yi,vyi,yf,vyf,ym,tPoints);

% create empty arrays to store data 
[x_ref,y_ref,theta_ref,v_ref,a_ref,w_ref] = deal([],[],[],[],[],[]);

% main for loop 
for t = 0:dt:tsim
    if(t < tPoints(2))
        x_ref(end+1) = calc_position(xCoefs(:,1),t);
        y_ref(end+1) = calc_position(yCoefs(:,1),t);

        vx = calc_velocity(xCoefs(:,1),t);
        vy = calc_velocity(yCoefs(:,1),t);

        v_ref(end+1) = hypot(vx,vy);
        theta_ref(end+1) = atan2d(vy,vx);

        % calculate acceleration components
        ax = calc_acceleration(xCoefs(:,1),t);
        ay = calc_acceleration(yCoefs(:,1),t);
        a_temp = hypot(ax,ay);

        % check the direction of the acceleration 
        if(length(v_ref) > 2 && v_ref(end)-v_ref(end-1) < 0)
            a_temp = -1 * a_temp;
        end 
        a_ref(end+1) = a_temp;

        % calculate reference angular velocity
        w_ref(end+1) = (ay * vx - ax * vy) / (vx.^2 + vy.^2);

              
    elseif(tPoints(2) <= t && t < tPoints(3))
        x_ref(end+1) = calc_position_shifted(xCoefs(:,2),t,tPoints(2));
        y_ref(end+1) = calc_position_shifted(yCoefs(:,2),t,tPoints(2));

        vx = calc_velocity_shifted(xCoefs(:,2),t,tPoints(2));
        vy = calc_velocity_shifted(yCoefs(:,2),t,tPoints(2));
        
        v_ref(end+1) = hypot(vx,vy);
        theta_ref(end+1) = atan2d(vy,vx);

        % calculate acceleration components
        ax = calc_acceleration_shifted(xCoefs(:,2),t,tPoints(2));
        ay = calc_acceleration_shifted(yCoefs(:,2),t,tPoints(2));
        a_temp = hypot(ax,ay);

        % check the direction of the acceleration 
        if(length(v_ref) > 2 && v_ref(end) - v_ref(end-1) < 0)
            a_temp = -1 * a_temp;
        end 
        a_ref(end+1) = a_temp;

        % calculate reference angular velocity
        w_ref(end+1) = (ay * vx - ax * vy) / (vx.^2 + vy.^2);

        
    elseif(tPoints(3) <= t && t < tPoints(4))
        x_ref(end+1) = calc_position_shifted(xCoefs(:,3),t,tPoints(3));
        y_ref(end+1) = calc_position_shifted(yCoefs(:,3),t,tPoints(3));

        vx = calc_velocity_shifted(xCoefs(:,3),t,tPoints(3));
        vy = calc_velocity_shifted(yCoefs(:,3),t,tPoints(3));
        
        v_ref(end+1) = hypot(vx,vy);
        theta_ref(end+1) = atan2d(vy,vx);

        % calculate acceleration components
        ax = calc_acceleration_shifted(xCoefs(:,3),t,tPoints(3));
        ay = calc_acceleration_shifted(yCoefs(:,3),t,tPoints(3));
        a_temp = hypot(ax,ay);

        % check the direction of the acceleration 
        if(length(v_ref) > 2 && v_ref(end) - v_ref(end-1) < 0)
            a_temp = -1 * a_temp;
        end 
        a_ref(end+1) = a_temp;

        % calculate reference angular velocity
        w_ref(end+1) = (ay * vx - ax * vy) / (vx.^2 + vy.^2);

    elseif(tPoints(4) <= t && t < tPoints(5))
        x_ref(end+1) = calc_position_shifted(xCoefs(:,4),t,tPoints(4));
        y_ref(end+1) = calc_position_shifted(yCoefs(:,4),t,tPoints(4));

        vx = calc_velocity_shifted(xCoefs(:,4),t,tPoints(4));
        vy = calc_velocity_shifted(yCoefs(:,4),t,tPoints(4));
        
        v_ref(end+1) = hypot(vx,vy);
        theta_ref(end+1) = atan2d(vy,vx);

        % calculate acceleration components
        ax = calc_acceleration_shifted(xCoefs(:,4),t,tPoints(4));
        ay = calc_acceleration_shifted(yCoefs(:,4),t,tPoints(4));
        a_temp = hypot(ax,ay);

        % check the direction of the acceleration 
        if(length(v_ref) > 2 && v_ref(end) - v_ref(end-1) < 0)
            a_temp = -1 * a_temp;
        end 
        a_ref(end+1) = a_temp;

        % calculate reference angular velocity
        w_ref(end+1) = (ay * vx - ax * vy) / (vx.^2 + vy.^2);

    elseif(tPoints(5) <= t && t < tPoints(6))
        x_ref(end+1) = calc_position_shifted(xCoefs(:,5),t,tPoints(5));
        y_ref(end+1) = calc_position_shifted(yCoefs(:,5),t,tPoints(5));

        vx = calc_velocity_shifted(xCoefs(:,5),t,tPoints(5));
        vy = calc_velocity_shifted(yCoefs(:,5),t,tPoints(5));
        
        v_ref(end+1) = hypot(vx,vy);
        theta_ref(end+1) = atan2d(vy,vx);

        % calculate acceleration components
        ax = calc_acceleration_shifted(xCoefs(:,5),t,tPoints(5));
        ay = calc_acceleration_shifted(yCoefs(:,5),t,tPoints(5));
        a_temp = hypot(ax,ay);

        % check the direction of the acceleration 
        if(length(v_ref) > 2 && v_ref(end) - v_ref(end-1) < 0)
            a_temp = -1 * a_temp;
        end 
        a_ref(end+1) = a_temp;

        % calculate reference angular velocity
        w_ref(end+1) = (ay * vx - ax * vy) / (vx.^2 + vy.^2);

    elseif(tPoints(6) <= t && t < tPoints(7))
        x_ref(end+1) = calc_position_shifted(xCoefs(:,6),t,tPoints(6));
        y_ref(end+1) = calc_position_shifted(yCoefs(:,6),t,tPoints(6));

        vx = calc_velocity_shifted(xCoefs(:,6),t,tPoints(6));
        vy = calc_velocity_shifted(yCoefs(:,6),t,tPoints(6));
        
        v_ref(end+1) = hypot(vx,vy);
        theta_ref(end+1) = atan2d(vy,vx);

        % calculate acceleration components
        ax = calc_acceleration_shifted(xCoefs(:,6),t,tPoints(6));
        ay = calc_acceleration_shifted(yCoefs(:,6),t,tPoints(6));
        a_temp = hypot(ax,ay);

        % check the direction of the acceleration 
        if(length(v_ref) > 2 && v_ref(end) - v_ref(end-1) < 0)
            a_temp = -1 * a_temp;
        end 
        a_ref(end+1) = a_temp;

        % calculate reference angular velocity
        w_ref(end+1) = (ay * vx - ax * vy) / (vx.^2 + vy.^2);

    else
        x_ref(end+1) = calc_position_shifted(xCoefs(:,7),t,tPoints(7));
        y_ref(end+1) = calc_position_shifted(yCoefs(:,7),t,tPoints(7));

        vx = calc_velocity_shifted(xCoefs(:,7),t,tPoints(7));
        vy = calc_velocity_shifted(yCoefs(:,7),t,tPoints(7));
        
        v_ref(end+1) = hypot(vx,vy);
        theta_ref(end+1) = atan2d(vy,vx);

        % calculate acceleration components
        ax = calc_acceleration_shifted(xCoefs(:,7),t,tPoints(7));
        ay = calc_acceleration_shifted(yCoefs(:,7),t,tPoints(7));
        a_temp = hypot(ax,ay);

        % check the direction of the acceleration 
        if(length(v_ref) > 2 && v_ref(end) - v_ref(end-1) < 0)
            a_temp = -1 * a_temp;
        end 
        a_ref(end+1) = a_temp;

        % calculate reference angular velocity
        w_ref(end+1) = (ay * vx - ax * vy) / (vx.^2 + vy.^2);

    end
end 
x_ref = x_ref';
y_ref = y_ref';
theta_ref = deg2rad(theta_ref)';

v_ref = v_ref';
a_ref = a_ref';
w_ref = w_ref';
end