function [x_ref,y_ref,theta_ref,v,xm1,ym1]=generate_cubic(dT,tsim)
% Author: Jingxue Jiang 
% ----------------------- % 
%  Simulation parameters  %
% ----------------------- %
% parameters include: 
% 1) sampling frequency (Hz) 
% 2) sampling period (s)
% 3) simulation time (s) 

% ---Start of user-controlled section--- % 

% Fs = 10; 
Ts = dT;    
Tsim = tsim; 

% ----End of user-controlled section---- % 

% ----------------------------------------- %
%  Establish trajecotry boundry conditions  %
% ----------------------------------------- %
% Boundry conditions includes: 
% - xi,yi: the position component at t = 0;
% - vxi,vyi: the velocity component at t = 0;
% - axi,ayi: the acceleration component at t = 0;
% - xf,yf: the desired position component at t = Tsim;
% - vxf,vyf: the desired velocity component at t = Tsim;
% - axf,ayf: the desired acceleration component at t = Tsim; 

% ---Start of user-controlled section--- % 

pi = [5 5];         % [xi yi] 
vi = 0.01;           % consists of vxi,vyi 
ai = 0.01;           % consists of axi,ayi      
thetai = 45;        % orientation of the starting point, used to compute vxi,vyi,axi,ayi

pf = [5 15];        % [xf yf]
vf = 0.01;
af = 0.01; 
thetaf = 90;

% ----End of user-controlled section---- % 

xi = pi(1);
yi = pi(2);
vxi = vi * cosd(thetai);
vyi = vi * sind(thetai);
axi = ai * cosd(thetai);
ayi = ai * sind(thetai);

xf = pf(1);
yf = pf(2);
vxf = vf * cosd(thetaf);
vyf = vf * sind(thetaf);
axf = af * cosd(thetaf);
ayf = af * sind(thetaf); 

% Safety Check: when the vehicle stops, vxf, vyf, axf, ayf = 0
% if((vxf | vyf | axf | ayf) ~= 0)
%     msg = 'Check velocity and acceleration profile of the destination point';
%     error(msg)
% end

% ------------------------------------------- %
%  Establish intermediate Points conditions   %
% ------------------------------------------- %
% Intermediate points conditions include: 
% - num: number of intermediate points (num can be 0,1 or 2)
% - opmode: manual mode or auto mode ( see notes below) 
% - pm: position of the intermediate point  
% - Tm: Tm is the specified time to reach the intermediate point.

% Explaination of the operation modes 
% - manual mode: the user manually choose the desired velocity at the
% intermediate points 
% - auto mode: The program automatically choose the desired velocity at the
% intermediate points in such a way as to cause the acceleration to be
% continuous 

% ---Start of user-controlled section--- % 
num = 1;
opmode = 'auto';
pm1 = [5 7.5];
Tm = Tsim / 2; 
% ----End of user-controlled section---- % 

xm1 = pm1(1);
ym1 = pm1(2);

% ------------------------------------- %
%  Find cubic polynomial trajecotry     %
% ------------------------------------- %

% pre-compute the coefficients of the polynomials  
[xs1_coe,xs2_coe] = compute_cubic_coeffs_auto(xi,vxi,xm1,xf,vxf,Tsim,Tm);
[ys1_coe,ys2_coe] = compute_cubic_coeffs_auto(yi,vyi,ym1,yf,vyf,Tsim,Tm);

% create empty arrays to store data 
[x,y,heading,v,a] = deal([],[],[],[],[]);

% from t = 0 to t = Tsim, compute the position of the vehicle at a
% frequency of 10 Hz
for t = 0:Ts:Tsim
    if(t < Tm)
        x(end+1) = calc_position(xs1_coe,t);
        y(end+1) = calc_position(ys1_coe,t);

        vx = calc_velocity(xs1_coe,t);
        vy = calc_velocity(ys1_coe,t);
        v(end+1) = hypot(vx,vy);

        % check if velocity exceeds limits 
        % code here 

        heading(end+1) = atan2d(vy,vx);

        ax = calc_acceleration(xs1_coe,t);
        ay = calc_acceleration(ys1_coe,t);
        a_temp = hypot(ax,ay);

        % check the direction of the acceleration
        if(length(v)>2 && v(end)-v(end-1) < 0)
            a_temp = -1 * a_temp;
        end 

        a(end+1) = a_temp;    

    else
        x(end+1) = calc_position_shifted(xs2_coe,t,Tm);
        y(end+1) = calc_position_shifted(ys2_coe,t,Tm);

        vx = calc_velocity_shifted(xs2_coe,t,Tm);
        vy = calc_velocity_shifted(ys2_coe,t,Tm);
        v(end+1) = hypot(vx,vy);

        % check if velocity exceeds limits 
        % code here 
        heading(end+1) = atan2d(vy,vx);

        ax = calc_acceleration_shifted(xs2_coe,t,Tm);
        ay = calc_acceleration_shifted(ys2_coe,t,Tm);

        % check the direction of the acceleration
        a_temp = hypot(ax,ay);
        if(length(v)>2 && v(end)-v(end-1) < 0)
            a_temp = -1 * a_temp;
        end

        a(end+1) = a_temp;

    end
end

x_ref = x';
y_ref = y';
theta_ref = deg2rad(heading)';

end 


















