% clear screen
clc
clear
close all

%   global
global path_type problem_type

%   problem names
problem_name = {'Without noise, disturbance, ...';...
    'Change in system parameters';'Measurement noise';...
    'Control input saturation';'External disturbance'};

for path_type = 1:2
    for problem_type = 1:5
        %   solve robot equations
        switch path_type
            case 1
                [t,Q] = ode45(@robot_model,0:0.1:12,[1.1;0;0;0;0;0]);
                path_name = 'Circle path - ';
            case 2
                [t,Q] = ode45(@robot_model,0:0.1:12,[-0.1;-0.1;0;0;0;0]);
                path_name = 'Square path - ';
        end

        %   reference path for plot
        xd = zeros(size(t));
        yd = zeros(size(t));
        for i = 1:length(t)
            [xd(i,1),~,~,yd(i,1),~,~] = ref_path(path_type,t(i));
        end

        %   plot the results
        figure, hold on
        plot(xd,yd,Q(:,1),Q(:,2),'r--')
        xlabel('xc')
        ylabel('yc')
        legend('Desired','Real','location','best')
        title([path_name,problem_name{problem_type}])
        figure
        plot(t,Q(:,3))
        xlabel('time (s)')
        ylabel('theta angle')
        title([path_name,problem_name{problem_type}])
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   other functions
function u_fuzz = fuzzy_controller(x1,x2)
mu_P = @(x)(1/(1+exp(-30*x)));
mu_N = @(x)(1/(1+exp(30*x)));

u_fuzz = (-5*mu_P(x1)*mu_P(x2)+5*mu_N(x1)*mu_N(x2))/...
    (mu_P(x1)*mu_P(x2)+mu_P(x1)*mu_N(x2)+mu_N(x1)*mu_P(x2)+mu_N(x1)*mu_N(x2));
end

function [xd,xd_dot,xd_dot_dot,yd,yd_dot,yd_dot_dot] = ref_path(type,t)
switch type
    case 1
        a = 0;
        b = 1;
        c = 0;
        wr = 0.5236;
        xd = a+b*cos(wr*t);
        yd = c+b*sin(wr*t);
        xd_dot = -b*wr*sin(wr*t);
        yd_dot = b*wr*cos(wr*t);
        xd_dot_dot = -b*wr^2*cos(wr*t);
        yd_dot_dot = -b*wr^2*sin(wr*t);
    case 2
        if (t <= 3)
            xd = t;
            yd = 0;
            xd_dot = 1;
            yd_dot = 0;
        elseif (t > 3 && t <= 6)
            xd = 3;
            yd = t-3;
            xd_dot = 0;
            yd_dot = 1;
        elseif (t > 6 && t <= 9)
            xd = 9-t;
            yd = 3;
            xd_dot = -1;
            yd_dot = 0;
        else
            xd = 0;
            yd = 12-t;
            xd_dot = 0;
            yd_dot = -1;
        end
        xd_dot_dot = 0;
        yd_dot_dot = 0;
end
end

function Q_dot = robot_model(t,Q)
%   global
global path_type problem_type

%   Q members
xc = Q(1);
xc_dot = Q(4);
yc = Q(2);
yc_dot = Q(5);
theta = Q(3);
theta_dot = Q(6);

%   robot parameters
m = 10;
I = 5;
R = 0.15;
r = 0.05;
d = 0.1;

%   change in system parameters
if (problem_type == 2)
    if (t >= 3)
        m = 3*10;
        I = 3*5;
        R = 3*0.15;
        r = 3*0.05;
        d = 3*0.1;
    end
end

%   measurement noise
noise = [0;0];
if (problem_type == 3)
    noise = 0.1*randn(2,1);
end

%   M, V, G, B, A
M = [m,0,m*d*sin(theta)
    0,m,-m*d*cos(theta)
    m*d*sin(theta),-m*d*cos(theta),I];
V = [m*d*theta_dot^2*cos(theta)
    m*d*theta_dot^2*sin(theta);
    0];
G = [0;0;0];
B = 1/r*[cos(theta),cos(theta)
    sin(theta),sin(theta)
    R,-R];
A = [-m*sin(theta)*(xc_dot*cos(theta)+yc_dot*sin(theta))*theta_dot
    m*cos(theta)*(xc_dot*cos(theta)+yc_dot*sin(theta))*theta_dot
    -d*m*(xc_dot*cos(theta)+yc_dot*sin(theta))*theta_dot];

%   reference paths
[xd,xd_dot,xd_dot_dot,yd,yd_dot,yd_dot_dot] = ref_path(path_type,t);

%   error
ex = xd - xc;
ey = yd - yc;
ex_dot = xd_dot - xc_dot;
ey_dot = yd_dot - yc_dot;

%   fuzzy control
u_fuzz_x = fuzzy_controller(ex,ex_dot);
u_fuzz_y = fuzzy_controller(ey,ey_dot);

%   supervisor control
u_s_x = supervisor_control(-ex,-ex_dot,xd_dot_dot,u_fuzz_x);
u_s_y = supervisor_control(-ey,-ey_dot,yd_dot_dot,u_fuzz_y);

%   total control input
ux = total_control(u_fuzz_x,u_s_x,[ex;ex_dot]);
uy = total_control(u_fuzz_y,u_s_y,[ey;ey_dot]);

%   conver u to tau
tau_matrix = (M^-1)*B;
tau = (tau_matrix(1:2,1:2)^-1)*[ux;uy];
tau = tau + noise;

%   control input saturation
sat_tau = 35;
if (problem_type == 4)
    for i = 1:2
        if (tau(i) > sat_tau)
            tau(i) = sat_tau;
        elseif (tau(i) < -sat_tau)
            tau(i) = -sat_tau;
        end
    end
end

%   external disturbance
ext_d = 0;
if (problem_type == 4)
    ext_d = 0.1*sin(t);
end

%   differential equations
Q_dot = [Q(4:6);(M^-1)*(-V - G + B*tau + A)];
Q_dot(4) = Q_dot(4) + ext_d;
Q_dot(5) = Q_dot(5) + ext_d;

%   time
disp(t)
end

function u_s = supervisor_control(e,e_dot,xd_dot_dot,u_fuzz)
lambda = 10;
eta = 10;
phi = 1e-3;

fhat = 2;
F = 2;

s = e_dot + lambda*e;

u_s = -fhat + xd_dot_dot - lambda*e_dot - (eta+F)*saturation(s,phi) - u_fuzz;

function s_sat = saturation(s,phi)
    if (s/phi <= -1)
        s_sat = -1;
    elseif (s/phi) > 1
        s_sat = 1;
    else
        s_sat = s/phi;
    end
end
end

function u = total_control(u_fuzz,u_s,x)
a = 0.005;
Mx = 2*a;

if norm(x) < a
    I_star = 0;
elseif (norm(x) >= a && norm(x) < Mx)
    I_star = (norm(x)-a)/(Mx-a);
else
    I_star = 1;
end

u = u_fuzz + I_star*u_s;
end
