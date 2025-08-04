clear; close all; clc;
try
    fclose(instrfindall);
catch
end
rosshutdown;

%% Load Classes

OPT = OptiTrack;
OPT.Initialize;

% Joystick
J = JoyControl;

A = ArDrone();

rb = OPT.RigidBody;
if rb(1).isTracked
    A = getOptData(rb(1),A);
    %             A.pPos.X
end

%% Initialize ROS
rosinit('192.168.0.100');
pub = rospublisher('/cf1/cmd_vel','geometry_msgs/Twist');
sub_isFlying = rossubscriber('/cf1/crazyflieIsFlying','std_msgs/Bool');

msg = rosmessage(pub);
cmd = zeros(1,4);

pause(4)

%% Initialize Timers and CrazyFlie Parameters
t  = tic;
T_exp = 1200; % tempo de experimento
T_run = 1/50; % período de amostragem do experimento
t_run = tic;

idle_time = 2;
takeoff_time = idle_time + 6;

% crazyflie
thrustMax = 60000;
tilt_max = deg2rad(60);
psi_ref_max = deg2rad(100);
g = 9.81;
a_max = 18.86; %%Motor novo
% a_max = 15; %%Motor velho
% conversao_digital = thrustMax/a_max;

theta_ref = 0;
phi_ref = 0;
psi_ref = 0;
thrust = 11000;

data = [];

pouso = 0;
centralizar = 0;

X_inicial = A.pPos.X(1:3);
corpo_old = A.pPos.X(1:3);
counter_perdaCorpo = 0;

%% %%%%%%%%%%%%%%%%%%%% Emergency button %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
    'String', 'land', ...
    'Callback', 'btnEmergencia=1', ...
    'Position', [50 50 400 300]);

%% Control Gains

lambda = 1;

Kp = diag([lambda*4 lambda*4 5]);
Kd = diag([lambda*2 lambda*2 5]);

Ku = diag([g g]);

K_amax = 1;
Kp_amax = 6;

k_psi = 2.5;

t_end_takeoff=0;

dX_old = [0 0 0]';
alpha_vel = 0.4;

flag = 2;

while toc(t) < (T_exp)
    if toc(t_run) > T_run
        t_run = tic;
        t_atual = toc(t);
        
        
        
        %% Obter os dados dos sensores
        rb = OPT.RigidBody;
        if rb(1).isTracked
            A = getOptData(rb(1),A);
        end
        
        %%
        phi = A.pPos.X(4); theta = A.pPos.X(5); psi = A.pPos.X(6);
        X = A.pPos.X(1:3);
        dX = A.pPos.X(7:9);
        
        dX_filt = alpha_vel*dX_old + (1 - alpha_vel)*dX;
        dX_old = dX_filt;
        dX = dX_filt;

        dphi = A.pPos.X(10); dtheta = A.pPos.X(11); dpsi = A.pPos.X(12);
        
        %%
        
        R = [cos(psi), -sin(psi);...
            sin(psi), cos(psi)];


        %% Take off
        if t_atual < takeoff_time
            Xd = [X_inicial(1) X_inicial(2) 1]';
            dXd = [0 0 0]';
            ddXd = [0 0 0]';
            psi_d = psi;
            dpsi_d = 0;
            t_end_takeoff=t_atual;
        end

        if flag == 1
        trajName = 'Posicionamento_0';
        if t_atual > takeoff_time
            Xd = [0 0 1]';
            dXd = [0 0 0]';
            ddXd = [0 0 0]';
            
            psi_d = 0;
            dpsi_d = 0;
        end
        
        else if flag == 2
        trajName = 'LemniscataX';
    
        T = 6;
        w = 2*pi/T;
        
        rX = 1; rY = 1;

        if t_atual > takeoff_time
            Xd = [rX*sin(w*(t_atual-t_end_takeoff));
                rY*sin(2*w*(t_atual-t_end_takeoff));
                1];
            dXd = [rX*w*cos(w*(t_atual-t_end_takeoff));
                rY*2*w*cos(2*w*(t_atual-t_end_takeoff));
                0];
            ddXd = [-rX*w*w*sin(w*(t_atual-t_end_takeoff));
                -rY*2*w*2*w*sin(2*w*(t_atual-t_end_takeoff));
                0];
         psi_d = 0;
            dpsi_d = 0;
        end
            end
        end

        if (t_atual-t_end_takeoff) > 60
             centralizar = 1;
        end

        
        %% Centralizar
        if centralizar == 1
            Xd = [0 0 1]';
            dXd = [0 0 0]';
            ddXd = [0 0 0]';
            
            psi_d = 0;
            dpsi_d = 0;
        end

        %% Controller
        
        Xtil = Xd - X;
        dXtil = dXd - dX;

        ddXref = ddXd + Kd*dXtil + Kp*Xtil;
        
        if pouso == 1
            ddXref(3) = 3*-.3;
            if A.pPos.X(3) < 0.1
                disp('Pousou')
                break
            end
        end

        u = (R*Ku)\(ddXref(1:2));

        u = min(max(u,-tilt_max),tilt_max);
        
%         ddXref(1:2) = min(max(ddXref(1:2),-8),8); % ~
        %% a_max adaptation
        if t_atual > takeoff_time
            %             a_max_dot = -K_amax*Xref(3)*(dXtil(3) + Kp_amax*Xtil(3));
            a_max_dot = -(Kp_amax*Xtil(3));
            a_max = a_max + a_max_dot*T_run;
        end
        
        a_max = min(max(a_max,16),21); %Motor novo
%         a_max = min(max(a_max,13),18); %Motor velho
        
        conversao_digital = thrustMax/a_max;

        thrust = (ddXref(3) + g)/(cos(theta)*cos(phi))*conversao_digital;
        thrust = min(max(thrust,11000),thrustMax);
        
        psi_til = psi_d - psi;
        if abs(psi_til) > pi
            psi_til = psi_til - 2*pi*sign(psi_til);
        end

        psi_ref = dpsi_d + k_psi*(psi_til);
        
        if t_atual < takeoff_time
            psi_ref = 0;
        end

        psi_ref = min(max(psi_ref,-psi_ref_max),psi_ref_max);
        psi_ref = 0;
        
        U = [u' thrust psi_ref]';

%         U([1 2 4]) = zeros(3,1);
        %% Joystick
        mRead(J);
        
        if norm(J.pAnalog(4:5)) > .1
            U(1) = -J.pAnalog(5);
            U(2) = -J.pAnalog(4);
            
            U(1) = U(1)*deg2rad(10);
            U(2) = U(2)*deg2rad(10);
        end
        
        if abs(J.pAnalog(3)) > .1
            U(4) = J.pAnalog(3);
            U(4) = min(max(U(4),-1),1);
            U(4) = U(4)*psi_ref_max;
        end
        
        if J.pDigital(1) == 1
            btnEmergencia = 1;
        end
        
        if J.pDigital(2) == 1
            pouso = 1;
        end
        
        if J.pDigital(3) == 1
            centralizar = 1;
        end
        
        if abs(J.pAnalog(2)) > .1
            U(3) = (.5 + -J.pAnalog(2)*.5)*thrustMax;
            U(3) = min(max(U(3),30000),thrustMax);
        end
        
        
        %% idle
        
        % idle
        if t_atual < idle_time
            U(3) = 12000;
            U([1 2 4]) = zeros(3,1);
        end
        
        
        %%

%         if thrust > .9*thrustMax
%             U(4) = 0;
%         end
        
        cmd = [-U(2)  U(1)  U(4) U(3)];

        msg.Angular.X = cmd(1); msg.Angular.Y = cmd(2); msg.Angular.Z = cmd(3); msg.Linear.Z = cmd(4);
        
%         disp([rad2deg(msg.Angular.X) rad2deg(msg.Angular.Y) rad2deg(msg.Angular.Z) msg.Linear.Z/thrustMax])
        disp(t_atual)

        %% Loss of Body
        
        if sum(A.pPos.X(1:3) - corpo_old) == 0
            counter_perdaCorpo = counter_perdaCorpo + 1;
            disp('Perdeu o corpo')
        else
            counter_perdaCorpo = 0;
        end
        
        if counter_perdaCorpo > 1
%             msg.Angular.X = 0; msg.Angular.Y = 0; msg.Linear.Z = cmd(4); msg.Angular.Z = 0;
        end
        corpo_old =  A.pPos.X(1:3);

        %% %%%%% SEND %%%%% %%
        send(pub,msg)
        
        %% EMERGENCY
        drawnow
        if btnEmergencia ~= 0
            msg.Angular.X = 0;
            msg.Angular.Y = 0;
            msg.Angular.Z = 0;
            msg.Linear.Z = 0;
            send(pub,msg) %NÃO COMENTAR
     
            %             disp('Bebop Landing through Emergency Command ');
            
            % Send 3 times Commands 1 second delay to Drone Land
            for i=1:nLandMsg
                disp("End Land Command");
                
            end
            break;
        end
        
        %% Data
        data = [data; A.pPos.X'  A.pPos.dX'    U'      Xd'      dXd'     ddXd'  psi_d dpsi_d a_max t_atual];
        %              1 - 12     13 - 24   25 - 28  29 - 31  32 - 34  35 - 37   38     39     40    end
        
    end
end

while sub_isFlying.LatestMessage.Data
   msg.Angular.X = 0;
    msg.Angular.Y = 0;
    msg.Angular.Z = 0;
    msg.Linear.Z = 0;
    send(pub,msg)
end

rosshutdown

%% Save Data
path = [pwd '\DataSbai\'];
filename = ['ExpCF' '_Data_' datestr(now,30) '_' trajName '.mat'];
fullname = [path filename];
% save(fullname,'data')

%% Position vs Desired Position
figure();
rateAxis = 1.2;
subplot(311)
hold on;
grid on;
plot(data(:,end),data(:,29));
plot(data(:,end),data(:,1),'--');
xlabel('time (s)');
ylabel('$x$','interpreter','latex');
legend('$x_{des}$','$x$','interpreter','latex');
axis([0 data(end,end) -rateAxis rateAxis])

subplot(312)
hold on;
grid on;
plot(data(:,end),data(:,30));
plot(data(:,end),data(:,2),'--');
xlabel('time (s)');
ylabel('$y$','interpreter','latex');
legend('$y_{des}$','$y$','interpreter','latex');
axis([0 data(end,end) -rateAxis rateAxis])


subplot(313)
hold on;
grid on;
plot(data(:,end),data(:,31));
plot(data(:,end),data(:,3),'--');
xlabel('time (s)');
ylabel('$z$','interpreter','latex');
legend('$z_{des}$','$z$','interpreter','latex');
axis([0 data(end,end) 0 rateAxis])


%% Position Error
figure();
rateAxis = 1.2;
% subplot(311)
hold on;
grid on;

x_til = data(:,29:31) - data(:,1:3);

plot(data(:,end),x_til(:,1),'r');
xlabel('time (s)');
ylabel('$x$','interpreter','latex');

axis([0 data(end,end) -rateAxis rateAxis])

% subplot(312)
plot(data(:,end),x_til(:,2),'g');
grid on;
xlabel('time (s)');
ylabel('$y$','interpreter','latex');

axis([0 data(end,end) -rateAxis rateAxis])

% subplot(313)
hold on;
grid on;
plot(data(:,end),x_til(:,3),'b');
xlabel('time (s)');
ylabel('$z$','interpreter','latex');

axis([0 data(end,end) -rateAxis rateAxis])
%%
figure 
subplot(311)
plot(data(:,end),rad2deg(data(:,4)),'b');
hold on
plot(data(:,end),rad2deg(-data(:,26)),'r');
xlabel('time (s)');
ylabel('roll')
legend('real','desired')
grid on
subplot(312)
plot(data(:,end),rad2deg(data(:,5)),'b');
hold on
plot(data(:,end),rad2deg(data(:,25)),'r');
xlabel('time (s)');
ylabel('pitch')
legend('real','desired')
grid on
subplot(313)
plot(data(:,end),rad2deg(data(:,6)),'b');
hold on
plot(data(:,end),rad2deg(data(:,28)),'r');
xlabel('time (s)');
ylabel('yaw')
legend('real','desired')
grid on