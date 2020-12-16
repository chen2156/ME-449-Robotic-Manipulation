TseInitial = [0, 0, 1, 0;0, 1, 0, 0;-1, 0, 0, 0.5;0, 0, 0, 1];
TscInitial = [1, 0, 0, 0.5;0, 1, 0, 0;0, 0, 1, 0.025;0, 0, 0, 1];
Tscfinal = [0, 1, 0, 0;-1, 0, 0, -0.5;0, 0, 1, 0.025;0, 0, 0, 1];
Tcegrasp = [0, 0, 1, 0;0, 1, 0, 0;-1, 0, 0, 0;0, 0, 0, 1];
TceStandoff = Tcegrasp;
k = 1;
cumulatedTraj = TrajectoryGenerator(TseInitial, TscInitial, Tscfinal, Tcegrasp, TceStandoff, k);

% CurrentConfig = [0;0;0;0;0;0.2;-1.6;0;0;0;0;0;0];
% Tse = [0.170, 0, 0.985, 0.387;0, 1, 0, 0;-0.985, 0, 0.170, 0.570; 0, 0, 0, 1];
% Tsed = [0, 0, 1, 0.5;0, 1, 0, 0;-1, 0, 0, 0.5;0, 0, 0, 1];
% Tsednext = [0, 0, 1, 0.6;0, 1, 0, 0;-1, 0, 0, 0.3;0, 0, 0, 1];
% Kp = eye(6);
% Ki = zeros(6);
% dt = 0.01;
% 
% global Xerrdtsum
% Xerrdtsum = zeros(6, 1);
% [result, Xerr] = FeedbackControl(Tse, Tsed, Tsednext, Kp, Ki, dt, CurrentConfig)

% currentConfig = [0;0;0;0;0;0;0;0;0;0;0;0];
% newMatrix = [currentConfig];
% jointWheelSpeed = [0;0;0; 0; 0; 10; 10; 10; 10];
% dt = 0.01;
% maxspeed = 5;
% for i = 1:100
%     newConfig = NextState(currentConfig, jointWheelSpeed, dt, maxspeed);
%     newMatrix = [newMatrix, newConfig];
%     currentConfig = newConfig;
% end    
% 
% writematrix(newMatrix', 'NextState.csv')


[r, c] = size(cumulatedTraj);
global Xerrdtsum
Xerrdtsum = [0.1;0.1; 0.001; -0.1; -0.001; 0.01];
Kp = eye(6) * 5;
Ki = eye(6) * 0.2;
phi=0;
x=0;
y=0;
w1=0;
w2=0;
w3=0;
w4=0;
j1=0;
j2=0;
j3=0.2;
j4=-1.6;
j5=0;
dt = 0.01;

B1 = [0;0;1;0;0.033;0];
B2 = [0;-1;0;-0.5076;0;0];
B3 = [0;-1;0;-0.3526;0;0];
B4 = [0;-1;0;-0.2176;0;0];
B5 = [0;0;1;0;0;0];

Blist = [B1, B2, B3, B4, B5];
M = [1, 0, 0, 0.033; 0, 1, 0, 0; 0, 0, 1, 0.6546;0, 0, 0, 1];

%thetalist0 = [j1;j2;j3;j4;j5];
%eomg = 0.01;
%ev = 0.001;
%[thetalist, success] = IKinBody(Blist, M, TseInitial, thetalist0, eomg, ev);


Tb0 = [1, 0, 0, 0.1662;0, 1, 0, 0;0, 0, 1, 0.0026;0, 0, 0, 1];

maxspeed = 1000;



currentConfig = [phi;x;y;j1;j2;j3;j4;j5;w1;w2;w3;w4;0];
configs = [currentConfig];

XerrCum = [];
for i = 1:r-1
    r11 = cumulatedTraj(i, 1);
    r12 = cumulatedTraj(i, 2); 
    r13 = cumulatedTraj(i, 3);
    r21 = cumulatedTraj(i, 4);
    r22 = cumulatedTraj(i, 5);
    r23 = cumulatedTraj(i, 6);
    r31 = cumulatedTraj(i, 7);
    r32 = cumulatedTraj(i, 8);
    r33 = cumulatedTraj(i, 9);
    px = cumulatedTraj(i, 10);
    py = cumulatedTraj(i, 11);
    pz = cumulatedTraj(i, 12);
    gripperState = cumulatedTraj(i, 13);
    
    Tsed = [r11, r12, r13, px;r21, r22, r23, py;r31, r32, r33, pz;0,0,0,1];
    
    r11 = cumulatedTraj(i+1, 1);
    r12 = cumulatedTraj(i+1, 2); 
    r13 = cumulatedTraj(i+1, 3);
    r21 = cumulatedTraj(i+1, 4);
    r22 = cumulatedTraj(i+1, 5);
    r23 = cumulatedTraj(i+1, 6);
    r31 = cumulatedTraj(i+1, 7);
    r32 = cumulatedTraj(i+1, 8);
    r33 = cumulatedTraj(i+1, 9);
    px = cumulatedTraj(i+1, 10);
    py = cumulatedTraj(i+1, 11);
    pz = cumulatedTraj(i+1, 12);
    
    Tsednext = [r11, r12, r13, px;r21, r22, r23, py;r31, r32, r33, pz;0,0,0,1];
    
    thetalist = [currentConfig(4);currentConfig(5);currentConfig(6);currentConfig(7);currentConfig(8)]; 
    T0e = FKinBody(M, Blist, thetalist);
    chassis_phi = currentConfig(1);
    chassis_x = currentConfig(2);
    chassis_y = currentConfig(3);
    
    Tsb = [cos(chassis_phi), -sin(chassis_phi), 0, chassis_x;sin(chassis_phi), cos(chassis_phi), 0, chassis_y;0, 0, 1, 0.0963;0, 0, 0, 1];
    
    Ts0 = Tsb * Tb0;
    Tinit = Ts0 * T0e;
    
    [result, Xerr] = FeedbackControl(Tinit, Tsed, Tsednext, Kp, Ki, dt, currentConfig);
    XerrCum = [XerrCum, Xerr];
    jointWheel = [result(5:9);result(1:4)];
    newConfig = NextState(currentConfig(1:12), jointWheel, dt, maxspeed);
    configs = [configs, [newConfig; gripperState]];  
    currentConfig = newConfig;
end

figure
[r1, c1] = size(XerrCum);
x = 1:c1;
y1 = XerrCum(1, :);
plot(x,y1)
hold on 

y2 = XerrCum(2, :);
plot(x,y2)
hold on 

y3 = XerrCum(3, :);
plot(x,y3)
hold on 

y4 = XerrCum(4, :);
plot(x,y4)
hold on 

y5 = XerrCum(5, :);
plot(x,y5)
hold on 

y6 = XerrCum(6, :);
plot(x,y6)
hold off
legend('Error1', 'Error2', 'Error3', 'Error4', 'Error5', 'Error6')

diary newTasklogfile.txt
diary on
fprintf("Writing error plot data.\n")

saveas(gcf,'newTaskfigure.pdf')


fprintf("Generating animation csv file.\n")
writematrix(configs', 'newTaskConfiguration.csv')
writematrix(XerrCum', 'newTaskXerr.csv')

fprintf("Done\n");

diary off

