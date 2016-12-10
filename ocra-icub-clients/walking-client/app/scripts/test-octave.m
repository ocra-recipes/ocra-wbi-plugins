%% PLOT

close all;
%% Load data
load("currentComLinVel.txt");
load("refLinVel.txt");
load("twistRoot.txt");
load("intComPositionRef.txt");
load("currentComPos.txt");
%%
t = currentComLinVel(:,1);
figure;
subplot 211;
hold on;
plot(t, refLinVel(:,3), ":","LineWidth",2);
plot(t, currentComLinVel(:,3), "LineWidth",2);
legend("COM Lin Vel Ref", "COM Lin Vel Actual");
axis tight;

subplot 212;
hold on;
plot(t, intComPositionRef(:,3), ":", "LineWidth", 2);
plot(t, currentComPos(:,3), "LineWidth", 2);
legend("CoM Reference", "CoM Actual");
axis tight;
%% DELETE ALL FILES
delete("currentComLinVel.txt");
delete("refLinVel.txt");
delete("intComPositionRef.txt");
delete("twistRoot.txt");
delete("currentComPos.txt");

% tLength = size(twistRoot,1);
% t2 = linspace(0,0.010*tLength,tLength);
% plot(t2, twistRoot(:,5));
% axis tight;
