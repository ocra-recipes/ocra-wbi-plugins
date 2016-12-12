close all;
clear all;

saveEnabled = input("Enter number 1 to save: ");
plotName = input("Name of the file: ");
%% PLOT

%% Load data
load("currentComLinVel.txt");
load("refLinVel.txt");
load("intComPositionRef.txt");
load("currentComPos.txt");
%%
t = currentComLinVel(:,1);
figure;
subplot(211);
hold on;
plot(t, refLinVel(:,3), ":","LineWidth",3);
plot(t, currentComLinVel(:,3), "LineWidth",3);
legend("COM Lin Vel Ref", "COM Lin Vel Actual");
ylim([0 0.013]);
grid on;

subplot(212);
hold on;
plot(t, intComPositionRef(:,3), ":", "LineWidth", 3);
plot(t, currentComPos(:,3), "LineWidth", 3);
legend("CoM Reference", "CoM Actual");
ylim([-0.07 -0.045]);
grid on;

if (saveEnabled)
 print([plotName ".png"], '-color', '-dpng', '-landscape', "-S1800,1000")
end

deleteData = input("Do you wanna delete these data? ");

if (deleteData)
%% DELETE ALL FILES
  delete("currentComLinVel.txt");
  delete("refLinVel.txt");
  delete("intComPositionRef.txt");
  delete("currentComPos.txt");
end

% tLength = size(twistRoot,1);
% t2 = linspace(0,0.010*tLength,tLength);
% plot(t2, twistRoot(:,5));
% axis tight;
