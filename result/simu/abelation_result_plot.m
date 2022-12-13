% clc;
% clear;
% addpath("../boxplot2/");
% addpath("../boxplot2/boxplot2/");

simu3 = load("noise3.mat");
figure;
position_O = 4:4:28;
% position_O = position_O + 8;
boxplot(simu3.t_errs1,'colors',[54,96,136]./255,'positions',position_O,...
        'width',1,'symbol', '','Whisker',1);
hold on;
position_O = position_O+1;
boxplot(simu3.t_errs,'colors',[144,3,95]./255,'positions',position_O,...
    'width',1,'symbol', '','Whisker',1);
hold on;
a=plot(0:0,'Color',[54,96,136]./255);
b=plot(0:0,'Color',[198,102,2]./255);
hold on;
set(gca,'fontname','Times','fontsize',16);
legend('Init','Optm','FontSize',16);
xticks([5:4:29]);
xticklabels({4,5,6,7,8,9,10});
xlabel("Pose Number",'FontSize',20);
ylabel("Error(meter)",'FontSize',20);
title("Translation",'FontSize',26);

figure;
position_O = 4:4:28;
boxplot(simu3.r_errs1,'colors',[54,96,136]./255,'positions',position_O,...
        'width',1,'symbol', '');
hold on;
position_O = position_O+1;
boxplot(simu3.r_errs,'colors',[144,3,95]./255,'positions',position_O,...
    'width',1,'symbol', '');
hold on;
a=plot(0:0,'Color',[54,96,136]./255);
b=plot(0:0,'Color',[198,102,2]./255);
hold on;
set(gca,'fontname','Times','fontsize',16);
xticks([5:4:29]);
xticklabels({4,5,6,7,8,9,10});
legend('Init','Optm','FontSize',16);
xlabel("Pose Number",'FontSize',20);
ylabel("Error(degree)",'FontSize',20);
title("Rotation",'FontSize',26);
