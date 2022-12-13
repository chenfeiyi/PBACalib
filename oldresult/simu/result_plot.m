clc;
clear;
addpath("../boxplot2/");
addpath("../boxplot2/boxplot2/");

simu1 = load("simu0.01.mat");
simu2 = load("simu0.02.mat");
simu3 = load("simu0.03.mat");
xticks = [4:1:10];
figure;
position_O = 4:4:28;
boxplot(simu3.t_errs,'colors',[54,96,136]./255,'positions',position_O,...
        'width',1,'symbol', '','Labels',xticks,'Whisker',1);
hold on;
position_O = position_O+1;
boxplot(simu2.t_errs,'colors',[144,3,95]./255,'positions',position_O,...
    'width',1,'symbol', '','Labels',xticks,'Whisker',1);
position_O = position_O+1;
boxplot(simu1.t_errs,'colors',[198,102,2]./255,'positions',position_O,...
        'width',1,'symbol', '','Labels',xticks,'Whisker',1);
hold on;
a=plot(0:0,'Color',[54,96,136]./255);
b=plot(0:0,'Color',[198,102,2]./255);
c=plot(0:0,'Color',[144,3,95]./255);
hold on;
set(gca,'fontname','Times','fontsize',12);
legend('k=1','k=2','k=3','FontSize',16);
xlabel("Pose Number",'FontSize',16);
ylabel("Error(meter)",'FontSize',16);
title("Translation",'FontSize',20);

figure;
position_O = 4:4:28;
boxplot(simu3.r_errs,'colors',[54,96,136]./255,'positions',position_O,...
        'width',1,'symbol', '','Labels',xticks);
hold on;
position_O = position_O+1;
boxplot(simu2.r_errs,'colors',[144,3,95]./255,'positions',position_O,...
    'width',1,'symbol', '','Labels',xticks);
position_O = position_O+1;
boxplot(simu1.r_errs,'colors',[198,102,2]./255,'positions',position_O,...
        'width',1,'symbol', '','Labels',xticks);
hold on;
a=plot(0:0,'Color',[54,96,136]./255);
b=plot(0:0,'Color',[198,102,2]./255);
c=plot(0:0,'Color',[144,3,95]./255);
hold on;
set(gca,'fontname','Times','fontsize',12);
legend('k=1','k=2','k=3','FontSize',16);
xlabel("Pose Number",'FontSize',16);
ylabel("Error(degree)",'FontSize',16);
title("Rotation",'FontSize',20);
