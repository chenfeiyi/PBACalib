% clc;
% clear;
% addpath("../boxplot2/");
% addpath("../boxplot2/boxplot2/");

simu1 = load("noise1.mat");
simu2 = load("noise2.mat");
simu3 = load("noise3.mat");
simu1.t_errs= simu1.t_errs(1:50,:);
simu1.t_errs1= simu1.t_errs1(1:50,:);
simu1.r_errs= simu1.r_errs(1:50,:);
simu1.r_errs1= simu1.r_errs1(1:50,:);

simu2.t_errs= simu2.t_errs(1:50,:);
simu2.t_errs1= simu2.t_errs1(1:50,:);
simu2.r_errs= simu2.r_errs(1:50,:);
simu2.r_errs1= simu2.r_errs1(1:50,:);

simu3.t_errs= simu3.t_errs(1:50,:);
simu3.t_errs1= simu3.t_errs1(1:50,:);
simu3.r_errs= simu3.r_errs(1:50,:);
simu3.r_errs1= simu3.r_errs1(1:50,:);
figure;
position_O = 4:4:28;
% position_O = position_O + 8;
h1=boxplot(simu3.t_errs,'colors',[54,96,136]./255,'positions',position_O,...
        'width',1,'symbol', '','Whisker',1);
hold on;
position_O = position_O+1;
h2=boxplot(simu2.t_errs,'colors',[144,3,95]./255,'positions',position_O,...
    'width',1,'symbol', '','Whisker',1);
position_O = position_O+1;
h3=boxplot(simu1.t_errs,'colors',[198,102,2]./255,'positions',position_O,...
        'width',1,'symbol', '','Whisker',1);
hold on;
set(h1,'LineWidth',2);
set(h2,'LineWidth',2);
set(h3,'LineWidth',2);
a=plot(0:0,'Color',[54,96,136]./255);
b=plot(0:0,'Color',[144,3,95]./255);
c=plot(0:0,'Color',[198,102,2]./255);
hold on;
set(gca,'fontname','Times','fontsize',16);
legend('k=3','k=2','k=1','FontSize',16);
xticks([5:4:29]);
xticklabels({4,5,6,7,8,9,10});
xlabel("Number of Poses",'FontSize',20);
ylabel("Error(meter)",'FontSize',20);
title("Translation",'FontSize',26);

figure;
position_O = 4:4:28;
h1=boxplot(simu3.r_errs,'colors',[54,96,136]./255,'positions',position_O,...
        'width',1,'symbol',' ');

hold on;
position_O = position_O+1;
h2=boxplot(simu2.r_errs,'colors',[144,3,95]./255,'positions',position_O,...
    'width',1,'symbol',' ');
position_O = position_O+1;
h3=boxplot(simu1.r_errs,'colors',[198,102,2]./255,'positions',position_O,...
        'width',1,'symbol',' ');
hold on;
set(h1,'LineWidth',2);
set(h2,'LineWidth',2);
set(h3,'LineWidth',2);
a=plot(0:0,'Color',[54,96,136]./255);
b=plot(0:0,'Color',[144,3,95]./255);
c=plot(0:0,'Color',[198,102,2]./255);
hold on;
set(gca,'fontname','Times','fontsize',16);
xticks([5:4:29]);
xticklabels({4,5,6,7,8,9,10});
legend('k=3','k=2','k=1','FontSize',16);
xlabel("Number of Poses",'FontSize',20);
ylabel("Error(degree)",'FontSize',20);
title("Rotation",'FontSize',26);
