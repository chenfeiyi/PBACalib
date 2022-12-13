T_scene1 = load("real-scene1.mat");
T_scene2 = load("real-scene2.mat");
T_scene3 = load("real-scene3.mat");
T_scene4 = load("real-scene4.mat");
ours_eul1=[];
ours_t1=[];
ours_eul2=[];
ours_t2=[];
ours_eul3=[];
ours_t3=[];
ours_eul4=[];
ours_t4=[];
for idx =1:size(T_scene2.Ts,2)
    T1 = T_scene1.Ts{idx};
    T2 = T_scene2.Ts{idx};
    T3 = T_scene3.Ts{idx};
    T4 = T_scene4.Ts{idx};
    eul = rotm2eul(T1(1:3,1:3),'XYZ')*180/pi-[88,0,88];
    ours_eul1 = [ours_eul1;eul];
    ours_t1 = [ours_t1;T1(1:3,4)'];
    eul = rotm2eul(T2(1:3,1:3),'XYZ')*180/pi-[88,-0.1,88.1];
    ours_eul2 = [ours_eul2;eul];
    ours_t2 = [ours_t2;T2(1:3,4)'];
    eul = rotm2eul(T3(1:3,1:3),'XYZ')*180/pi-[87.7,0,88];
    ours_eul3 = [ours_eul3;eul];
    ours_t3 = [ours_t3;T3(1:3,4)'+[0,0.03,0]];
    eul = rotm2eul(T4(1:3,1:3),'XYZ')*180/pi-[87.8,0.1,88.1];
    ours_eul4 = [ours_eul4;eul];
    ours_t4 = [ours_t3;T4(1:3,4)'+[0,0.03,0]];
end
% xticks=["roll","pitch","yaw"];
figure;
position_O = [1:4.5:10];
g1 = boxplot(ours_eul1,'colors',[54,96,136]./255,'positions',position_O,...
        'width',1,'symbol', '');
hold on;
position_O = position_O+1;
g2=boxplot(ours_eul2,'colors',[144,3,95]./255,'positions',position_O,...
    'width',1,'symbol', '');
hold on;
position_O = position_O+1;
g3=boxplot(ours_eul3,'colors',[198,102,2]./255,'positions',position_O,...
    'width',1,'symbol', '');
hold on;
position_O = position_O+1;
g4=boxplot(ours_eul4,'colors',[46,184,114]./255,'positions',position_O,...
    'width',1,'symbol', '');
hold on;
a=plot(0:0,'Color',[54,96,136]./255);
b=plot(0:0,'Color',[144,3,95]./255);
c=plot(0:0,'Color',[198,102,2]./255);
d=plot(0:0,'Color',[46,184,114]./255);
set(g1,'LineWidth',2);
set(g2,'LineWidth',2);
set(g3,'LineWidth',2);
set(g4,'LineWidth',2);
set(gca,'fontname','Times','fontsize',16);
xticks([2.5,7,11.5]);
xticklabels({"roll","pitch","yaw"});
legend('scene 1','scene 2','scene 3','scene 4');
ylabel("Degree",'FontSize',20);
title("Rotation",'FontSize',26);
figure;
position_O = 1:5:11;
g1=boxplot(ours_t1,'colors',[54,96,136]./255,'positions',position_O,...
        'width',1,'symbol', '','Whisker',1);
hold on;
position_O = position_O+1;
g2=boxplot(ours_t2,'colors',[144,3,95]./255,'positions',position_O,...
    'width',1,'symbol', '','Whisker',1);
hold on;
position_O = position_O+1;
g3=boxplot(ours_t3,'colors',[198,102,2]./255,'positions',position_O,...
    'width',1,'symbol', '','Whisker',1);
hold on;
xticks([2.5,7,11.5]);
xticklabels({"x","y","z"});
position_O = position_O+1;
g4=boxplot(ours_t4,'colors',[46,184,114]./255,'positions',position_O,...
    'width',1,'symbol', '','Labels',xticks,'Whisker',1);
a=plot(0:0,'Color',[54,96,136]./255);
b=plot(0:0,'Color',[144,3,95]./255);
c=plot(0:0,'Color',[198,102,2]./255);
d=plot(0:0,'Color',[46,184,114]./255);
hold on;
set(g1,'LineWidth',2);
set(g2,'LineWidth',2);
set(g3,'LineWidth',2);
set(g4,'LineWidth',2);
set(gca,'XTickLabel',{'X','Y','Z'},'fontname','Times','fontsize',16);
legend('scene 1','scene 2','scene 3','scene 4');
ylabel("Meters",'FontSize',20);
title("Translation",'FontSize',26);
