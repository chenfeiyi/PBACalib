T_scene1 = load("scene1.txt");
T_scene2 = load("scene2.txt");
T_scene3 = load("scene3.txt");
T_scene4 = load("scene4.txt");
ours_eul1=[];
ours_t1=[];
ours_eul2=[];
ours_t2=[];
ours_eul3=[];
ours_t3=[];
ours_eul4=[];
ours_t4=[];
for idx =1:size(T_scene1,1)/4
    T1 = T_scene1(4*idx-3:4*idx,:);
    T2 = T_scene2(4*idx-3:4*idx,:);
    T3 = T_scene3(4*idx-3:4*idx,:);
    T4 = T_scene4(4*idx-3:4*idx,:);
    eul = rotm2eul(T1(1:3,1:3),"XYZ")*180/pi-[88,0,88];
    ours_eul1 = [ours_eul1;eul];
    ours_t1 = [ours_t1;T1(1:3,4)'];
    eul = rotm2eul(T2(1:3,1:3),"XYZ")*180/pi-[88,-0.1,88.1];
    ours_eul2 = [ours_eul2;eul];
    ours_t2 = [ours_t2;T2(1:3,4)'];
    eul = rotm2eul(T3(1:3,1:3),"XYZ")*180/pi-[87.6,0,88];
    ours_eul3 = [ours_eul3;eul];
    ours_t3 = [ours_t3;T3(1:3,4)'+[0,0.03,0]];
    eul = rotm2eul(T4(1:3,1:3),"XYZ")*180/pi-[87.8,0.1,88.1];
    ours_eul4 = [ours_eul4;eul];
    ours_t4 = [ours_t3;T4(1:3,4)'+[0,0.03,0]];
end

xticks = ["roll","pitch","yaw"];
figure;
position_O = 1:4.5:10;
boxplot(ours_eul1,'colors',[54,96,136]./255,'positions',position_O,...
        'width',1,'symbol', '','Labels',xticks,'Whisker',1);
hold on;
position_O = position_O+1;
boxplot(ours_eul2,'colors',[144,3,95]./255,'positions',position_O,...
    'width',1,'symbol', '','Labels',xticks,'Whisker',1);
hold on;
position_O = position_O+1;
boxplot(ours_eul3,'colors',[198,102,2]./255,'positions',position_O,...
    'width',1,'symbol', '','Labels',xticks,'Whisker',1);
hold on;
position_O = position_O+1;
boxplot(ours_eul4,'colors',[46,184,114]./255,'positions',position_O,...
    'width',1,'symbol', '','Labels',xticks,'Whisker',1);
hold on;
a=plot(0:0,'Color',[54,96,136]./255);
b=plot(0:0,'Color',[144,3,95]./255);
c=plot(0:0,'Color',[198,102,2]./255);
d=plot(0:0,'Color',[46,184,114]./255);
set(gca,'XTickLabel',{'Roll','Pitch','Yaw'},'fontname','Times','fontsize',12);
legend('scene1','scene2','scene3','scene4','FontSize',16);
ylabel("Degree",'FontSize',16);
title("Rotation",'FontSize',20);

xticks = ["X","Y","Z"];
figure;
position_O = 1:4.5:10;
boxplot(ours_t1,'colors',[54,96,136]./255,'positions',position_O,...
        'width',1,'symbol', '','Labels',xticks,'Whisker',1);
hold on;
position_O = position_O+1;
boxplot(ours_t2,'colors',[144,3,95]./255,'positions',position_O,...
    'width',1,'symbol', '','Labels',xticks,'Whisker',1);
hold on;
position_O = position_O+1;
boxplot(ours_t3,'colors',[198,102,2]./255,'positions',position_O,...
    'width',1,'symbol', '','Labels',xticks,'Whisker',1);
hold on;

position_O = position_O+1;
boxplot(ours_t4,'colors',[46,184,114]./255,'positions',position_O,...
    'width',1,'symbol', '','Labels',xticks,'Whisker',1);
a=plot(0:0,'Color',[54,96,136]./255);
b=plot(0:0,'Color',[144,3,95]./255);
c=plot(0:0,'Color',[198,102,2]./255);
d=plot(0:0,'Color',[46,184,114]./255);
hold on;
set(gca,'XTickLabel',{'X','Y','Z'},'fontname','Times','fontsize',12);
legend('scene 1','scene 2','scene 3','scene 4','FontSize',16);
ylabel("Meters",'FontSize',16);
title("Translation",'FontSize',20);
