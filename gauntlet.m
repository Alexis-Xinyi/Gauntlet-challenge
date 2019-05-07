load scandata3.mat


polar1 = [r_theta(:,1) r_theta(:,5)];
polar2 = [r_theta(:,2) r_theta(:,6)];
polar3 = [r_theta(:,3) r_theta(:,7)];
polar4 = [r_theta(:,4) r_theta(:,8)];

cleanpolar1 = polar1(all(polar1,2),:);
cleanpolar2 = polar2(all(polar2,2),:);
cleanpolar3 = polar3(all(polar3,2),:);
cleanpolar4 = polar4(all(polar4,2),:);
%polarplot(deg2rad(polar1(:,2)),polar1(:,1),'ks')
%%
% figure(1)
% polarplot(deg2rad(cleanpolar1(:,2)),cleanpolar1(:,1),'ks')
% figure(2)
% polarplot(deg2rad(cleanpolar2(:,2)),cleanpolar2(:,1),'ks')
% figure(3)
% polarplot(deg2rad(cleanpolar3(:,2)),cleanpolar3(:,1),'ks')
% figure(4)
% polarplot(deg2rad(cleanpolar4(:,2)),cleanpolar4(:,1),'ks')

%%
pos1 = zeros(length(cleanpolar1),2);
pos2 = zeros(length(cleanpolar2),2);
pos3 = zeros(length(cleanpolar3),2);
pos4 = zeros(length(cleanpolar4),2);


for i=1:length(pos1)
    pos1(i,1) = cleanpolar1(i,1)*cos(deg2rad(cleanpolar1(i,2)));
    pos1(i,2) = cleanpolar1(i,1)*sin(deg2rad(cleanpolar1(i,2)));
end

for i=1:length(pos2)
    pos2(i,1) = cleanpolar2(i,1)*cos(deg2rad(cleanpolar2(i,2)));
    pos2(i,2) = cleanpolar2(i,1)*sin(deg2rad(cleanpolar2(i,2)));
end

for i=1:length(pos3)
    pos3(i,1) = cleanpolar3(i,1)*cos(deg2rad(cleanpolar3(i,2)));
    pos3(i,2) = cleanpolar3(i,1)*sin(deg2rad(cleanpolar3(i,2)));
end

for i=1:length(pos4)
    pos4(i,1) = -cleanpolar4(i,1)*cos(deg2rad(cleanpolar4(i,2)));
    pos4(i,2) = -cleanpolar4(i,1)*sin(deg2rad(cleanpolar4(i,2)));
end
% 
% plot(pos1(:,1),pos1(:,2),'ks')
% figure(5)
% plot(pos2(:,1),pos2(:,2),'ks')
% figure(6)
% plot(pos3(:,1),pos3(:,2),'ks')

%Fitting all scans into the same coordinate

opos1 = [pos1(:,1), pos1(:,2)];
opos2 = [pos2(:,1)-0.5, pos2(:,2)+0.36];
opos3 = [pos3(:,1)-1.38, pos3(:,2)+1];
opos4 = [pos4(:,1)-1.9, pos4(:,2)+0.13];
pos = [opos1; opos3; opos4];

% figure()
% plot(opos1(:,1),opos1(:,2),'ks')
% hold on
% plot(opos2(:,1),opos2(:,2),'ks')
% plot(opos3(:,1),opos3(:,2),'ks')
% plot(opos4(:,1), opos4(:,2),'ks')
%
one = zeros(length(pos),1)+1;
posro = [pos, one];
th = -0.05;
trans = [1 0 -0.25; 0 1 0.3; 0 0 1];
rotate = [cos(th) sin(th) 0 ; -sin(th) cos(th) 0; 0 0 1];
posfinal = transpose(rotate*trans*transpose(posro));
pos = [posfinal(:,1)+0.25,posfinal(:,2)-0.3];

figure(100)
plot(pos(:,1),pos(:,2),'ks')
hold on 
%plot(path(:,1),path(:,2))
% title('MAP')
%%
%RANSAC for walls
%plots have y<0 (bottom wallb)

finalm = [0];
finalb = [0];

wallb = [0,0];
for i = 1:length(pos)
    if pos(i,2)<-0.27 && pos(i,2)>-0.35
        wallb = [wallb; pos(i,:)];
    end
end
wallb(1,:) = [];



%Fitting wallb
maxvel = 0;

for p = 1:length(wallb)
    point1 = wallb(p,:);
    for k = p+1:length(wallb)
        total = 0;
        point2 = wallb(k,:);
        points = [point1; point2];
        x0 = [points(:,1),[1;1]];
        B = linsolve(x0,points(:,2));
        for j = 1:length(wallb)
  
            inlier = robustLineFit(wallb(j,1),wallb(j,2),B(1),B(2));
            total = total+inlier;
        end
         
            if total > maxvel
                maxvel = total;
                finalm(1) = [B(1)];
                finalb(1) = [B(2)];
            
            end
       
    end
end

maxvel
% figure()
ywall = wallb(:,1)*finalm(1)+finalb(1);
plot(wallb(:,1),ywall)


%% Left wall

walll = [0,0];
for i = 1:length(pos)
    if pos(i,1)<-2.2 && pos(i,1) >-2.3
        walll = [walll; pos(i,:)];
    end
end
walll(1,:) = [];

%Fitting wallb
maxvel = 0;

for p = 1:length(walll)
    point1 = walll(p,:);
    for k = p+1:length(walll)
        total = 0;
        point2 = walll(k,:);
        points = [point1; point2];
        x0 = [points(:,1),[1;1]];
        B = linsolve(x0,points(:,2));
        for j = 1:length(walll)
           
            inlier = robustLineFit(walll(j,1),walll(j,2),B(1),B(2));
            total = total+inlier;
        end
         
            if total > maxvel
                maxvel = total;
                finalm(2) = B(1);
                finalb(2) = B(2);
            
            end
       
    end
end
% figure()
ywalll = walll(:,1)*finalm(2)+finalb(2);
plot(walll(:,1),ywalll)
% hold on
% plot(walll(:,1),walll(:,2),'ks')
% title('leftwall')

%% Top wall

wallt = [0,0];
for i = 1:length(pos)
    if pos(i,2)>1.2% && pos(i,1)> -2.15 &&pos(i,1)<0.181
        wallt = [wallt; pos(i,:)];
    end
end
wallt(1,:) = [];

%Fitting wallb
maxvel = 0;

for p = 1:length(wallt)
    point1 = wallt(p,:);
    for k = p+1:length(wallt)
        total = 0;
        point2 = wallt(k,:);
        points = [point1; point2];
        x0 = [points(:,1),[1;1]];
        B = linsolve(x0,points(:,2));
        for j = 1:length(wallt)
           
            inlier = robustLineFit(wallt(j,1),wallt(j,2),B(1),B(2));
            total = total+inlier;
        end
         
            if total > maxvel
                maxvel = total;
                finalm(3) = B(1);
                finalb(3) = B(2);
            
            end
       
    end
end
% figure()
ywallt = wallt(:,1)*finalm(3)+finalb(3);
plot(wallt(:,1),ywallt)
% hold on
% plot(wallt(:,1),wallt(:,2),'ks')
% title('topwall')

%% Right Wall
wallr = [0,0];
for i = 1:length(pos)
    if pos(i,1) >0.2 
        wallr = [wallr; pos(i,:)];
    end
end
wallr(1,:) = [];

%Fitting wallb
maxvel = 0;

for p = 1:length(wallr)
    point1 = wallr(p,:);
    for k = p+1:length(wallr)
        total = 0;
        point2 = wallr(k,:);
        points = [point1; point2];
        x0 = [points(:,1),[1;1]];
        B = linsolve(x0,points(:,2));
        for j = 1:length(wallr)
           
            inlier = robustLineFit(wallr(j,1),wallr(j,2),B(1),B(2));
            total = total+inlier;
        end
         
            if total > maxvel
                maxvel = total;
                finalm(4) = B(1);
                finalb(4) = B(2);
            
            end
       
    end
end
% figure()
ywallr = wallr(:,1)*finalm(4)+finalb(4);
plot(wallr(:,1),ywallr)
% hold on
% plot(pos(:,1),pos(:,2),'ks')
% %plot(wallr(:,1),wallr(:,2),'ks')
% title('rightwall')

%% Barrier 1

bar1 = [0,0];
for i = 1:length(pos)
    if pos(i,2) >0.7357 && pos(i,2)< 0.9631 && pos(i,1)>-1 && pos(i,1)< -0.509
        bar1 = [bar1; pos(i,:)];
    end
end
bar1(1,:) = [];

%Fitting wallb
maxvel = 0;

for p = 1:length(bar1)
    point1 = bar1(p,:);
    for k = p+1:length(bar1)
        total = 0;
        point2 = bar1(k,:);
        points = [point1; point2];
        x0 = [points(:,1),[1;1]];
        B = linsolve(x0,points(:,2));
        for j = 1:length(bar1)
           
            inlier = robustLineFit(bar1(j,1),bar1(j,2),B(1),B(2));
            total = total+inlier;
        end
         
            if total > maxvel
                maxvel = total;
                finalm(5) = B(1);
                finalb(5) = B(2);
            
            end
    end
end
% figure()
ybar1 = bar1(:,1)*finalm(5)+finalb(5);
plot(bar1(:,1),ybar1)
% hold on
% 
% 
% plot(pos(:,1),pos(:,2),'ks')
% title('bar1')


