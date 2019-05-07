
%edit your ranges to display here.  important to not include the actual
%location of your object in this grid of points or it will give you
%infinities

[px,py]=meshgrid(-2.5:.05:.5,-.5:.05:1.5);
[xlim,ylim] = size(px);
V = zeros(xlim, ylim);

for i=1:xlim
    for j=1:ylim
%this is the equation and integral with ranges for a specific object:  you
%should be able to figure out what this is and edit appropriately to get
%what you want
dV1 = @(x)  1./sqrt((px(i,j)-x).^2 + (py(i,j)-(0.0117*x-0.264)).^2);
dV2 = @(x)  1./sqrt((px(i,j)-x).^2 + (py(i,j)-(15.38*x+35.1)).^2);
dV3 = @(x)  1./sqrt((px(i,j)-x).^2 + (py(i,j)-( -0.0094*x+1.325)).^2);
dV4 = @(x)  1./sqrt((px(i,j)-x).^2 + (py(i,j)-(-11.166*x+2.77)).^2);

dV5 = @(x,y)  1./sqrt((px(i,j)-x).^2 + (py(i,j)-(y)).^2);
dV6 = @(x,y)  1./sqrt((px(i,j)-x).^2 + (py(i,j)-y).^2);
dV7 = @(x)  1./sqrt((px(i,j)-x).^2 + (py(i,j)-(.04758*x+1.205)).^2);
dV8 = @(x)  1./sqrt((px(i,j)-x).^2 + (py(i,j)-(.04758*x+1.35)).^2);
dV9 = @(x)  -1./sqrt((px(i,j)-x).^2 + (py(i,j)+29.0891*x+7.4636).^2);

V1(i,j) = integral(dV1,-2.3247,0.2583);
V2(i,j) = integral(dV2,-2.3,-1.191)*10;
V3(i,j) = integral(dV3,-2.27,.2);
V4(i,j) = integral(dV4,.12,.3)*10;

V5(i,j) = integral2(dV5,-1.76,-1.402,0.1454,0.3351)*10;
V6(i,j) = -integral2(dV6,-1.976,-1.791,0.7649,0.9974)*60;
V7(i,j) = integral(dV7,-1.003,-0.509);
V8(i,j) = integral(dV8,-1.113,-0.62)*0.8;
V9(i,j) = integral(dV9,-.3144,-.02439)*10;
V = (V1 + V2 + V3 + V4 +V5 + V6+V7+V8)*10 ;
%V = V3;
    end
end

figure()
surf(px,py,V)
figure()
hold off

contour(px,py,V)
%%
[Ex,Ey] = gradient(V);
hold on
quiver(px,py,-Ex,-Ey)



gnorm = inf;
r = [0;0];
x = 0;
y = 0;
lambda = 0.05;
delta = 1.02;
tol = 0.001;
path = [x,y];

while gnorm>=tol



    pos = [round((x+2.5)/0.05,0),round((y+0.5)/0.05,0)]

        x = x - lambda * Ex(pos(2), pos(1));

        y = y - lambda * Ey(pos(2),pos(1));

    lambda = lambda * delta
    path = [path; x y];
    gnorm = sqrt(Ex(pos(2),pos(1))^2+Ey(pos(2),pos(1))^2);
 
end
%%
k=1
for i = 1:length(path)
    if path(k,1)<-2.3 || path(k,1)>0.5 ||path(k,2)>1.5 ||path(k,2)<-0.5
        path(k,:) = [];
    else k = k+1;
    end
end
hold on
plot(path(:,1),path(:,2),'ks');


hold off
%%
save('path.mat','path')
