
load path.mat
dest = path;

route = zeros(length(dest)-1,2);
%each x and y displacement between each points.
for i = 2:length(dest)
    route(i-1,1) = dest(i,1) - dest(i-1,1);
    route(i-1,2) = dest(i,2) - dest(i-1,2);
end
totalangle = zeros(length(route),1);
dist = zeros(length(route),1);
tangentvalue = zeros(length(route),1);
for i = 1:length(route)
    totalangle(i) = atan(route(i,1)/route(i,2));
  
%     angle(i) = deg2rad(angle(i));
    dist(i) = sqrt((route(i,1)^2 + route(i,2)^2));
end
angle(1) = totalangle(2)-totalangle(1);
for i=2:length(totalangle)
    angle(i) = totalangle(i)-totalangle(i-1);
end

%stable angular velocity 0.8
omega = 0.8;
d = 0.254;
VLturn = -omega*d/2;
VRturn = omega*d/2;

turntime = zeros(1,length(route));
for i = 1:length(route)
    turntime(i) = -angle(i)/omega;
end

%stable going forward speed of 0.1

forwardtime = zeros(1,length(route));
for i = 1:length(route)
    forwardtime(i) = dist(i)/0.1;
end
VLgo=0.1;
VRgo=0.1;

pubvel = rospublisher('/raw_vel') ;
message = rosmessage(pubvel); 
tic
%%
while 1
    while toc<turntime(1)
            pause(0.005);
            message.Data = [VLturn,VRturn];
            send(pubvel,message);
    end
    while toc<turntime(1)+forwardtime(1) 
            pause(0.005);
            message.Data = [VLgo,VRgo];
            send(pubvel,message);
    end
    for i = 2:length(route)
        VLturn = -omega*d/2;
        VRturn = omega*d/2;
        if turntime(i)<0
            VLturn = -VLturn;
            VRturn = -VRturn;
            turntime(i) = -turntime(i);
        end
        while toc<sum(turntime(1:i))+sum(forwardtime(1:i-1))
            pause(0.005);
            message.Data = [VLturn,VRturn];
            send(pubvel,message);
            disp(turntime(i))
            disp('turn')
        end
        
        while toc<sum(turntime(1:i))+sum(forwardtime(1:i)) 
            pause(0.005);
            message.Data = [VLgo,VRgo];
            send(pubvel,message);
            disp(forwardtime(i))
            disp('go')
        end
    end

   message.Data = [0,0];
   send(pubvel,message);
   break;
    
    
 
%     prompt = 'Press Enter to Stop Robot';
%     str = input(prompt,'s');
%     if isempty(str)
%         message.Data=[0,0];
%         send(pubvel,message)
%     end
    
    %in Matlab tic and toc start and stop a timer. In this program we are making sure we 
    %drive the desired distance by finding the necessary time based on speed

end