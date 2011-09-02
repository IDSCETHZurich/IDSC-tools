function generateData(numberOfData)
%generateData.m  
Rx =[ 0 -1 0; 1 0 0; 0 0 1];  

rx = [0.17;0.02;0.03];

tool_camera_T = [Rx rx; 0 0 0 1];
base_CB_T = [eye(3) ones(3,1); 0 0 0 1];

for i=1:numberOfData
    base_tool_T = [generateRandomR(), rand(3,1); 0 0 0 1];
    camera_CB_T = inv(base_tool_T*tool_camera_T)*base_CB_T;
    
    assignin('base',['rotRB',num2str(i)], base_tool_T(1:3, 1:3));
    assignin('base',['transRB',num2str(i)], base_tool_T(1:3, 4));
    assignin('base',['rotCB',num2str(i)], camera_CB_T(1:3, 1:3));
    assignin('base',['transCB',num2str(i)], camera_CB_T(1:3, 4));
end

end
