function [result]=getLogTheta(R)
theta =  acos((trace(R)-1)/2);
logTheta = 0.5*theta/sin(theta)*(R-R');
result = [logTheta(3,2);logTheta(1,3);logTheta(2,1)];
end