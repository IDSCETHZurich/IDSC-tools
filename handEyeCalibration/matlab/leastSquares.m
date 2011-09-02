% LS solution
clear; clc;
numberOfData = 3;

% import data into matlab
dataset20cpp

% generateRandomData
%generateData(numberOfData);

M = zeros(3,3);
C = [];
bA = [];
bB = [];

numberOfSamples = 0;

for i = 1:numberOfData
    for j=1:numberOfData
        if (i ~= j)
        numberOfSamples = numberOfSamples + 1;

        %j = i+1;
        rbi1 = [eval(['rotRB',num2str(i)])  eval(['transRB',num2str(i)]); 0 0 0 1];
        rbi2 = [eval(['rotRB',num2str(j)])  eval(['transRB',num2str(j)]); 0 0 0 1];
        Ai = inv(rbi2)*rbi1;

        cbi1 = [eval(['rotCB',num2str(i)])  eval(['transCB',num2str(i)]); 0 0 0 1];
        cbi2 = [eval(['rotCB',num2str(j)])  eval(['transCB',num2str(j)]); 0 0 0 1];
        Bi = cbi2*inv(cbi1);
        
        alpha = getLogTheta(Ai(1:3,1:3));
        beta = getLogTheta(Bi(1:3, 1:3));
        M = M + beta*alpha';
        
        C = [C; eye(3) - Ai(1:3,1:3)];
        bA = [bA; Ai(1:3,4)];
        bB = [bB; Bi(1:3,4)];
        end;
    end
end
display(M);
[V,D] = eig(M'*M); 
display(V);
display(D);
Lambda = diag([sqrt(1/D(1,1)),sqrt(1/D(2,2)), sqrt(1/D(3,3))]);

x_est = V * Lambda * inv(V) * M';

display(x_est);

d = bA - kron(eye(numberOfSamples), x_est)*bB; 

bX = (C'*C)\C'*d; 
display(bX);