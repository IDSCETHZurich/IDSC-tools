function [] = handEyeCalibration()
%% Generate Solution
[Xr,~] = qr(randn(3));
Xt = randn(3,1); Xt = Xt / norm(Xt);

%% Generate Data
%Generate random rotational matrices Ar, Br
[Ar1,~] = qr(randn(3));
[Ar2,~] = qr(randn(3));

Br1 = inv(Xr)*Ar1*Xr;
Br2 = inv(Xr)*Ar2*Xr;

%Generate random rotational matrices At, Bt
At1 = randn(3,1); At1 = At1 / norm(At1);
At2 = randn(3,1); At2 = At2 / norm(At2);

Bt1 = inv(Xr)*(Ar1*Xt -Xt + At1);
Bt2 = inv(Xr)*(Ar2*Xt -Xt + At2);

% [Ar1 At1; 0 0 0 1]*[Xr Xt; 0 0 0 1] - [Xr Xt; 0 0 0 1]*[Br1 Bt1; 0 0 0 1]
% [Ar2 At2; 0 0 0 1]*[Xr Xt; 0 0 0 1] - [Xr Xt; 0 0 0 1]*[Br2 Bt2; 0 0 0 1]

Alpha1 = getLogTheta(Ar1);
Alpha2 = getLogTheta(Ar2);

Beta1 = getLogTheta(Br1);
Beta2 = getLogTheta(Br2);

Astylish = [Alpha1 Alpha2 crossProduct(Alpha1, Alpha2)];
Bstylish = [Beta1 Beta2 crossProduct(Beta1, Beta2)];

Xr_est = Astylish*inv(Bstylish);

display(Xr - Xr_est);

end

function [result]=crossProduct(V1,V2)
%CROSS_PRODUCT calculates the cross product of two vectors.
i=(V1(2)*V2(3) - V2(2)*V1(3));
j=(V1(3)*V2(1) - V2(3)*V1(1));
k=(V1(1)*V2(2) - V2(1)*V1(2));
result=[i;j;k];
end

function [result]=getLogTheta(R)
q = dcm2quat(R);
theta =  acos((trace(R)-1)/2);
logTheta = 0.5*theta/sin(theta)*(R-R');
result = [logTheta(3,2);logTheta(1,3);logTheta(2,1)];
end