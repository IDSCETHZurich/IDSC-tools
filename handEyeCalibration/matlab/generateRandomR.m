function result = generateRandomR()
    %Generate Random rotational AXIS
    randAxis = rand(3,1);
    randAxis = randAxis/norm(randAxis);
    
    %Generate Random rotational Angle
    randAngle = rand();
    
    q = [cos(randAngle/2), sin(randAngle/2)*randAxis'];  
    result = quat2dcm(q);
end