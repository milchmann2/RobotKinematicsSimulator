function jointDetectionSingle( log )

    numModules = size(log.position,2);
    
    connectionMatrix = zeros(numModules, numModules);
    
    gyroSum = abs(log.gyroX) + abs(log.gyroY) + abs(log.gyroZ);
    gyroMean = mean(gyroSum);
    
    [moduleOrdering, idx] = sort(gyroMean);
 
    for i=2:numModules
        modulePosition = idx(i-1);
        moduleNumber = idx(i);
        connectionMatrix(modulePosition, moduleNumber) = 1;
    end
    
    figure
    imagesc(connectionMatrix);
    title('Connection Matrix');
    xlabel('Joint Number');
    ylabel('Parent Joint');
end

