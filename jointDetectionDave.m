function [ output_args ] = jointDetectionDave( log )

    % Identify the base module based on the one that moved least
    % The index of the smallest value in meanVels corresponds to the base
    % module, becuase the IMU is before anything that moves.

    % cc5
    meanVels = mean( [ mean(abs(log.gyroX));
                       mean(abs(log.gyroY));
                       mean(abs(log.gyroZ)) ] );

    wgx = mean(abs(log.gyroX))
    wgy = mean(abs(log.gyroY))
    wgz = mean(abs(log.gyroZ))
    wvel = mean(abs(log.velocity))

    for proximal = 1:log.numModules
        for distal = 1:log.numModules

            proximalModule = proximal; 
            distalModule = distal;
            proximalVels = [ log.gyroX(:,proximalModule) ...
                         log.gyroY(:,proximalModule) ...
                         log.gyroZ(:,proximalModule) + log.velocity(:,proximalModule)];

            distalVels = [ log.gyroX(:,distalModule) ...
                         log.gyroY(:,distalModule) ...
                         log.gyroZ(:,distalModule) ];

            % Rotate baseVels by the joint angle
            for i=1:length(log.time)
                proximalVels(i,:) = ...
                    [R_z(-log.position(i,proximalModule)) * proximalVels(i,:)']';
            end

            % Dot-product all the velocities
            A = distalVels' * proximalVels;

            % Take the SVD
            [U,S,V] = svd(A);

            % Get the determinant of U*V
            S_SO3 = diag( [1, 1, sign(det(U*V))] );

            % Get the rotation
            R = V * S_SO3 * U';


            %% RESIDUAL ERROR
            errors(proximal,distal) = ...
                mean(mean((proximalVels - [R*distalVels']').^2));

            R_tests(:,:,proximal,distal) = R;

    %         %% PLOTTING
    %         figure(123);
    %         plot(log.time,baseVels);
    %         hold on;
    %         plot(log.time,[R*nextVels']','--');
    %         hold off;
    %         
    %         legend x y z x y z
    %         title(['Velocities: ' num2str(proximal) '-->' num2str(distal) ]);
    %         xlabel('time (sec)');
    %         ylabel('angular velocity (rad/sec)');
    %         drawnow;
    %         pause;

        end
    end

    %% FINAL RESULT

    errorThresh = .5;
    connectionMatrix = errors < errorThresh

    figure
    imagesc(errors);
    title('Velocity Matching Error');
    xlabel('distal module');
    ylabel('proximal module');
    colorbar;

end

