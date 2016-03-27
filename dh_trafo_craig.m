function T = dh_trafo_craig(alpha,a,d,theta)
    % alpha,a,d,theta: dh-parameter according to craig

    % Calculates Transformational Matrix from DH-Parameters
    % according to CRAIG frame assignments
    %
    %   ALL ANGLES IN DEG!!
    error(nargchk(4,4,nargin));

    T = [cosd(theta), -sind(theta), 0, a;
        sind(theta)*cosd(alpha), cosd(theta)*cosd(alpha), -sind(alpha), -sind(alpha)*d;
        sind(theta)*sind(alpha),cosd(theta)*sind(alpha), cosd(alpha),cosd(alpha)*d;
        0, 0, 0, 1];
end 