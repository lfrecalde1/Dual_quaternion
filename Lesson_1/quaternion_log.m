function log_q = quaternion_log(q)
    % Calculate the quaternion logarithm
    norm_q = norm(q(2:4));
    %angle = atan2(norm_q, q(1));
    %axis = q(2:4) / norm_q;
    angle = (1/norm(q(2:4)))*acos(q(1)/norm(q));
    axis = q(2:4);
    log_q = [log(norm(q)), axis * angle];
end