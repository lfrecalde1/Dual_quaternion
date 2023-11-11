function logDQ = log_dual_quaternion(DQ)
    % Extract the real and dual parts
    q0 = DQ(1:4);
    qe = DQ(5:8);

    % Calculate the quaternion logarithm of q0
    log_q0 = quaternion_log(q0);

    % Calculate the logarithm of the dual quaternion
    logDQ = [log_q0; 0.5 * quaternion_multiply(log_q0, qe)];
end

