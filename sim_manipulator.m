function [q_rcd, qdot_rcd, tau_rcd, s_rcd, q_est_rcd, qdot_est_rcd, d_est_rcd, s_comp_rcd, q_comp_est_rcd, qdot_comp_est_rcd, d_comp_est_rcd] = sim_manipulator(Ts, Tn, d, q_des_rcd)

    % General parameters for simulation 
    N = Tn/Ts;
    n = 3;
    
    % Control parameters of the robot manipulator
    Kd = 48;
    Kp = 1500;
    
    % Parameters of the ISMO
    Q = 10*eye(n);
    P1 = 10*eye(n);
    P2_base = 20*eye(n);
    sigma_v = 40;
    sigma_x = 20;
    sigma_s = 3.5;
    sigma_s_bar = 3;
    
    delta_x = 0.2;
    delta_s = 0.8;
    delta_v = 0.8;
    
    tau_filter = 0.5;
    
    % Generate disturbance and desired trajectory
    qdot_des_rcd =  diff(q_des_rcd, 1, 2);
    qdot_des_rcd = [qdot_des_rcd(:,1), qdot_des_rcd]/Ts;
    qddot_des_rcd = diff(qdot_des_rcd, 1, 2);
    qddot_des_rcd = [qddot_des_rcd(:,1), qddot_des_rcd]/Ts;
    
    % Create buffers to record data
    q_rcd = zeros(n, N+1);
    qdot_rcd = zeros(n, N+1);
    s_rcd = zeros(n, N+1);
    sdot_rcd = zeros(n, N+1);
    tau_rcd = zeros(n, N+1);
    v1_rcd = zeros(n, N+1);
    v2_rcd = zeros(n, N+1);
    d_est_rcd = zeros(n, N+1);
    d_comp_est_rcd = zeros(n, N+1);
    
    q_est_rcd = zeros(n, N+1);
    qdot_est_rcd = zeros(n, N+1);
    
    s_comp_rcd = zeros(n, N+1);
    v_comp_rcd = zeros(n, N+1);
    q_comp_est_rcd = zeros(n, N+1);
    qdot_comp_est_rcd = zeros(n, N+1);
    
    % Initialization of the data buffer
    q_rcd(:,1) = q_des_rcd(:,1);
    qdot_rcd(:,1) = qdot_des_rcd(:,1);
    
    q_est_rcd(:,1) = q_rcd(:,1);
    q_comp_est_rcd(:,1) = q_rcd(:,1);
    
    % Initialization of the sliding variable
    s_base = [0; 0; 0];
    
    
    % The main loop of simulation
    for i = 1:N
        
        % Read data i from the buffer
        q_des = q_des_rcd(:,i);
        qdot_des = qdot_des_rcd(:,i);
        qddot_des = qddot_des_rcd(:,i);
        q = q_rcd(:,i);
        qdot = qdot_rcd(:,i);
        
        q_est = q_est_rcd(:,i);
        qdot_est = qdot_est_rcd(:,i);
        q_comp_est = q_comp_est_rcd(:,i);
        qdot_comp_est = qdot_comp_est_rcd(:,i);
        
        d_est = d_est_rcd(:,i);
        d_comp_est = d_comp_est_rcd(:,i);
        
        % Execute the robot PID controller
        Aqc = qddot_des - Kd*(qdot-qdot_des) - Kp*(q-q_des);
        tau = mass(q)*Aqc + centrifugal(q,qdot) + friction(qdot);
        
    
        % The continuous integral sliding mode observer (ISMO)
        
        sdot = sgn(q - q_est,sigma_x,delta_x) + P1*(q - q_est) + (q - q_est)/Ts;
        s_base = s_base + sgn(q - q_est,sigma_x,delta_x) + P1*(q - q_est);
        s = s_base + q - q_est - (q_rcd(:,1) - q_est_rcd(:,1));
        s= Q*s;
        
        v1 = sgn(q - q_est,sigma_x,delta_x) + sigma_s_bar*sgn(s,sigma_s+norm(qdot_est,2),delta_s);
        v2 = sgn(v1,sigma_v,delta_v);
        
        qddot_est = mass(q)\(tau - centrifugal(q,qdot_est) - friction(qdot_est));
        q_est = q_est + Ts*(qdot_est + P1*(q-q_est)+v1);
        P2 = P2_base + 0.5*norm(qdot_est,2);
        qdot_est = qdot_est + Ts*(qddot_est + P2*v1 + v2);
        d_est = d_est*tau_filter/Ts + (mass(q)*(P2*v1+v2) - d_est);
    
        % The conventional sliding mode observer (SMO)
        
        s_comp = qdot - qdot_comp_est + P1*(q - q_comp_est);
        v_comp = 1*sigma_s_bar*sgn(s_comp,sigma_s+norm(qdot_est,2),delta_s);
        qddot_comp_est = mass(q)\(tau - centrifugal(q,qdot) - friction(qdot));
        q_comp_est = q_comp_est + Ts*(qdot_comp_est);
        qdot_comp_est = qdot_comp_est + Ts*(qddot_comp_est+v_comp);
        d_comp_est = d_comp_est*tau_filter/Ts + (mass(q)*v_comp - d_comp_est);
        
        % Robot dynamics
        qddot = mass(q)\(tau + d(:,i) - centrifugal(q,qdot) - friction(qdot));
        q = q + Ts*(qdot);
        qdot = qdot + Ts*qddot;
        
        % Store the observation to the buffer
        q_rcd(:,i+1) = q;
        qdot_rcd(:,i+1) = qdot;
        q_est_rcd(:,i+1) = q_est;
        qdot_est_rcd(:,i+1) = qdot_est;
        s_rcd(:,i) = s;
        sdot_rcd(:,i) = sdot;
        tau_rcd(:,i) = tau;
        v1_rcd(:,i) = v1;
        v2_rcd(:,i) = v2;
        s_comp_rcd(:,i) = s_comp;
        
        q_comp_est_rcd(:,i+1) = q_comp_est;
        qdot_comp_est_rcd(:,i+1) = qdot_comp_est;
        v_comp_rcd(:,i) = v_comp;
        
        d_est_rcd(:,i) = d_est;
        d_comp_est_rcd(:,i) = d_comp_est;
    
    end
    
    tau_rcd(:,i+1) = tau_rcd(:,i);

end




