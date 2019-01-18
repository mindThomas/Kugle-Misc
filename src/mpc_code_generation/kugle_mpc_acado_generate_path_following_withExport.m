clear all;

    acadoSet('problemname', 'kugle_mpc_export'); 

    %% Define variables (both internal and external/inputs)
    DifferentialState q2; % OBS. The order of construction defines the order in the chi vector
    DifferentialState q3;
    DifferentialState x;
    DifferentialState y;
    DifferentialState dx;
    DifferentialState dy;   
    DifferentialState s;
    DifferentialState ds;
    
    Control omega_ref_x;
    Control omega_ref_y;   
    Control s_acceleration;
    
    ts = 1/20;
    N = 50;  
    
    %OnlineData represents data that can be passed to the solver online (real-time)
    OnlineData desiredVelocity;
    OnlineData maxAngle;
    OnlineData maxOmegaRef;    
    OnlineData minVelocity;
    OnlineData maxVelocity;    
    OnlineData trajectoryLength;
    OnlineData trajectoryStart;
    
    % Reference polynomial coefficients (for up to 7th order polynomial)
    OnlineData cx7;
    OnlineData cx6;
    OnlineData cx5;
    OnlineData cx4;
    OnlineData cx3;
    OnlineData cx2;
    OnlineData cx1;
    OnlineData cx0;
    OnlineData cy7;
    OnlineData cy6;
    OnlineData cy5;
    OnlineData cy4;
    OnlineData cy3;
    OnlineData cy2;
    OnlineData cy1;
    OnlineData cy0;
    
    % Evaluate polynomial based on s variable
    % Intermediate states helps to speed up the Automatic Differentiation of ACADO Symbolic
    s_ = acado.IntermediateState(s + trajectoryStart);
    x_ref = acado.IntermediateState(cx7*s_^7 + cx6*s_^6 + cx5*s_^5 + cx4*s_^4 + cx3*s_^3 + cx2*s_^2 + cx1*s_ + cx0);
    y_ref = acado.IntermediateState(cy7*s_^7 + cy6*s_^6 + cy5*s_^5 + cy4*s_^4 + cy3*s_^3 + cy2*s_^2 + cy1*s_ + cy0);
    
    %TIME t; % the TIME
    %Parameter l; % parameters are time varying variables
    %Variable
    %Vector
    %Matrix
        
    %input1 = acado.MexInput; % inputs into the MEX function in the order of construction
    %input2 = acado.MexInputVector; % myexample RUN(10, [0 1 2 3], eye(3,3)),
    %input3 = acado.MexInputMatrix;


    %% Define differential equation (model of plant) - (see page ?? in ACADO MATLAB manual)
    f = acado.DifferentialEquation();  
    % possibility 1: link a Matlab ODE
    % f.linkMatlabODE('LinearMPCmodel_acado'); % however this method will slow down the generated code
    % possibility 2: write down the ODE directly in ACADO syntax    
    f.add(dot(q2) == 1/2 * omega_ref_x);
    f.add(dot(q3) == 1/2 * omega_ref_y);
    f.add(dot(x) == dx);
    f.add(dot(y) == dy);
    f.add(dot(dx) == 13.71*q3);
    f.add(dot(dy) == -13.71*q2);    
    f.add(dot(s) == ds);
    f.add(dot(ds) == s_acceleration);
    % possibility 4: write down the discretized ODE directly in ACADO syntax
    % Note that the start time, end time, step size (ts), and the number N of control intervals should be chosen in such a way that the relation
    % (t_end - t_start) / ts = N*i    (should hold for some integer, i)
    % f = acado.DiscretizedDifferentialEquation(ts); 
    % f.add(next(q2) == q2 + ts * 1/2 * omega_ref_x);           
    
    %% Define optimal control problem (see page 29 in ACADO MATLAB manual)
    %ocp = acado.OCP(0.0, tEnd, N); % note that if the time is optimized, the output time will be normalized between [0:1]
    ocp = acado.OCP(0.0, N*ts, N);
    
    %x_err = acado.IntermediateState(x - x_ref);
    %y_err = acado.IntermediateState(y - y_ref);
    x_err = x - x_ref;
    y_err = y - y_ref;  
    ds_err = ds - desiredVelocity;
    %s_to_end = trajectoryLength - s;
    
    %h = [diffStates; controls]; % 'diffStates' and 'controls' are automatically defined by ACADO
    %hN = [diffStates]; 
    h = [x_err;y_err;ds_err; omega_ref_x;omega_ref_y];
    hN = [x_err;y_err;ds_err];
    W = acado.BMatrix(eye(length(h))); % Cost-function weighting matrix
    WN = acado.BMatrix(eye(length(hN)));
    
    ocp.minimizeLSQ(W, h);
    ocp.minimizeLSQEndTerm(WN, hN);
    %ocp.minimizeLSQ({q2,q3}); % min(q2^2 + q3^2)
    %ocp.minimizeLagrangeTerm( omeg_ref_x*omeg_ref_x + omeg_ref_y*omeg_ref_y ); % Lagrange terms are on the whole sequence
    %ocp.minimizeMayerTerm( x ); % Mayer terms are only the final state, control input etc.
    
    %ocp.subjectTo( f );
    ocp.setModel( f );  
    
    %% Define final-state requirements    
    %ocp.subjectTo( 'AT_END', x_err == 0 );
    %ocp.subjectTo( 'AT_END', y_err == 0 );    

    %% Define constraints
    ocp.subjectTo( q2 - sin(1/2*maxAngle) <= 0 );  % q2 <= sin(1/2*maxAngle)
    ocp.subjectTo( -q2 - sin(1/2*maxAngle) <= 0 ); % -q2 <= sin(1/2*maxAngle)  --->  q2 >= -sin(1/2*maxAngle)
    ocp.subjectTo( q3 - sin(1/2*maxAngle) <= 0 );  % q3 <= sin(1/2*maxAngle)
    ocp.subjectTo( -q3 - sin(1/2*maxAngle) <= 0 ); % -q3 <= sin(1/2*maxAngle)  --->  q2 >= -sin(1/2*maxAngle)
    
    ocp.subjectTo( omega_ref_x - maxOmegaRef <= 0 ); % omega_ref_x <= maxOmegaRef
    ocp.subjectTo( -omega_ref_x - maxOmegaRef <= 0 ); % omega_ref_x >= -maxOmegaRef
    ocp.subjectTo( omega_ref_y - maxOmegaRef <= 0 ); % omega_ref_x <= maxOmegaRef
    ocp.subjectTo( -omega_ref_y - maxOmegaRef <= 0 ); % omega_ref_x >= -maxOmegaRef    

    ocp.subjectTo( ds - minVelocity >= 0 );
    ocp.subjectTo( ds - maxVelocity <= 0 );  % ds <= maxVelocity
    %ocp.subjectTo( -ds + minVelocity <= 0 );  % ds >= minVelocity   --->   -ds <= -minVelocity   --->   -ds + minVelocity <= 0
    
    %% Create and configure ACADO optimization algorithm
    %algo = acado.OptimizationAlgorithm(ocp);   
    %algo.set('KKT_TOLERANCE', 1e-10); % Set a custom KKT tolerance
        
    mpc = acado.OCPexport( ocp );
    mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
    mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
    mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');  % FULL_CONDENSING, FULL_CONDENSING_N2
    mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL2'       );  % INT_RK45, INT_IRK_GL2, INT_IRK_GL4
    mpc.set( 'NUM_INTEGRATOR_STEPS',        2*N                 );
    mpc.set( 'QP_SOLVER',                   'QP_QPOASES3'    	);
    mpc.set( 'LEVENBERG_MARQUARDT',         1e-4                );    
    mpc.set( 'HOTSTART_QP',                 'YES'             	);   
    %mpc.set( 'CG_HARDCODE_CONSTRAINT_VALUES','YES'             	);
    mpc.set( 'FIX_INITIAL_STATE',            'YES'             	);     % should be set to YES for MPC
    %mpc.set( 'GENERATE_MATLAB_INTERFACE', 'YES'               );     
    %mpc.set( 'GENERATE_SIMULINK_INTERFACE', 'YES'               );     
    %mpc.set( 'CG_USE_VARIABLE_WEIGHTING_MATRIX', 'YES'       );   % allow different weighting matrices for each stage in the horizon (1:(N+1))
    
%     mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
%     mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
%     mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2'); % FULL_CONDENSING, FULL_CONDENSING_N2
%     mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'       ); % INT_RK45, INT_IRK_GL4
%     mpc.set( 'NUM_INTEGRATOR_STEPS',         3*N                );
%     mpc.set( 'QP_SOLVER',                   'QP_QPOASES3'    	);
%     mpc.set( 'HOTSTART_QP',                 'YES'             	);
%     mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-10				);
%     mpc.set( 'CG_HARDCODE_CONSTRAINT_VALUES','YES'             	);
%     mpc.set( 'FIX_INITIAL_STATE',            'YES'             	);
%     % mpc.set('KKT_TOLERANCE',1e-10)
%     % mpc.set('MAX_NUM_ITERATIONS ',100)    

% mpc.set ( 'HESSIAN_APPROXIMATION' , 'GAUSS_NEWTON' ); % solving algorithm
% mpc.set ( 'DISCRETIZATION_TYPE' , 'MULTIPLE_SHOOTING' ); %  Discretization algorithm
% mpc.set ( 'INTEGRATOR_TYPE' , 'INT_RK4' ) ; % Intergation algorithm
% mpc.set ( 'NUM_INTEGRATOR_STEPS' , 250) ; % Number of integration steps
% mpc.set ( 'SPARSE_QP_SOLUTION' , 'FULL_CONDENSING_N2' );
% mpc.set ( 'FIX_INITIAL_STATE' , 'YES' );
% mpc.set ( 'HOTSTART_QP' , 'YES' );
% mpc.set ( 'GENERATE_TEST_FILE' , 'YES' );
    
    mpc.exportCode( 'kugle_mpc_export' );
    mpc.printDimensionsQP();        
    global ACADO_;
    ACADO_.helper
    copyfile([ACADO_.pwd '/../../external_packages/qpoases3'], 'kugle_mpc_export/qpoases3')
    cd('kugle_mpc_export');      
    make_acado_solver
    copyfile('acado_solver.mex*', '../')
    cd('..');
    
    MPCparameters.ts = ts;
    MPCparameters.N = N;
    save('MPCparameters.mat', 'MPCparameters');
    
    return;
    pause(1.0);
    
    cd('kugle_mpc_export');
    copyfile('../make_custom_solver_sfunction.m', 'make_custom_solver_sfunction.m');
    copyfile('../acado_solver_sfun.c', 'acado_solver_sfun.c');
    %make_acado_solver_sfunction
    make_custom_solver_sfunction
    copyfile('acado_solver_sfun.mex*', '../')
    cd('..');
    
    make_ACADO_MPC_MEX

%clear;

%% Configuration parameters for test
N = 50;
xInit = [0,0,  2,1,  0,0]';
uInit = [0,0]';
Wmat = eye(4);
WNmat = eye(2);
ref0 = [0,0, 0,0];
refInit = repmat(ref0, [N+1,1]);
maxAngle = deg2rad(10);
maxOmegaRef = deg2rad(30);
xFinal = 0;
yFinal = 0;
od0 = [maxAngle, maxOmegaRef, xFinal, yFinal];
odInit = repmat(od0, [N+1,1]);

%% Test MPC with regular MATLAB interface
clear acado_input;
acado_input.od = odInit; % Online data
acado_input.W = Wmat;
acado_input.WN = WNmat;
acado_input.x = repmat(xInit', [N+1,1]);
acado_input.x0 = xInit';
acado_input.y = refInit(1:N, :);
acado_input.yN = refInit(N+1, 1:2);
acado_input.u = repmat(uInit', [N,1]); % Set initial inputs to zero

tic;
acado_output = acado_solver(acado_input);
toc;

%% Test the MPC
ACADO_MPC_MEX(0, xInit, uInit, Wmat, WNmat, refInit, odInit)

x = xInit;
ref = refInit;
od = odInit;
nIter = 1;

tic;
[u0, xTraj, uTraj, kktTol, status, nIter, objVal] = ACADO_MPC_MEX(1, x, ref, od, nIter)
toc;

% Compare the two compiled program outputs
acado_output.x == xTraj

% Visualize
figure(2);
plot(xTraj(:,3));
hold on;
plot(xTraj(:,4));
hold off;