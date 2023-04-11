close all
%clear
clc

% Set up the vehicle model (for state estimation purpose)
unicycle = unicycleKinematics("VehicleInputs","VehicleSpeedHeadingRate","WheelSpeedRange",[0 Inf]);

% Parameters
dT = 0.1;           % sampling time
num_states = 3;     % number of states 
num_inputs = 2;     % number of inputs

% Prediction horizon 
N = 10;

% % Weight matrix 
% Q = diag([2.0 2.0 0.01]);          % weight of the states 
% R = diag([40 2.0]);                % weight of the control variables
% S = diag([0.001 0.001 5]);         % termination state cost 

% Weight matrix 
Q = diag([1.0 1.0 1.0]);           % weight of the states 
R = diag([1.0 1.0]);             % weight of the control variables
S = diag([1.0 1.0 1.0]);        % termination state cost 

% Constraint vectors
umin = [-4;-4];
umax = [4;4];

% velocity at time step k, theta at time step k 
v_k = 0.0;
theta_k = 0.0;

% Store the parameters into a structure
sys.dT = dT;
sys.num_states = num_states;
sys.num_inputs = num_inputs;
sys.umin = umin;
sys.umax = umax;

% Create the trajectory
tsim = 30.0;              % Simulation time 
t = 0:dT:tsim;         
total_sim_iter = length(t);   % number of iteration of the main simulation loop

% [x_ref,y_ref,theta_ref,~,~,~,~]=generate_cubic(dT,tsim);
[x_ref,y_ref,theta_ref,v_ref,~,a_ref,w_ref] = generate_route(dT,tsim);


% Create a empty column vector of size of the trajectory
% Store the reference state trajectory in this single column vector 
refTrajectory = zeros(length(x_ref) * 3, 1);    % 303x1

index = 1;
for i = 1:3:length(refTrajectory)
    refTrajectory(i) = x_ref(index);
    refTrajectory(i + 1) = y_ref(index);
    refTrajectory(i + 2) = theta_ref(index);
    index = index + 1;
end

% Initial state of the robot 
% Set initial state to be the start of the trajectory
%x0 = [x_ref(1);y_ref(1);theta_ref(1)];
x0 = [0;-2;theta_ref(1)];
theta_k = x0(3);         % position the robot such that that the orientation matches with the ref trajectory 

% Create A,B,C matrix for the discrete-time motion model
[A,B,C] = prediction_model(v_k,theta_k,dT);

% Store the matrices into the structure
sys.A = A;
sys.B = B;
sys.C = C;

% used to introudce integral action in MPC 
A_tilde = [sys.A sys.B;                               % 5x5 
         zeros(num_inputs,num_states) eye(num_inputs)
          ];

B_tilde = [sys.B;                                     % 5x2
         eye(num_inputs)
          ];

C_tilde = [sys.C zeros(size(C,1),num_inputs)];        % 3x5

% Store the matrices into the structure
sys.A_tilde = A_tilde;
sys.B_tilde = B_tilde;
sys.C_tilde = C_tilde;


% Initialize MPC parammeters
mpc.N = N;
mpc.Q = Q;
mpc.R = R;
mpc.S = S; 

% Initialize parameters for the main simulation loop
xActual(:,1) = x0;          % insert the initial state of the robot to the column vector 
u_k(:,1) = [0;0];           % control variable at time step k 
du(:,1) = [0;0];            % chnage in control variable (delta u_k)

i_ref = 1;                  % index used to read reference trajectory

% calculate the euclidean distance between the two points
p1 = [x0(1) x0(2)];
p2 = [x_ref(1) y_ref(1)];
D = norm(p2 - p1);
c = 1.5*D;

% run simulation 
% [fitness,xActual,~,~,~,~,~,~] = evaluate_fitness(xActual,u_k,du,total_sim_iter,v_k,theta_k,dT,sys,i_ref,D,c,refTrajectory,unicycle,x_ref,y_ref,theta_ref,v_ref,w_ref,mpc); 

%% Main simulation loop for Genetic Algorithm  (used to tune the weight matrices automatically) 
tic
% Initialize parameters
animate = 1;                     % activate animation at the end of evolution
rng('default');                  % set the random number generator
rng shuffle;                     % creates a different seed each time 
termination_counter = 1;         % current number of generation 
generation_limit = 50;           % iteration limit of the genetic algorithm
num_children = 2;                % number of children produced every iteration (no more than 2)
population_size = 20;            % population size
tournament_size = 3;             % 0.2 * population size 
pool_size = 2;                   % pool size: number of parent yo want  
crossover_rate = 0.9;            % crossover rate 
mutation_rate = 0.2;             % mutation rate
pm = 0.8;                        % probability of mutation for random resetting
b = 3;                           % parameter for non-uniform mutation
upperBound = [100,100,10,10];    % upper bound of the weights
lowerBound = [0,0,0,1];          % lower bound of the weights
child_prev = struct('N',[zeros(1)],'Q',[zeros(3)],'R',[zeros(2)],'S',[zeros(3)]); % children produced from the previous generation


% Initialize population           
[population,queue] = initialize_population(population_size);
fitness = [];

%% Evaluate population
for i = 1:population_size
   
    [xActual,u_k,du,v_k,theta_k,D] = reset_states(x0,x_ref(1),y_ref(1));

    individual = population(i);
    [fitness(end+1),xActual,~,~,~,~,theta_k,cte] = evaluate_fitness(xActual,u_k,du,total_sim_iter,v_k,theta_k,dT,sys,i_ref,D,c,refTrajectory,unicycle,x_ref,y_ref,theta_ref,v_ref,w_ref,individual); 
    disp(['Evaluating initial population ', num2str(toc), 's'])

    % disp(theta_k);
end 

%% Evolution begins 

[xActual,u_k,du,v_k,theta_k,D] = reset_states(x0,x_ref(1),y_ref(1));

while(IsTerminationSatisfied(termination_counter,generation_limit) ~= 1)

    for i = 1:num_children
    % parent selection: select two parents to reproduce 
    parents_index = parent_selection_tournament(fitness,population_size,tournament_size,pool_size);

    % implement crossover with probability
    r = rand;
    
    if r < crossover_rate
        
        % wright's heuristic crossover
        child = wright_heuristic_crossover(population(parents_index(1)),fitness(parents_index(1)),population(parents_index(2)),fitness(parents_index(2)));
    else
        if child_prev.N ~= 0
            child = child_prev(1);
        else
            rc = randi([1 2],1,1);
            child = population(parents_index(rc));
        end       
    end

    % implement mutation with porability
    r1 = rand;

    if r1 < mutation_rate

        % non-uniform mutation 
        child = nonuniform_mutation(child,termination_counter,generation_limit,b,pm,upperBound,lowerBound);
        
    end

    % check if the child is legitimate
    if child_prev.N ~= 0
        child = impose_limits(child,child_prev);
    else
        rc = randi([1 2],1,1);
        child = impose_limits(child,population(parents_index(rc)));
    end

    % evaluate the fitness of the child
    [child_fitness_value,xActual,D,u_k,du,v_k,theta_k] = evaluate_fitness(xActual,u_k,du,total_sim_iter,v_k,theta_k,dT,sys,i_ref,D,c,refTrajectory,unicycle,x_ref,y_ref,theta_ref,v_ref,w_ref,child);
    children_fitness(i) = child_fitness_value;

    % update the child
    children(i) = child;

    % update the previous child;
    child_prev = children(end);

    % reset state
    [xActual,u_k,du,v_k,theta_k,D] = reset_states(x0,x_ref(1),y_ref(1));

    disp(['Auto-tuning in progress ', num2str(toc), 's'])

    end 

    % deletion: make place for this new child
    [population,fitness,queue] = delete_oldest(population,fitness,queue,children,children_fitness);

    % update the counter 
    % disp('Tuning in progress')
    termination_counter = termination_counter + 1;
    

end

%% Evolution Ends

% Find which individual in the population has the highest fitness
[best_fitness,best_index] = max(fitness);

% Evaulate the best individual
[~,xActual,D,u_k,du,v_k,theta_k,cte] = evaluate_fitness(xActual,u_k,du,total_sim_iter,v_k,theta_k,dT,sys,i_ref,D,c,refTrajectory,unicycle,x_ref,y_ref,theta_ref,v_ref,w_ref,population(best_index));

%% Animate results
if animate == 1
    animate_MPC(x_ref,y_ref,xActual);
end








%% Simulation loop function
function [xActual,u_k,du,D,convergence_flag] = run_simulation(xActual,u_k,du,total_sim_iter,v_k,theta_k,dT,sys,mpc,i_ref,D,c,N,refTrajectory,unicycle,x_ref,y_ref,convergence_flag)
    % loop 
    for i = 1:total_sim_iter-10

    % Update the matrices A,B that governs the motions of the robot 
    % Purpose of this step: prepare that matrices that are used to 
    % predict the states of the robot 
    % from current time step k over a specified horizon
    % Note: v_k, theta_k, dT needs to be updated at the end of the loop
    [A,B,C] = prediction_model(v_k,theta_k,dT);

    % Create augumented matrices，used to introudce integral action
    sys.A = A;
    sys.B = B;
    sys.C = C;

    sys.A_tilde = [sys.A sys.B;                               % 5x5 
               zeros(sys.num_inputs,sys.num_states) eye(sys.num_inputs)
              ];

    sys.B_tilde = [sys.B;                                     % 5x2
               eye(sys.num_inputs)
              ];

    sys.C_tilde = [sys.C zeros(size(sys.C,1),sys.num_inputs)];    % 3x5

    % Update the vector [x_k+1; u_k] 
    % x_k+1 : Robot state at sample instant one sample interval after current
    % sample.
    % u_k: the control inputs at current sample instant 
    x_kplus1_tilde = [xActual(:,end);u_k(:,end)];

    % Decide how far you want to predict
    % Extract part of the trajectory states based on specified horizon
    % if N=8, you need to extract 24 data 
        % if the index exceeds the trajecotry size, extract til the end
        % else extract 3*N number of data, but remember 
        % to substract 1 because i_ref already includes the current element
        if i_ref + (3 * N - 1) >= length(refTrajectory)
            extracted_refTrajectory = refTrajectory(i_ref:end);
            % decreasing the horizon
            N = N - 1;
        else 
            extracted_refTrajectory = refTrajectory(i_ref:i_ref + 3*N - 1);
        end
        i_ref = i_ref + sys.num_states; 

    % run MPC
    % find the first delta u in the optimal sequence of delta u (change in control inputs)
    % store the delta u in 
    du(:,end+1) = run_MPC(sys,mpc,x_kplus1_tilde,extracted_refTrajectory);

    % uk = uk-1 + delta uk
    u_k(:,end+1) = u_k(:,end) + du(:,end);
    
    % Send the control command to the actual robot ( simulated robot)
    t0 = dT*i;
    tf = t0 + dT;
    T = t0:dT:tf;

    currentState = xActual(:,end);  % alternative: xActual(:,end) 
    u = u_k(:,end);                        % Similiarly, u_k(:,end) 

    [T,q] = ode45(@(t,q) derivative(unicycle,q,u),T,currentState);

    % update state
    finalState = q(end,:);
    xActual(:, end+1) = finalState;       % alternative: xActual(:,end + 1) = q(end,:)
    theta_k = finalState(3);                        % v_k gets updated?
    v_k = u_k(1,end);
    if v_k < 0
        v_k = 0;
    end 
        
    % disp(v_k)

    % Terminate when robot is off the course
    p1 = finalState(1:2);
    p2 = [x_ref(i+1) y_ref(i+1)];
    D(end+1) = norm(p2-p1);
    D_avg = sum(D) / length(D);
        if(D_avg > D(1) + c)
            convergence_flag = 0;
            break
        else
            convergence_flag = 1;
        end 

    end 
end 



%% Function to fill matrice A_bar and B_bar(Tested) 
function [A_bar,B_bar] = fill_AB(A_bar,B_bar,A_tilde,B_tilde,N)

    % need two loops, outer loop to fill the rows
    % inner loop to fill the columns
    
    % the matrix B_bar has size 5*N by 2*N 
    % each sub matrix has size of 5 by 2 

    for sample_i = 1:N
        % fill the A_bar
        A_bar((sample_i-1)*size(A_tilde,1)+1:sample_i * size(A_tilde,1),1:size(A_tilde,2)) = A_tilde^(sample_i);
        for col_i = 1:N
            if col_i <= sample_i
                % fill the B_bar
                B_bar((sample_i-1)*size(B_tilde,1)+1:sample_i * size(B_tilde,1),(col_i-1)*size(B_tilde,2)+1:col_i * size(B_tilde,2)) = A_tilde^(sample_i - col_i)*B_tilde;
            end 
        end
    end     
end 

%% Function to fill matrice CQC_bar,QC_bar, R_bar (Tested)
function [CQC_bar,QC_bar,R_bar] = fill_weightMatrices(CQC_bar,QC_bar,R_bar,CQC,QC,CSC,SC,R,N)
    for sample_i = 1:N
        if sample_i == N
            CQC_bar((sample_i-1)*size(CQC,1)+1:sample_i * size(CQC,1),(sample_i-1)*size(CQC,2)+1:sample_i * size(CQC,2)) = CSC;
            QC_bar((sample_i-1)*size(QC,1)+1:sample_i * size(QC,1),(sample_i-1)*size(QC,2)+1:sample_i * size(QC,2)) = SC;
        else
            % exponentially increasing weight matrix Q 
            % QC = 2^(sample_i - 1) * QC;
            % CQC = 2^(sample_i - 1) * CQC;
            CQC_bar((sample_i-1)*size(CQC,1)+1:sample_i * size(CQC,1),(sample_i-1)*size(CQC,2)+1:sample_i * size(CQC,2)) = CQC;
            QC_bar((sample_i-1)*size(QC,1)+1:sample_i * size(QC,1),(sample_i-1)*size(QC,2)+1:sample_i * size(QC,2)) = QC;
        end

        R_bar((sample_i-1)*size(R,1)+1:sample_i * size(R,1),(sample_i-1)*size(R,2)+1:sample_i * size(R,2)) = R;      
    end    
end 


%% MPC
function du_optimal = run_MPC(sys,mpc,x_kplus1_tilde,extracted_refTrajectory)

    % load the basic matrices
    N = mpc.N;
    Q = mpc.Q;
    R = mpc.R;
    S = mpc.S;

    Atil = sys.A_tilde;
    Btil = sys.B_tilde;
    Ctil = sys.C_tilde;

    % create empty stacked diagonal weighting matrices
    CQC = Ctil' * Q * Ctil;
    CSC = Ctil' * S * Ctil;
    QC = Q * Ctil;
    SC = S * Ctil;

    CQC_bar = zeros(size(CQC,1) * N,size(CQC,2) * N);       % 5*N by 5*N
    QC_bar = zeros(size(QC,1)* N,size(QC,2) * N);           % 3*N by 5*N 
    R_bar = zeros(size(R,1) * N, size(R,2) * N);            % 2*N by 2*N 

    % create empty stacked coefficient matrices A,B for the prediction model
    A_bar = zeros(size(Atil,1) * N, size(Atil,2) * 1);            % 5*N by 5 
    B_bar = zeros(size(Btil,1) * N, size(Btil,2) * N);            % 5*N by 2*N

    % fill in A_bar, B_bar for the prediction model
    [A_bar,B_bar] = fill_AB(A_bar,B_bar,Atil,Btil,N);

    % fill in CQC_bar, QC_bar and R_bar for the MPC cost function 
    [CQC_bar,QC_bar,R_bar] = fill_weightMatrices(CQC_bar,QC_bar,R_bar,CQC,QC,CSC,SC,R,N);

    % compute the H and f term of the quadratic objective function
    % Note: f should be a column vector 
    H = 2.* B_bar'* CQC_bar * B_bar + R_bar;
    fT = [x_kplus1_tilde' extracted_refTrajectory'] * [A_bar' * CQC_bar * B_bar; -QC_bar * B_bar];
    % fT = [x_kplus1_tilde' extracted_refTrajectory'] * [A_bar' ; -QC_bar * B_bar];

    % Set up the min and max for the quadratic objective function
    du_min = ones(sys.num_inputs, N);
    du_max = ones(sys.num_inputs, N);
    for i = 1:N
        du_min(:, i) = sys.umin;
        du_max(:, i) = sys.umax;
    end

    % Prepare the coefficient matrices/vecotrs for the solver
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = du_min;
    ub = du_max;
    x0 = [];
    %options;
    H = (H+H')/2;

    % call the solver
    % matlab function: x = quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options)
    % decision variable is change in control inputs u 
    du_star = quadprog(H,fT,A,b,Aeq,beq,lb,ub,x0,optimset('Display', 'off'));          % du_optimal is a column vector
    
    du_optimal = du_star(1:2,1);   
end 


%% Discrete Kinematic Model of the Unicycle
function [A,B,C] = prediction_model(vk,thetak,dT)
    A = [1 0 -vk*sin(thetak)*dT;
         0 1 vk*cos(thetak)*dT;
         0 0 1
        ];
    B = [cos(thetak)*dT 0;
         sin(thetak)*dT 0;
         0 dT
        ];
    C = [1 0 0;
         0 1 0;
         0 0 1
        ];

end 

%% Plot the reference trajectory and animate robot's motion
function animate_MPC(x_ref,y_ref,xActual)
close all
figure 
% Plot the initial position of the robot
plot(xActual(1,1),xActual(2,1),"rpentagram",'LineWidth',1,'MarkerSize',10);

% set axis limits
set(gca,'XLim',[-6 12],'YLim',[-6 14]);

% plot the reference trajectory
% plot(x_ref(1:end),y_ref(1:end),"k-",'LineWidth',2);
hold on;
grid on;
% axis equal;

% parameters c
curve_robot = animatedline('Color','r','LineWidth',2);
curve_ref = animatedline('Color','k','LineWidth',2);

x = xActual(1,:);
y = xActual(2,:);
num_states = size(xActual,2);
num_iter = length(0:0.1:30);

% Plot the initial position of the robot
% plot(xActual(1,1),xActual(2,1),"rpentagram",'LineWidth',1,'MarkerSize',10);

for i = 1:num_iter
    %plot(xActual(1,i),xActual(2,i),'o','MarkerSize',2,'LineWidth',2);
    %pause(0.2)
    if(i <= num_states)
        addpoints(curve_robot,x(i),y(i));
    end 
    addpoints(curve_ref,x_ref(i),y_ref(i));
    drawnow;
    % pause(0.02);
    % movie(i) = getframe;
end
% generate video
% video = VideoWriter('curve4_GA_3','MPEG-4');
% video.FrameRate = 30;
% 
% open(video);
% writeVideo(video,movie);
% close(video);    
end 
