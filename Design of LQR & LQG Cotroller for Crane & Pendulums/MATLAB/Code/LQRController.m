%% Value of Parameters

m1 = 100; m2 = 100; M = 1000;
l1 = 20; l2 = 10;
g = 9.81;

%% Substituting the Values of Different Parameters in Matrices

A = [0 1 0 0 0 0;
     0 0 -(m1*g)/M 0 -(m2*g)/M 0;
     0 0 0 1 0 0;
     0 0 -(M+m1)*g/(M*l1) 0 -(m2*g)/(M*l1) 0;
     0 0 0 0 0 1;
     0 0 -(m1*g)/(M*l2) 0 -(M+m2)*g/(M*l2) 0];

B = [0;
     1/M;
     0;
     1/(M*l1);
     0;
     1/(M*l2)];

C = [1 0 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 0 1 0];
 
D = 0;

%% Choosing the Values of Q & R

Q = (C')*(C);

% Assigning the Values in Q using Trial & Error Method

Q(1,1) = 70000000;
Q(3,3) = 8000000000;
Q(5,5) = 9000000000;             

R = 1;                  % Ideal Value of R 

%% Designing the LQR 

% Calculating the Optimal Gain Matrix K

K = lqr(A, B, Q, R)

% Calculating the Closed Loop Matrix Value of A using K

ANew = (A - (B*K));     % Values of Other Matrices remain the same

%% Checking Controllability

RC = rank(C);

% Checking the Controllability

if (RC == min(size(C)))
    disp('System is Controllable');
else
    disp('System is Uncontrollable');
end

%% State Space Representation

States = {'x' 'xDot' 'theta1' 'theta1Dot' 'theta2' 'theta2Dot'};
Inputs = {'r'};
Outputs = {'x'; 'theta1'; 'theta2'};

% Creating the State Space Model

ContSS = ss(ANew, B, C, D, 'statename', States, 'inputname', Inputs, 'outputname', Outputs);

% Displaying the Eigen Values

EigenValues = eig(ANew)