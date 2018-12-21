%% Value of Parameters

m1 = 100; m2 = 100; M = 1000;
l1 = 20; l2 = 10;
g = 9.81;

%% Substituting the Values of Different Parameters in Matrices

A = [0 1 0 0 0 0;
     0 0 (-(m1*g)/M) 0 (-(m2*g)/M) 0;
     0 0 0 1 0 0;
     0 0 (-(M+m1)*g/(M*l1)) 0 (-(m2*g)/(M*l1)) 0;
     0 0 0 0 0 1;
     0 0 (-(m1*g)/(M*l2)) 0 (-(M+m2)*g/(M*l2)) 0];

B = [0;
     1/M;
     0;
     1/(M*l1);
     0;
     1/(M*l2)];

% Uncomment one of the Cases and Execute the Code for Different Conditions 

% Case-1 
C = [1 0 0 0 0 0];      
Outputs = {'x'};

% Case-2
% C = [0 0 1 0 0 0
%      0 0 0 0 1 0];
% Outputs = {'theta1', 'theta2'};

% Case-3
% C = [1 0 0 0 0 0
%      0 0 0 0 1 0];
% Outputs = {'x', 'theta2'};

% Case-4
% C = [1 0 0 0 0 0
%      0 0 1 0 0 0
%      0 0 0 0 1 0];
% Outputs = {'x', 'theta1', 'theta2'};

D = 0;

P = [-0.2 -0.3 -0.4 -0.5 -0.6 -0.7];


X0 = [0;
      0;
      ((10*pi)/180);
      0;
      ((15*pi)/180);
      0];

Xhat = [0;
        0;
        ((10*pi)/180);
        0;
        ((15*pi)/180);
        0];

L = place(A', C', P)';

States = {'X', 'Xdot', 'phi1', 'phi1_dot', 'phi_2', 'phi2_dot'};
Inputs = {'F'};

SystSS = ss(A, B, C, D, 'statename', States, 'inputname', Inputs, 'outputname', Outputs);
O = obsv(SystSS);
RO = rank(O);

%% Checking the Controllability

if (RO == min(size(O)))
    disp('System is Observable');
else
    disp('System is Not Observable');
end

% Simulating the Time Response of Dynamic System to Arbitrary Inputs

t = 0:0.01:100;
u = ones(size(t));

[Y,~,X] = lsim(SystSS,u,t,X0);

X_est = Xhat';
k = 2;

for n = 0.01:0.01:100
    dXhat = A * Xhat + B .* u(k) + L * (Y(k,:)' - C*Xhat);
    Xhat = Xhat + 0.01.*dXhat;
    X_est = [X_est;Xhat'];
    k = k + 1;
end

%% Plotting the Output

figure,
plot(t, X(:,1));
hold on;

plot(t, X_est(:,1),'g');
xlabel('Time(sec)'), ylabel('Crane Position(m)'), legend('X', 'X_est');
hold off;

figure,
plot(t, X(:,3));
hold on;

plot(t, X_est(:,3), 'g');
xlabel('Time(sec)'), ylabel('Pendulum Angle(theta1)'),legend('theta1', 'theta1Est');
hold off;

figure,
plot(t, X(:,5));
hold on;

plot(t, X_est(:,5), 'g');
xlabel('Time(sec)'), ylabel('Pendulum Angle(theta2)'), legend('theta2', 'theta2Est');
hold off;
