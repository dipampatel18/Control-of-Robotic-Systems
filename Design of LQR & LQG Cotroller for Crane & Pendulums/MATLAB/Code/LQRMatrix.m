%% Value of Parameters

m1 = 100; m2 = 100; M = 1000;
l1 = 20; l2 = 10;
g = 9.81;

%% Defining the Matrix A (6 x 6)

A = [0 1 0 0 0 0;
     0 0 (-(m1*g)/M) 0 (-(m2*g)/M) 0;
     0 0 0 1 0 0;
     0 0 (-(m1+M)*g/(l1*M)) 0 (-(m2*g)/(l1*M)) 0;
     0 0 0 0 0 1;
     0 0 (-(m1*g)/(l2*M)) 0 (-(m2+M)*g/(l2*M)) 0];

%% Defining the Matrix B (6 x 1)

B = [0;
     1/M;
     0;
     (1/(M*l1));
     0;
     (1/(M*l2))];

%% Defining the Matrix C (6 x 6)

C = [B (A*B) (A^2*B) (A^3*B) (A^4*B) (A^5*B)];
RL = rank(C);

%% Defining D

D = det(C);