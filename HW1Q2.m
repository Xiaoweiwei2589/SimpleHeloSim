clear all;clc;
% Initialize the model in hover. Compute the eigenvectors of the A matrix
SimpleHeloInit;

% Calculate eigenvectors and eigenvalues of A matrix
[eigenvectors, eigenvalues] = eig(A);

% Display the eigenvalues
disp('Eigenvalues of A matrix:');
disp(diag(eigenvalues));



% Display the eigenvectors
%disp('Eigenvectors of A matrix:');
%disp(eigenvectors);

% Display pairs of eigenvalues and their corresponding eigenvectors
%fprintf('\n--- Eigenvalue and Eigenvector Pairs ---\n');
%for i = 1:size(A,1)
%    fprintf('\nEigenvalue %d: %.4f + %.4fi\n', i, real(eigenvalues(i,i)), imag(eigenvalues(i,i)));
%    fprintf('Eigenvector %d:\n', i);
%    disp(eigenvectors(:,i));
%end

% Assuming your dataset variable is called 'out' or 'logsout'
% Replace 'out' with your actual variable name
%{
% Method 1: Access by signal name
Time=Attitudes{1}.Values.Time;
Phi=Attitudes{1}.Values.Data(:,1);
Theta=Attitudes{1}.Values.Data(:,2);
Psi=Attitudes{1}.Values.Data(:,3);
u=BodyVel{1}.Values.Data(:,1);
v=BodyVel{1}.Values.Data(:,2);
w=BodyVel{1}.Values.Data(:,3);

AoA=atan2(w,u)*180/pi; % Angle of Attack in degrees
Beta=atan2(u,w)*180/pi; % Sideslip Angle in degrees

% Plot Phi and Theta over time
figure;
plot(Time, AoA, 'r', Time, Theta, 'b');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('AoA', 'Theta');
title('AoA and Theta over Time');
grid on;
%}