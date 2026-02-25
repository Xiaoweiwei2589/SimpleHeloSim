
function plot_AttitudeAndControlsAndEigenvalues(constants,VXTRIM_vals,VYTRIM_vals,VZTRIM_vals,PSIDTRIM_vals)

% initialize arrays to store trim solutions
x0_vals=zeros(length(VXTRIM_vals),constants.NSTATES);
u0_vals=zeros(length(VXTRIM_vals),constants.NCTRLS);
for i=1:length(VXTRIM_vals)
    [x0,u0]=SingleTrim(constants,VXTRIM_vals(i),VYTRIM_vals(i),VZTRIM_vals(i),PSIDTRIM_vals(i));
    x0_vals(i,:)=x0;
    u0_vals(i,:)=u0;
end

VXTRIM_vals_kts=VXTRIM_vals/1.68781; % Convert back to kts for plotting

% Plot the roll and pitch attitude (in degrees) versus forward speed (in kts) 
figure;
%Convert x0_vals(:,7) and x0_vals(:,8) from radians to degrees
plot(VXTRIM_vals_kts,x0_vals(:,7)*180/pi,'-o'); hold on; % Plot roll (phi)
plot(VXTRIM_vals_kts,x0_vals(:,8)*180/pi,'-s'); % Plot pitch (theta)
xlabel('Forward Speed (kts)');
ylabel('Attitude (Degrees)');
title("Roll and Pitch Attitude versus Airspeed")
legend('Phi', 'Theta');

% Plot the control inputs versus forward speed (in kts)
figure;
plot(VXTRIM_vals_kts,u0_vals(:,1),'-o'); hold on;  % Plot lateral cyclic
plot(VXTRIM_vals_kts,u0_vals(:,2),'-s');% Plot longitudinal cyclic
plot(VXTRIM_vals_kts,u0_vals(:,3),'-d'); % Plot MR collective
plot(VXTRIM_vals_kts,u0_vals(:,4),'-^'); % Plot TR collective
xlabel('Forward Speed (kts)');
ylabel('Control Input (Degrees)');
title("Controls versus Airspeed")
legend('Lateral Cyclic', 'Longitudinal Cyclic', 'MR Collective', 'TR Collective');

% Plot eigenvalues of A versus forward speed
n = size(x0_vals,1);
reals = [];
imags = [];
speeds = [];

for i = 1:n
    x0 = x0_vals(i,:)';
    u0 = u0_vals(i,:)';
    xdot0 = zeros(constants.NSTATES,1);
    % Linearize the system at the trim condition to get A matrix, then compute eigenvalues
    [F,G,M,A,B,C,D] = linearize('SimpleHelo', x0, u0, xdot0, constants);
    e = eig(A);
    reals = [reals; real(e)];
    imags = [imags; imag(e)];
    speeds = [speeds; ones(length(e),1)*VXTRIM_vals_kts(i)];
end

figure;
scatter(reals, imags, 60, speeds, 'filled');
colormap(jet);
h = colorbar;
h.Label.String = 'Forward speed (kts)';
xlabel('Real part of eigenvalue');
ylabel('Imaginary part of eigenvalue');
title('Eigenvalues of A vs Forward Speed');
grid on;
hold on;
plot(xlim, [0 0], 'k--');
plot([0 0], ylim, 'k--');
hold off;

return