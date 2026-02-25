%Define constants and initial conditions for Simple Helo Simulation. 
% This is called by all other scripts, and should be run first.
constants=DefineConstants;

%Initial Trim Solution - this sets forward speed, no climb, side velocity
%or turn rate
VXTRIM_vals_kts=0:20:120; % Airspeeds to trim at, in kts
VXTRIM_vals_kts(1)=0.01; % Cannot trim at exactly 0 airspeed, so set first value to 0.01 kts
VXTRIM_vals=VXTRIM_vals_kts*1.68781; % Convert to ft/sec

VYTRIM_vals=zeros(size(VXTRIM_vals)); % No side velocity in trim
VZTRIM_vals=zeros(size(VXTRIM_vals)); % No climb in trim
PSIDTRIM_vals=zeros(size(VXTRIM_vals)); % No turn rate in trim

plot_AttitudeAndControlsAndEigenvalues(constants,VXTRIM_vals,VYTRIM_vals,VZTRIM_vals,PSIDTRIM_vals);

