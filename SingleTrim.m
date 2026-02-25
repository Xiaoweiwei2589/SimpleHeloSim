function[x0,u0]=SingleTrim(constants,VXTRIM,VYTRIM,VZTRIM,PSIDTRIM)
%Trim the helicopter at a single forward speed.  
%This is used for plotting controls versus airspeed, and for linearizing at a single flight condition.
%Note that the trim procedure is not guaranteed to converge, and may fail at some speeds.

%Set up trim variables
%Initial guess for state in trim solution
x0=zeros(constants.NSTATES,1);
x0(1)=VXTRIM;
x0(2)=VYTRIM;
x0(3)=VZTRIM;

%Initial guess for controls
u0=[0;0;20.;25.]; % Reset initial guess for controls, make it the same for each trim

%Trim Target values 
targ_des=zeros(constants.NSTATES+constants.NOUT,1);  % [xdot_e, y_e]
targ_des(10)=VXTRIM;
targ_des(11)=VYTRIM;
targ_des(12)=VZTRIM;
targ_des(9)=PSIDTRIM;

[x0,u0]=trimmer('SimpleHelo',x0,u0,targ_des,constants);