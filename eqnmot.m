function[xdot]=eqnmot(x,FM,constants);

%Equations of Motion Module

%Inputs: Fuselage State Vector, x
%        Total Aerodynamic Forces and Moments, FM
%Outputs: Fuselage State Vector Derivative, xdot
%This module simulates the standard aircraft, rigid body equations of motion

% The important thing in practice, I believe,  is not to code EOM, in terms of this ,AI can
% do a better job than me. Able to derive the correct EOM. 


u=x(1);
v=x(2);
w=x(3);
p=x(4);
q=x(5);
r=x(6);
phi=x(7);
theta=x(8);
psi=x(9);

X=FM(1);
Y=FM(2);
Z=FM(3);
L=FM(4);
M=FM(5);
N=FM(6);

cphi=cos(phi);
sphi=sin(phi);
cthe=cos(theta);
sthe=sin(theta);
cpsi=cos(psi);
spsi=sin(psi);

xdot=zeros(12,1);


%The following equations are given in Padfield pages 92 and 173-178
% Calculate state derivatives
%Velocities
%udot eqn.
xdot(1)=X/constants.mass-constants.g*sthe-q*w+r*v;
%vdot eqn.
xdot(2)=Y/constants.mass+constants.g*cthe*sphi-r*u+p*w;
%wdot eqn.
xdot(3)=Z/constants.mass+constants.g*cthe*cphi-p*v+q*u;

%Angular rates
gam=constants.Ix*constants.Iz-constants.Ixz^2;
%pdot eqn.
xdot(4)=(constants.Iz*L+constants.Ixz*N+constants.Ixz*(constants.Ix-constants.Iy+constants.Iz)*p*q-(constants.Iz^2-constants.Iy*constants.Iz+constants.Ixz^2)*q*r)/gam;
%qdot eqn.
xdot(5)=(M+(constants.Iz-constants.Ix)*p*r-constants.Ixz*(p^2-r^2))/constants.Iy;
%rdot eqn.
xdot(6)=(constants.Ix*N+constants.Ixz*L-constants.Ixz*(constants.Ix-constants.Iy+constants.Iz)*q*r+(constants.Ix^2-constants.Ix*constants.Iy+constants.Ixz^2)*p*q)/gam;

%Attitudes
%phi dot eqn.
xdot(7)=p+q*sphi*sthe/cthe+r*cphi*sthe/cthe;
%theta dot eqn.
xdot(8)=q*cphi-r*sphi;
%psi dot eqn.
xdot(9)=q*sphi/cthe+r*cphi/cthe;

%Position
xdot(10)=u*cthe*cpsi+v*(sphi*sthe*cpsi-cphi*spsi)+w*(cphi*sthe*cpsi+sphi*spsi);
xdot(11)=u*cthe*spsi+v*(sphi*sthe*spsi+cphi*cpsi)+w*(cphi*sthe*spsi-sphi*cpsi);
xdot(12)=-u*sthe    +v*sphi*cthe                 +w*cphi*cthe;

return;
