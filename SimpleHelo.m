function [x_dot,y]=SimpleHelo(x,u,~,constants)

% ~ this input argument exists, but I am intentionally ignoring it.

%Simple model of a helicopter using static aerodynamic model for rotor,
%tail rotor, horz tail, and fuselage.
%This the main executive, calls each of the sub-modules for trim,
%linearization, and dynamic simulation
% Input: time, t
%        aircraft state vector, x = [u v w p q r phi theta psi X Y Z]'
%        control input, u = [theta1c theta1s theta0 theta0tr]'
%        state derivative input, xdot *NOT USED IN THIS MODEL*, use ~ above for place holder
%        constants, data structure with vehicle and simulation constants
% Output: state derivative, x_dot
%         output vector, y 
%         y is total airspeed amd the sensed accelerations at CG
%         y = [Vtot;X/constants.mass;Y/constants.mass;Z/constants.mass]

[FM,y]=SimpleAero(x,u,constants); 
x_dot=eqnmot(x,FM,constants);

return;