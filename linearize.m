function [F,G,M,A,B,C,D]=linearize(acfunction,x0,u0,xdot_in,constants)

%This module is used to linearize the governing equations.
%Used in trim algorithm and to derive a linear model for control design.

DELXLIN=constants.DELXLIN;
DELULIN=constants.DELULIN;
NSTATES=constants.NSTATES;
NCTRLS=constants.NCTRLS;
NOUT=constants.NOUT;

[xdot0,y0]=feval(acfunction,x0,u0,xdot_in,constants);
F=zeros(NSTATES);
G=zeros(NSTATES,NCTRLS);
M=eye(NSTATES);
C=zeros(NOUT,NSTATES);
D=zeros(NOUT,NCTRLS);

for k=1:NSTATES
    x_p=x0;
    x_p(k)=x_p(k)+DELXLIN(k);
    [xdot_p1,y_p1]=feval(acfunction,x_p,u0,xdot_in,constants);
    x_p(k)=x_p(k)-2*DELXLIN(k);
    [xdot_p2,y_p2]=feval(acfunction,x_p,u0,xdot_in,constants);
    F(:,k)=(xdot_p1-xdot_p2)/(2*DELXLIN(k));
    C(:,k)=(y_p1-y_p2)/(2*DELXLIN(k));
end

for k=1:NSTATES
    xd_p=xdot_in;
    xd_p(k)=xd_p(k)+DELXLIN(k);
    xdot_p1=feval(acfunction,x0,u0,xd_p,constants);
    xd_p(k)=xd_p(k)-2*DELXLIN(k);
    xdot_p2=feval(acfunction,x0,u0,xd_p,constants);
    M(:,k)=M(:,k)-(xdot_p1-xdot_p2)/(2*DELXLIN(k));
end

for k=1:NCTRLS
    u_p=u0; 
    u_p(k)=u_p(k)+DELULIN(k);
    [xdot_p1,y_p1]=feval(acfunction,x0,u_p,xdot_in,constants);
    u_p(k)=u_p(k)-2*DELULIN(k);
    [xdot_p2,y_p2]=feval(acfunction,x0,u_p,xdot_in,constants);
    G(:,k)=(xdot_p1-xdot_p2)/(2*DELULIN(k));
    D(:,k)=(y_p1-y_p2)/(2*DELULIN(k));
end

A=M\F;
B=M\G;

return;