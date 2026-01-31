function [FM,yvec]=SimpleAero(xvec,uvec,constants)

%Simple static aerodynamic model of a helicopter.
%Inputs: xvec = state vector = [u v w p q r phi theta psi x y z]'
%        uvec = control vector = [theta1c theta1s theta0 theta0tr]'
%        contstants = data structure with aircraft and sim constants
%        Note that phi, theta, psi, x, y, and z are not used.
%Outputs: FM = forces and moments = [X Y Z L M N]' 
% Units of state variables are ft/sec, rad/sec,rad, and ft
% Units of input vector are degrees
% Units of force and moment vector are lbs and ft-lbs

%State and control vairables. Note that phi, theta, psi, x, y, and z are
%not used.
u=xvec(1);
v=xvec(2);
w=xvec(3);
p=xvec(4);
q=xvec(5);
r=xvec(6);

%Convert control angles to radians
theta1c=uvec(1)*pi/180;
theta1s=uvec(2)*pi/180;
theta0=uvec(3)*pi/180;
theta0tr=uvec(4)*pi/180;

%Main Rotor Model
%Get hub velocities
Vh=[u;v;w]+cross([p;q;r],constants.rMR);
uh=Vh(1);
vh=Vh(2);
wh=Vh(3);

%Main Rotor advance ratios
Vinplane=sqrt(uh^2+vh^2);
mu=Vinplane/(constants.OMEGA*constants.RADIUS);
muz=wh/(constants.OMEGA*constants.RADIUS);
mu2=mu*mu;

%Get wind axis transformation
psiw=atan2(vh,uh);
cpsiw=cos(psiw);
spsiw=sin(psiw);
Th2w=[cpsiw spsiw;-spsiw cpsiw];
Tw2h=Th2w';

%Non-dimensional angular rates in wind axes
temp=Th2w*[p;q];
phw=temp(1)/constants.OMEGA;
qhw=temp(2)/constants.OMEGA;

%Cyclic pitch translated to wind axes
temp=Th2w*[theta1s;theta1c];
theta1sw=temp(1);
theta1cw=temp(2);

%CT and lambda0 iteration, solves for inflow and thrust coefficient
lambda0=0.05;

delta_lambda=1;
iter=0;
CT=0.;
while( (abs(delta_lambda)>eps) && (iter<200) )
     CT=0.5*constants.A0*constants.SOL*((1/3+0.5*mu2)*theta0+0.5*mu*(theta1sw+0.5*phw)+0.5*(muz-lambda0)+0.25*(1+mu2)*constants.TWIST);
     lamtot2=mu2+(lambda0-muz)^2;
     delta_lambda=-(2.*lambda0*sqrt(lamtot2)-CT)*lamtot2/ ...
                           (2.*lamtot2^1.5+0.25*constants.A0*constants.SOL*lamtot2-CT*(muz-lambda0));
     lambda0=lambda0+0.5*delta_lambda;
     iter=iter+1;
end

%Thrust
T=CT*constants.RHO*constants.OMEGA^2*pi*constants.RADIUS^4;

%Uniform inflow assumption in this model.
lambda1cw=0.;
lambda1sw=0.;

%Wake skew angle in deg
chi = atan2(mu,lambda0-muz)*180./pi;

%Quasi-Steady Flapping in Wind Axes (page 107 of Padfield)
Abt = [-8/3*mu*(1+0.5*mu2)     -2*mu*(1+0.5*mu2)     -(1+2*mu^2)            0;   ...
      -constants.GAMMA/6*mu*(1+0.5*mu2) -2*constants.GAMMA*mu/15*(1+mu2/3)      -2*constants.GAMMA*mu2/9.    (1-0.5*mu2*mu2)];
Abl = [-2*mu*(1+0.5*mu2)             (1+0.5*mu^2)                   0.; ...
      -2*constants.GAMMA*mu/9*(1-0.5*mu2)  constants.GAMMA/9.*mu^2+0.5*constants.GAMMA/9*mu2  -(1-0.5*mu2) ];
Abo = [-(1+0.5*mu^2)    16/constants.GAMMA*(1+0.5*mu2); 16/constants.GAMMA*(1-0.5*mu2)+constants.GAMMA/9.*mu^2  (1-0.5*mu2)];

temp=Abt*[theta0;constants.TWIST;theta1sw;theta1cw]+Abl*[muz-lambda0;lambda1sw;lambda1cw]+Abo*[phw;qhw];
beta1cw=temp(1);
beta1sw=temp(2);

%Convert back to hub system
temp=Tw2h*[beta1sw;beta1cw];
beta1s=temp(1);
beta1c=temp(2);

%Torque
delta=constants.DELTA0+constants.DELTA2*CT*CT;
CQ=-(muz-lambda0)*CT+0.125*delta*constants.SOL*(1+(7/3)*mu^2);
Q=CQ*constants.RHO*constants.OMEGA^2*pi*constants.RADIUS^5;

%Translate rotor forces and moments to CG
Xr=T*beta1c;
Yr=-T*beta1s;
Zr=-T;
MvecMR=[0;0;Q]+cross(constants.rMR,[Xr;Yr;Zr]);
Lr=MvecMR(1);
Mr=MvecMR(2);
Nr=MvecMR(3);

%Fuselage
%Induced downwash on fuselage
klambda_FUSE=table_lookup(constants.CHITABF,constants.KLAMBDAF,chi);
vif=2.*lambda0*constants.OMEGA*constants.RADIUS*klambda_FUSE;
%Body force per method of Stevens, Lewis, and Johnson (page 658)
Ff=-0.5*constants.RHO*[constants.SXFUSE*abs(u)*u;constants.SYFUSE*abs(v)*v;constants.SZFUSE*abs(w-vif)*(w-vif)];
%Transform to body forces
Xf=Ff(1);
Yf=Ff(2);
Zf=Ff(3);
MvecF=cross(constants.rFUSE,Ff);
Lf=MvecF(1);
Mf=MvecF(2);
Nf=MvecF(3);
%Horizontal Stabilizer
%Induced downwash on horizontal tail
klambda_HT=table_lookup(constants.CHITABHT,constants.KLAMBDAHT,chi);
viht=2.*lambda0*constants.OMEGA*constants.RADIUS*klambda_HT;
%Calc. local alpha
Vht=[u;v;w-viht]+cross([p;q;r],constants.rHT);
uht=Vht(1);
wht=Vht(3);
alphaht=atan2(wht,uht);

%Lift force
CLht=min(constants.AHT*abs(alphaht),1.)*sign(alphaht); %Max. CL of 1, will effectively give drag force in trim
LIFTht=0.5*constants.RHO*(Vht'*Vht)*constants.SHT*CLht;

%Transform to body forces at CG
Xht=0.;
Yht=0.;
Zht=-LIFTht;
MvecHT=cross(constants.rHT,[Xht;Yht;Zht]);
Lht=MvecHT(1);
Mht=MvecHT(2);
Nht=MvecHT(3);

%Tail Rotor
%Local velocities at tail rotor hub in body CS
Vtr=[u;v;w]+cross([p;q;r],constants.rTR);
%Transfrom to TR Coordinate System
Ttrg=[1 0 0;0 0 1;0 -1 0];
Vtr_tr=Ttrg*Vtr;
utr=Vtr_tr(1);
vtr=Vtr_tr(2);
wtr=Vtr_tr(3);
mutr=sqrt(utr^2+vtr^2)/(constants.OMEGTR*constants.RTR);
muztr=wtr/(constants.OMEGTR*constants.RTR);

%CT and lambda0 iteration for Tail Rotor
lambda0tr=0.05;

delta_lambda=1;
iter=0;
CTtr=0.;
while( (abs(delta_lambda)>eps) && (iter<200) )
     CTtr=0.5*constants.A0TR*constants.SOLTR*((1/3+0.5*mutr^2)*theta0tr+0.5*(muztr-lambda0tr)+0.25*(1+mutr^2)*constants.TWISTTR);
     lamtot2=mutr^2+(lambda0tr-muztr)^2;
     delta_lambda=-(2.*lambda0tr*sqrt(lamtot2)-CTtr)*lamtot2/ ...
                  (2.*lamtot2^1.5+0.25*constants.A0TR*constants.SOLTR*lamtot2-CTtr*(muztr-lambda0tr));
     lambda0tr=lambda0tr+0.5*delta_lambda;
     iter=iter+1;
end

%Calc tail rotor thrust and transform to body frame at CG
Ttr=CTtr*constants.RHO*constants.OMEGTR^2*pi*constants.RTR^4;
CQtr=-(muztr-lambda0tr)*CTtr+0.125*constants.DELTATR*constants.SOLTR*(1+(7/3)*mutr^2);
Qtr=CQtr*constants.RHO*constants.OMEGTR^2*pi*constants.RTR^5;

%TR Force Vector Transformed to body system
Ftr=Ttrg'*[0;0;-Ttr];
Xtr=Ftr(1);
Ytr=Ftr(2);
Ztr=Ftr(3);
%Resolve force and moment about CG
MvecTR=cross(constants.rTR,Ftr)+[0.;Qtr;0.];
Ltr=MvecTR(1);
Mtr=MvecTR(2);
Ntr=MvecTR(3);

%Sum total forces and moments
X=Xr+Xf+Xtr+Xht;
Y=Yr+Yf+Ytr+Yht;
Z=Zr+Zf+Ztr+Zht;
L=Lr+Lf+Ltr+Lht;
M=Mr+Mf+Mtr+Mht;
N=Nr+Nf+Ntr+Nht;
FM=[X;Y;Z;L;M;N];

Vtot=sqrt(u^2+v^2+w^2);
%Output is total airspeed amd the sensed accelerations at the CG
yvec=[Vtot;X/constants.mass;Y/constants.mass;Z/constants.mass];

return;

