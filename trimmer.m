function [x0trim,u0trim,itrim]=trimmer(acfunction,x0,u0,targ_des,constants)


%This module is used to calculate a trim (or equilibrium) condition for a dynamic model
%This variant calculates xdot as an azimuithal average over psi = 0 to 360/NB
%Need model constant indicating index of psi

XSCALE=constants.XSCALE;
YSCALE=constants.YSCALE;

TRIMVARS=constants.TRIMVARS;
TRIMTARG=constants.TRIMTARG;

NSTATES=constants.NSTATES;
NCTRLS=constants.NCTRLS;

x0trim=x0;
u0trim=u0;

it=0;
err=100;
trim_tol=5e-4;
itmax=80;

xdot_in=zeros(constants.NSTATES,1);
idx_targ_state=find(constants.TRIMTARG<=constants.NSTATES); % Sometimes the number of Trim Vairables is more than Trim Targets.
xdot_in(constants.TRIMTARG(idx_targ_state))=targ_des(constants.TRIMTARG(idx_targ_state));
daztrim=2*pi/constants.NB/constants.NAZTRIM;

disp('Iteration     Max. Error');

while ((it<itmax)&&(err>trim_tol))
 it=it+1;
 %Azimuthal average of xdot and y
 % I dont understand what does this Azimuthal avergae means, but I think I
 % can get it after reading the codes.
 xdot0=zeros(constants.NSTATES,1);
 y0=zeros(constants.NOUT,1);
 
 for iaz=1:constants.NAZTRIM
     x0trim(constants.IDXAZ)=(iaz-1)*daztrim;
     [xdotaz,yaz]=feval(acfunction,x0trim,u0trim,xdot_in,constants);
     xdot0=xdot0+xdotaz/constants.NAZTRIM;
     y0=y0+yaz/constants.NAZTRIM;
 end
 x0trim(constants.IDXAZ)=0.;
 
 targvec_full=[xdot0;y0];
 targvec=targvec_full(TRIMTARG);
 targ_err=targvec-targ_des(TRIMTARG);
 scalevec_full=[XSCALE;YSCALE];
 err=max(abs(targ_err)./scalevec_full(TRIMTARG));
 disp([num2str(it), '            ' ,num2str(err)]);
 
 if (err>trim_tol)
  [~,~,~,A,B,C,D]=linearize(acfunction,x0trim,u0trim,xdot_in,constants);
  %Don't use M in trimmer, as it is prone to diverge
  Jac=[A B;C D];
  Jac = Jac(TRIMTARG,TRIMVARS);
  trimvec=[x0trim;u0trim];
  trimvec(TRIMVARS)=trimvec(TRIMVARS)-0.5*pinv(Jac)*targ_err;
  x0trim=trimvec(1:NSTATES);
  u0trim=trimvec(NSTATES+1:NSTATES+NCTRLS);
 end
 
  
end

if (err>trim_tol)
    disp('Warning: Trim not acheived');
    itrim=0;
else
    disp(['Successful trim']);
    itrim=1;
    
end

return;