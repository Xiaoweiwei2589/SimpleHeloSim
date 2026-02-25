function[constants]=DefineConstants
%Set aircraft properties and constants
% Similar to a Bell 206L

%Mass properties
%Accel of gravity (ft/s^2)
constants.g=32.17;
%Gross weight in lbs
constants.W=4000.;
%Aircraft Mass (slugs)
constants.mass=constants.W/constants.g;
% Moment of Inertias sl-ft^2
constants.Ix=900;
constants.Iy=4000;
constants.Iz=3400;
constants.Ixz=300;

%Air density (slug/ft^3) - sea level standard.
constants.RHO=0.002378;   

%Rotor properties
%Rotor Radius (ft)
constants.RADIUS=18.;
%Rotor speed (rad/sec)
constants.OMEGA=390.*2*pi/60.;
%Solidity
constants.SOL=0.04;
%Blade lift slope (1/rad)
constants.A0=5.73;
%Blade Twist (rad)
constants.TWIST=-11.*pi/180.;
%Lock number = rho*a*c*R^4/Ibeta
constants.GAMMA=4.5;
%Location of Rotor (ft)
constants.rMR=[0.;0.;-5.2];
% Rotor Blade profile CD parameters
constants.DELTA0=0.01;
constants.DELTA2=250.;
%Main rotor number of blades
constants.NB=2;

%Fuselage flat plate drag area in three axes (ft^2)
constants.SXFUSE=16.0;
constants.SYFUSE=110.0;
constants.SZFUSE=80.0;
%Location (assume coincident with CG)
constants.rFUSE=[0.;0.;0.];
%Downwash factors on fuselage, assume downwash goes away as chi approaches 70 deg
constants.CHITABF=[0. 50. 70. 180.];
constants.KLAMBDAF=[1. 1. 0. 0.];


%Horizontal Stabilizer
%Area (ft^2)
constants.SHT=11.;  
%Lift Slope (1/rad)
constants.AHT=2.3;
%Location (ft)
constants.rHT=[-13.0;0.;0.];
%Downwash factors on horizontal tail, assume downwash goes away as chi approaches 80 deg
constants.CHITABHT=[0. 60. 80. 180.];
constants.KLAMBDAHT=[1. 1. 0. 0.];

%Tail Rotor Properties
%Tail rotor speed (rad/sec)
constants.OMEGTR=2550.*2*pi/60.;
%Tail rotor radius (ft)
constants.RTR=2.7;
%Tail rotor blade lift slope (1/rad)
constants.A0TR=5.73;
%Tail rotor solidity
constants.SOLTR=0.21;
%Tail rotor twist (rad)
constants.TWISTTR=-4.*pi/180.;
%Location of tail rotor rel. to CG (ft)
constants.rTR=[-21.;0.;-1.4];
%Tail rotor blade profile drag
constants.DELTATR=0.012;

%Number of states, fuselage states, rotor states, propulsion states
constants.NSTATES=12; 
constants.NFSTATES=12; % I guess NSTATES and NFSTATES are the same thing.
constants.NRSTATES=0;
constants.NPSTATES=0;
%Number of controls
constants.NCTRLS=4;    % [lateral cyclic, longi cyclic, MR collective, TR collective]
%Number of outputs
constants.NOUT=4;      % What are outputs?

%Constants used by trim and linearization algorithm
constants.XSCALE=[ [0.1 0.1 0.1 ],pi/180*0.1*ones(1,6), [0.1 0.1 0.1]]'; %Scales state derivative when computing target error
constants.YSCALE=[0.1 0.1 0.1 0.1]'; %Scales outputs when computing target error
constants.DELULIN=[0.1 0.1 0.1 0.1]; %Control perturbations when linearizing
constants.DELXLIN=constants.XSCALE*0.1; %State derivatives when linearizing
constants.TRIMTARG=[1:12];         %Trim target indices (Trim out state derivatives only in this case)
constants.TRIMVARS=[1:8,13:16];    %Trim variable indicies.  Trim vleocity, rates, roll/pitch attitude, and 4 controls.
constants.TRIMTOL=5.e-4; %Will trim until all components of scaled error vector are < TRIMTOL
%Index of rotor azimuth state (no need in Simple Helo)
constants.IDXAZ=[];
%Numbder of azimuth locations to average over during trim.  No azimuth averaging needed in Simple Helo
constants.NAZTRIM=1;
return