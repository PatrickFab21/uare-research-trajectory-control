function UAV_3D_minEnergy_PathLossDominant()
% UAV_3D_minEnergy_PathLossDominant:
%
%  Este código primal-dual está configurado para que la restricción
%  de PATH-LOSS sea, con alta probabilidad, la dominadora.
%
%  (1) Calcula c0 según eq.(38a)
%  (2) Restricción beamwidth => (R+Rc)<= H tan(phi), 
%  (3) Restricción v >= v_min => con v_min~0,
%  (4) Permite H muy grande para no saturar la altitud,
%  (5) Permite T_p muy libre (T_p_min=1 p.ej.)
%
%  De esa forma, cambiando el environment ("SU","UB","DU","HU")
%  cambiarás => B,C,etaLoS,etaNLoS => la cota de path-loss => distintos
%  resultados en R*,H*,T_p*.
%
% ---------------------------------------------------------------------
% Uso:
%    1) Ajusta "environment" (p.e. "SU","UB","DU","HU") 
%    2) Cambia path-loss dB si deseas
%    3) Ejecuta:
%         >> UAV_3D_minEnergy_PathLossDominant
%
%    Verás que la constraint de path-loss se vuelve “tight” y 
%    al cambiar environment verás R*,H*,Tp* distintos.
% ---------------------------------------------------------------------

%% 1) Configuración
params = struct();

% Covertura:
params.Rc   = 56.13; 
params.Hmin = 1;      % relaja -> 1 m
params.Hmax = 3000;   % sube -> 3000 m

% Velocidades y periodo
params.vmax   = 30;   % subimos v_max
params.vmin   = 5;    % ~0 => no limita
params.Tp_max = 200;  
params.Tp_min = 10;    % p.ej. 1 s

% Haz de antena => ancho ~85 deg => no limita
params.phi_deg=85; 
params.phi_rad=deg2rad(params.phi_deg);

% Modelo de propulsión (eq.(11)):
params.gamma1=504.6; params.gamma2=6.53e-5; params.gamma3=6.53e-3;
params.gamma4=687.88; params.gamma5=90.20; params.g=9.81;

% PATH-LOSS threshold => p.ej. 145 dB (aún más estricto que 155)
PL_dB=145;  
params.TH_PL = 10^(PL_dB/10); 
params.delta = 2.5;
params.Pser =0.8;

% Selección del environment => "SU","UB","DU","HU"
environment= 'UB';  

% Cargar: B_logistic, C_logistic, etaLoS_lin, etaNLoS_lin
logEnv= getLogisticParams(environment);
params.B_logistic   = logEnv.B_logistic;
params.C_logistic   = logEnv.C_logistic;
params.etaLoS_lin   = 10^(logEnv.etaLoS_dB /10);
params.etaNLoS_lin  = 10^(logEnv.etaNLoS_dB/10);

% eq.(10): freq
f_c= 2.5e9; c_light=3e8;
G0_lin= 2.9e4/( (2*params.phi_rad)^2 );
params.K0= (1/G0_lin)* (4*pi*f_c/c_light)^params.delta;

% Step-size
params.alpha0=0.1; 
params.alpha1=1.5;
params.IterMax=200;

%% 2) A0,B0,c0 => eq.(25),(38a)
[params.A0, params.B0, params.c0] = computeABC(params);

fprintf('\n*** CORREMOS EN environment=%s, PATH-LOSS=%.1f dB\n',environment,PL_dB);

%% 3) Solver
[R_opt,H_opt,Tp_opt,E_opt,history] = solveUAVviaPrimalDual(params);

%% 4) Resultados
fprintf('\n=== RESULTADO ÓPTIMO ===\n');
fprintf('R*    = %.3f m\n', R_opt);
fprintf('H*    = %.3f m\n', H_opt);
fprintf('T_p*  = %.3f s\n', Tp_opt);
vfin= (2*pi*R_opt)/Tp_opt;
fprintf('v*    = %.3f m/s\n', vfin);
fprintf('E*    = %.3f J\n', E_opt);

figure;
plot(1:params.IterMax,history.E,'o-','LineWidth',1.4);
grid on; xlabel('Iter'); ylabel('E [J]');
title(sprintf('Energy evolution - env=%s, PL=%.1fdB',environment,PL_dB));

% Revisar si la path-loss saturó => phi5 ~0 ?
phiEval= evaluateConstraints(log([R_opt,H_opt,Tp_opt,0,0]),params);
fprintf('\nConstraints phi_i= \n');
disp(phiEval);

end
% -------------------------------------------------------------------------

function envOut= getLogisticParams(envType)
switch upper(envType)
    case 'SU' % Sub-Urban
        envOut.B_logistic   =4.88;
        envOut.C_logistic   =0.43;
        envOut.etaLoS_dB    =0.1;
        envOut.etaNLoS_dB   =21;
    case 'UB' % Urban
        envOut.B_logistic   =9.61;
        envOut.C_logistic   =0.16;
        envOut.etaLoS_dB    =1.0;
        envOut.etaNLoS_dB   =20;
    case 'DU' % Dense Urban
        envOut.B_logistic   =12.08;
        envOut.C_logistic   =0.11;
        envOut.etaLoS_dB    =1.6;
        envOut.etaNLoS_dB   =23;
    case 'HU' % High Rise Urban
        envOut.B_logistic   =27.23;
        envOut.C_logistic   =0.08;
        envOut.etaLoS_dB    =2.3;
        envOut.etaNLoS_dB   =34;
    otherwise
        error('Entorno desconocido. Usa: SU, UB, DU, HU');
end
end

function [A0,B0,c0]= computeABC(par)
thDegMin= invertLogisticLoS(par.Pser,par.B_logistic,par.C_logistic);
A0= tan(deg2rad(thDegMin));
tmpNum= par.K0*( par.etaNLoS_lin-(par.etaNLoS_lin-par.etaLoS_lin)*par.Pser);
val= tmpNum/ par.TH_PL;
B0= val^(2/par.delta);

c0= computeC0eq38a(par);

fprintf('\nEn computeABC => A0=%.4f, B0=%.4e, c0=%.4f\n',A0,B0,c0);
end




function c0Val= computeC0eq38a(par)
% eq.(38a)
% c0= 1+ 2*vmax^2 / [ gamma5 * (zMin)^2 ]
vVec= linspace(0,par.vmax,40);
tpVec= linspace(0,par.Tp_max,40);
zMin= inf;
for vv= vVec
    for tt= tpVec
        if tt<=0, continue; end
        ac= (2*pi*vv)/tt; 
        termInside= 1+(ac^2/(par.g^2))+(vv^4/(par.gamma5^2));
        bigSqrt= sqrt(termInside);
        zSquared= bigSqrt-(vv^2/par.gamma5);
        if zSquared>0
            zVal= sqrt(zSquared);
            if zVal<zMin
                zMin= zVal;
            end
        end
    end
end
if zMin<1e-8, zMin=1e-8; end
c0Val= 1+ (2*(par.vmax^2))/( par.gamma5*(zMin^2));
end

function tDeg = invertLogisticLoS(pser, Bval, Cval)
% Calcula theta (en deg) tal que:
%   PLoS(theta)= 1/(1 + C e^{-B (theta-C)})
% => pser= PLoS(theta)
%
% Maneja casos pser<=0 o pser>=1 con valores 'por defecto'.

    % Valor por defecto
    tDeg = 90;

    % 1) Manejo de extremos
    if pser<=0
        % si pser <=0 => la eq. no tiene mucho sentido => 
        % definimos tDeg= 90 o 0 según convenga
        tDeg= 90;
        return;
    end
    if pser>=1
        % si pser>=1 => eq. => theta=??? => forzamos tDeg=0 => 
        tDeg= 0;
        return;
    end

    % 2) Calculamos lhs= (1/pser)-1 => si es <=0 => tDeg=0
    lhs = (1/pser) - 1;
    if lhs <= 0
        tDeg= 0;
        return;
    end

    % 3) Dividimos por Cval => si sale <=0 => tDeg=0
    lhs2 = lhs / Cval;
    if lhs2 <= 0
        tDeg= 0;
        return;
    end

    % 4) Valor nominal => eq. => 
    th = Cval - (1/Bval)*log(lhs2);

    % 5) Acotamos en [0,90]
    th= max(0, th);
    th= min(90, th);
    tDeg= th;
end


%% ========================================================================
% SOLVER
%% ========================================================================
function [R_opt,H_opt,Tp_opt,E_opt,history]= solveUAVviaPrimalDual(params)
xR0= log(params.Rc/2);
xH0= log((params.Hmax+params.Hmin)/2);
xT0= log((params.Tp_max+params.Tp_min)/2);
X_log= [xR0; xH0; xT0; 0; 0];

mu= zeros(8,1);
history.Xlog= zeros(5,params.IterMax);
history.mu  = zeros(8,params.IterMax);
history.E   = zeros(1,params.IterMax);

for k=1:params.IterMax
    [gX,gMu,E_val]= gradients_primal_dual(X_log,mu,params);
    history.Xlog(:,k)= X_log;
    history.mu(:,k)= mu;
    history.E(k)= E_val;
    
    % step-size
    nGX= norm(gX); 
    nGM= norm(gMu);
    gp= params.alpha0/ sqrt(params.alpha1+ nGX^2);
    gd= params.alpha0/ sqrt(params.alpha1+ nGM^2);
    
    Xcand= X_log - gp*gX;
    Xupd= projectXlogFeasible(Xcand, params);
    
    muCand= mu+ gd*gMu;
    muUpd= max(muCand,0);
    
    X_log= Xupd;
    mu= muUpd;
end

[R_opt,H_opt,Tp_opt,E_opt]= computeFinalPrimal(X_log,params);
end

function [gradX, gradMu, E_val]= gradients_primal_dual(X_log, mu, P)
xR= X_log(1); xH= X_log(2); xT= X_log(3);
xY= X_log(4); xZ= X_log(5);

R= exp(xR); H= exp(xH); Tper= exp(xT);

% eq.(37) => E
c1= P.gamma1* exp(xT);
c2= P.gamma1* P.gamma2*(2*pi)^2* exp(2*xR- xT);
c3= P.gamma3*(2*pi)^3* exp(3*xR-2*xT);
c4= P.gamma4* exp(xY+ xT- xZ);
E_val= c1+ c2+ c3+ c4;

% eq.(36b)... => 8 constraints
% phi1 => eq.(36b) c0 => Induced
phi1= (1/P.c0)* exp(-4*xY -2*xZ)-1;
% phi2 => eq.(36c)
termB= (2*pi)^4/(P.g^2);
phi2= exp(2*xY)+ termB* exp(2*xR+2*xY-4*xT)-1;
% phi3 => (R+Rc)* e^{-xH}/tan(phi) -1
phi3= (R+P.Rc)* exp(-xH)/ tan(P.phi_rad)-1;
% phi4 => coverage A0 => ...
phi4= P.A0* exp(-xH)*( R+ P.Rc )-1;
% phi5 => path-loss B0 => ...
phi5= P.B0*( exp(2*xR)+ exp(2*xH)+ 2*P.Rc*R+ P.Rc^2 )-1;
% phi6 => v <= vmax
phi6= (2*pi/ P.vmax)* exp(xR- xT)-1;
% phi7 => v >= vmin => 1 - (2*pi/vmin) e^(xR-xT)<=0
alpha7= (2*pi)/ P.vmin;
phi7= 1- alpha7* exp(xR- xT);
% phi8 => T_p >= T_{p,min} => 1 - (1/Tp_min) e^(xT)<=0
alpha8= 1/(P.Tp_min);
phi8= 1- alpha8* exp(xT);

gradMu= [phi1;phi2;phi3;phi4;phi5;phi6;phi7;phi8];

% deriv E
dE_xR= 2*c2+ 3*c3; 
dE_xH= 0;
dE_xT= c1 - c2 -2*c3+ c4;
dE_xY= c4; 
dE_xZ=- c4;

% phi1
dphi1_xY= -4*(1/P.c0)* exp(-4*xY -2*xZ);
dphi1_xZ= -2*(1/P.c0)* exp(-4*xY -2*xZ);

% phi2
tmpA= exp(2*xY);
tmpB= termB* exp(2*xR+ 2*xY- 4*xT);
dphi2_xR= 2* tmpB;
dphi2_xY= 2* tmpA+ 2* tmpB;
dphi2_xT=-4* tmpB;

dphi3_xR= exp(-xH)/ tan(P.phi_rad);
dphi3_xH= (R+P.Rc)* (-1)* exp(-xH)/ tan(P.phi_rad);

dphi4_xR= P.A0* exp(-xH)* exp(xR);
dphi4_xH=-P.A0* exp(-xH)*( R+P.Rc );

dphi5_xR= P.B0*(2* exp(2*xR)+ 2*P.Rc* exp(xR));
dphi5_xH= P.B0*(2* exp(2*xH));

dphi6_xR= (2*pi/ P.vmax)* exp(xR- xT);
dphi6_xT=-(2*pi/ P.vmax)* exp(xR- xT);

dphi7_xR= - alpha7* exp(xR- xT);
dphi7_xT= alpha7* exp(xR- xT);

dphi8_xT= - alpha8* exp(xT);

gradX= zeros(5,1);
gradX(1)= dE_xR ...
   + mu(2)* dphi2_xR + mu(3)* dphi3_xR + mu(4)* dphi4_xR ...
   + mu(5)* dphi5_xR + mu(6)* dphi6_xR + mu(7)* dphi7_xR;
gradX(2)= dE_xH ...
   + mu(3)* dphi3_xH + mu(4)* dphi4_xH + mu(5)* dphi5_xH;
gradX(3)= dE_xT ...
   + mu(2)* dphi2_xT + mu(6)* dphi6_xT + mu(7)* dphi7_xT + mu(8)* dphi8_xT;
gradX(4)= dE_xY ...
   + mu(1)* dphi1_xY + mu(2)* dphi2_xY;
gradX(5)= dE_xZ ...
   + mu(1)* dphi1_xZ;

end

function Xproj= projectXlogFeasible(Xcand, P)
xR= Xcand(1); xH= Xcand(2); xT= Xcand(3);
xY= Xcand(4); xZ= Xcand(5);

% 1) R <= rmax
rmax= min( P.Rc, (P.vmax* P.Tp_max)/(2*pi) );
xR= min(xR, log(rmax));

% 2) H in [Hmin,Hmax]
xH= max(xH, log(P.Hmin));
xH= min(xH, log(P.Hmax));

% 3) T_p in [Tp_min, Tp_max]
xT= max(xT, log(P.Tp_min));
xT= min(xT, log(P.Tp_max));

% 4) Forzar v>=v_min => (xR- xT)>= ln(vmin/(2*pi))
vMinDelta= log( P.vmin/(2*pi) );
delta= xR- xT;
if delta< vMinDelta
   xR= xT+ vMinDelta;
end

% 5) Forzar beamwidth => (R+Rc)<= H tan(phi)
Rcand= exp(xR);
Hcand= exp(xH);
beamMax= Hcand* tan(P.phi_rad)- P.Rc;
if beamMax<0
   Rcand=1e-8;
else
   if Rcand> beamMax
       Rcand= beamMax;
       if Rcand<1e-8
           Rcand=1e-8;
       end
   end
end
xR= log(Rcand);

% y>0 => xY>=-20
xY= max(xY, -20);
% z>0 => xZ>=-20
xZ= max(xZ, -20);

Xproj= [xR;xH;xT;xY;xZ];
end

function [Rfin,Hfin,Tpfin,Efin]= computeFinalPrimal(X_log, P)
xR= X_log(1); xH= X_log(2); xT= X_log(3);
xY= X_log(4); xZ= X_log(5);

Rfin= exp(xR);
Hfin= exp(xH);
Tpfin= exp(xT);

c1= P.gamma1*exp(xT);
c2= P.gamma1*P.gamma2*(2*pi)^2*exp(2*xR- xT);
c3= P.gamma3*(2*pi)^3*exp(3*xR-2*xT);
c4= P.gamma4*exp(xY+ xT- xZ);
Efin= c1+ c2+ c3+ c4;
end

function phiVals= evaluateConstraints(xLog, P)
% Devuelve un vector con las 8 phi_i en la solucion final
xR= xLog(1); xH= xLog(2); xT= xLog(3);
xY= xLog(4); xZ= xLog(5);

R= exp(xR); H= exp(xH); 
phi1= (1/P.c0)* exp(-4*xY -2*xZ)-1;
termB= (2*pi)^4/(P.g^2);
phi2= exp(2*xY)+ termB* exp(2*xR+2*xY-4*xT)-1;
phi3= (R+P.Rc)* exp(-xH)/ tan(P.phi_rad)-1;
phi4= P.A0* exp(-xH)*( R+ P.Rc )-1;
phi5= P.B0*( exp(2*xR)+ exp(2*xH)+ 2*P.Rc*R + P.Rc^2 )-1;
phi6= (2*pi/ P.vmax)* exp(xR- xT)-1;
alpha7= (2*pi)/ P.vmin;
phi7= 1- alpha7* exp(xR- xT);
alpha8= 1/(P.Tp_min);
phi8= 1- alpha8* exp(xT);

phiVals= [phi1; phi2; phi3; phi4; phi5; phi6; phi7; phi8];
end
