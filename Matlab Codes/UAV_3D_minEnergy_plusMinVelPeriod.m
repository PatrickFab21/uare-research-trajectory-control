function UAV_3D_minEnergy_plusMinVelPeriod()
% UAV_3D_minEnergy_plusMinVelPeriod:
%  Ejemplo de resolución primal-dual para optimizar (R,H,Tp) minimizando
%  la energía de un UAV Rotary-Wing en trayectoria circular, con:
%   - Restricciones de:
%       (1) R <= Rmax,
%       (2) Hmin <= H <= Hmax
%       (3) Tp <= Tp_max
%       (4) v <= v_max
%       (5) (R+Rc)/ H tan(phi) (beamwidth)
%       (6) coverage path-loss
%    +  se AÑADEN:
%       (7) v >= v_min
%       (8) Tp >= Tp_min
%
% (Basado en el guión anterior y el paper "Energy-minimizing 3D circular...")
%
% ---------------------------------------------------------------------

%% 1) CONFIGURACIÓN BÁSICA DE PARÁMETROS
params = struct();

% --- Cobertura y geometría
params.Rc     = 56.13;  % Radio a cubrir (m)
params.Hmin   = 30;     % Altura mínima (m)
params.Hmax   = 350;    % Altura máxima (m)

% --- Velocidades y períodos
params.vmax   = 13;    % v_max (m/s)
params.vmin   = 7;     % v_min (m/s) -> Restricción nueva
params.Tp_max = 50;    % T_p <= 13
params.Tp_min = 10;     % T_p >= 3    -> Restricción nueva

% --- Haz de antena
params.phi_deg= 80;  
params.phi_rad= deg2rad(params.phi_deg);

% --- Modelo de propulsión
params.gamma1=504.6; params.gamma2=6.53e-5; params.gamma3=6.53e-3;
params.gamma4=687.88; params.gamma5=90.20;
params.g     = 9.81;

% --- Path-loss threshold, etc. 
PL_dB= 155;
params.TH_PL= 10^(PL_dB/10);
params.delta= 2.5;
params.Pser = 0.8; 
%
% Ganancias/constantes de path-loss 
etaLoS_dB  = 1.0;  % (Ejemplo)
etaNLoS_dB = 20; 
params.etaLoS  = 10^(etaLoS_dB/10);
params.etaNLoS = 10^(etaNLoS_dB/10);
f_c= 2.5e9; c_light=3e8;
G0 = 2.9e4/((2*params.phi_deg)^2);
params.K0= (1/G0)*(4*pi*f_c/c_light)^params.delta; 

% --- Cálculo de ctes A0,B0,c0 (ver eq.(25),(38a) del paper)
[params.A0, params.B0, params.c0] = computeABC(params);

% --- Step-size primal-dual
params.alpha0= 0.1;
params.alpha1= 1.5;
params.IterMax= 10000;

%% 2) LLAMADA AL OPTIMIZADOR
[R_opt,H_opt,Tp_opt,E_opt,history] = solveUAVviaPrimalDual(params);

%% 3) RESULTADOS
fprintf('\n=== RESULTADO ÓPTIMO (con v_min y Tp_min) ===\n');
fprintf('R*   = %.3f m\n', R_opt);
fprintf('H*   = %.3f m\n', H_opt);
fprintf('Tp*  = %.3f s\n', Tp_opt);
vfin = (2*pi*R_opt)/Tp_opt;
fprintf('v*   = %.3f m/s\n', vfin);
fprintf('E*   = %.3f J\n', E_opt);

%% 4) GRAFICAR TRAZA DE LA ENERGÍA
figure;
plot(1:params.IterMax, history.E, 'o-','LineWidth',1.5);
grid on; xlabel('Iteración'); ylabel('E [J]');
title('Evolución de la Energía con restricciones (v_{min}, T_{p,min})');

end
% -------------------------------------------------------------------------
%
%              SUB-FUNCIONES
%
% -------------------------------------------------------------------------

function [A0,B0,c0] = computeABC(par)
% A modo ilustrativo: 
theta_deg_min = invertLogisticLoS(par.Pser); 
A0 = tan( deg2rad(theta_deg_min) );

tmpNum = par.K0 * (par.etaNLoS - (par.etaNLoS - par.etaLoS)* par.Pser);
val    = tmpNum / par.TH_PL;
B0     = val^(2/par.delta);

c0     = 2.0; % simplificado; ajusta si deseas más exactitud

end

function tDeg = invertLogisticLoS(pser)
% Ejemplo hardcode B=9.61, C=0.16 (urban)
B=9.61; C=0.16;
lhs= (1/pser) -1;
cVal=0.16;
lhs2= lhs/cVal;
th = C - (1/B)*log(lhs2);
th = max(th,0); th=min(th,90);
tDeg= th;
end

function [R_opt,H_opt,Tp_opt,E_opt,history] = solveUAVviaPrimalDual(params)
% Resuelve el GP con 8 constraints => eq.(36) + (dos extras)

% 1) INICIAL
xR0 = log(params.Rc/2);
xH0 = log( (params.Hmax+params.Hmin)/2 );
xT0 = log( (params.Tp_max + params.Tp_min)/2 );
xY0 = 0; 
xZ0 = 0;
X_log= [xR0;xH0;xT0;xY0;xZ0];

% 8 restricciones => mu(1..8)
mu= zeros(8,1);

IterMax= params.IterMax;
history.Xlog= zeros(5,IterMax);
history.mu  = zeros(8,IterMax);
history.E   = zeros(1,IterMax);

for k=1:IterMax
    [gradX, gradMu, E_val] = gradients_primal_dual(X_log, mu, params);
    
    history.Xlog(:,k)= X_log;
    history.mu(:,k)  = mu;
    history.E(k)     = E_val;
    
    % step-size
    nGX = norm(gradX); 
    nGM = norm(gradMu);
    g_p = params.alpha0 / sqrt(params.alpha1 + nGX^2);
    g_d = params.alpha0 / sqrt(params.alpha1 + nGM^2);
    
    % primal
    Xcand= X_log - g_p*gradX;
    Xupd = projectXlogFeasible(Xcand, params);
    % dual
    muCand= mu + g_d*gradMu;
    muUpd = max(muCand,0);
    
    X_log= Xupd;
    mu   = muUpd;
end

[R_opt,H_opt,Tp_opt,E_opt] = computeFinalPrimal(X_log,params);

end

function [gradX, gradMu, E_val] = gradients_primal_dual(X_log, mu, P)
xR= X_log(1); xH= X_log(2); xTp= X_log(3);
xY= X_log(4); xZ= X_log(5);

R= exp(xR); H= exp(xH); Tp= exp(xTp);
y= exp(xY); z= exp(xZ);

% ---- Energía eq.(37)
c1= P.gamma1*exp(xTp);
c2= P.gamma1*P.gamma2*(2*pi)^2*exp(2*xR - xTp);
c3= P.gamma3*(2*pi)^3*exp(3*xR -2*xTp);
c4= P.gamma4*exp(xY + xTp - xZ);
E_val= c1 + c2 + c3 + c4;

% ---- Constraints "phi1..phi8" ----
% (1) eq.(36b)
phi1 = (1/P.c0)*exp(-4*xY -2*xZ) -1;
% (2) eq.(36c)
termB= (2*pi)^4 / (P.g^2);
phi2= exp(2*xY) + termB*exp(2*xR+2*xY -4*xTp) -1;
% (3) beamwidth eq.(36g)
phi3= (R+P.Rc)*exp(-xH)/tan(P.phi_rad) -1;
% (4) coverage eq.(36h)
phi4= P.A0*exp(-xH)*(R+P.Rc) -1;
% (5) path-loss eq.(36i)
phi5= P.B0*( exp(2*xR)+ exp(2*xH)+ 2*P.Rc*R + P.Rc^2 ) -1;
% (6) v <= v_max eq.(36j)
phi6= (2*pi/P.vmax)*exp(xR - xTp) -1;
% (7) v >= v_min =>  eq:  v >= v_min => 1 <= (2 pi / v_min) exp(...) =>  =>  phi7= 1 - (2*pi/v_min) exp(...) <=0
alpha7= (2*pi)/P.vmin;
phi7= 1 - alpha7*exp(xR - xTp);
% (8) Tp >= Tp_min => eq: exp(xTp)>=Tp_min => 1 <= (1/Tp_min) exp(xTp ) => => phi8= 1 - (1/Tp_min) exp(xTp) <=0
alpha8= 1/(P.Tp_min);
phi8= 1 - alpha8*exp(xTp);

% Lagrangiana L= E + sum_{i=1..8} mu_i phi_i
gradMu= [phi1;phi2;phi3;phi4;phi5;phi6;phi7;phi8];

% Deriv E
dE_dxR= 2*c2 + 3*c3; 
dE_dxH= 0;
dE_dxTp= c1 - c2 -2*c3 + c4;
dE_dxY= c4;
dE_dxZ=-c4;

% Deriv phi1..phi8
% phi1
dphi1_dY= -4*(1/P.c0)*exp(-4*xY -2*xZ);
dphi1_dZ= -2*(1/P.c0)*exp(-4*xY -2*xZ);

% phi2
tmpA= exp(2*xY);
tmpB= termB*exp(2*xR+2*xY -4*xTp);
dphi2_dR= 2* tmpB;
dphi2_dY= 2* tmpA + 2* tmpB;
dphi2_dTp=-4* tmpB;

% phi3
dphi3_dR= exp(-xH)/tan(P.phi_rad);
dphi3_dH= (R+P.Rc)*(-1)*exp(-xH)/tan(P.phi_rad);

% phi4
dphi4_dR= P.A0*exp(-xH)*exp(xR);
dphi4_dH=-P.A0*exp(-xH)*(R+P.Rc);

% phi5
dphi5_dR= P.B0*(2*exp(2*xR)+ 2*P.Rc*exp(xR));
dphi5_dH= P.B0*(2*exp(2*xH));

% phi6
dphi6_dR= (2*pi/P.vmax)*exp(xR - xTp);
dphi6_dTp=-(2*pi/P.vmax)*exp(xR - xTp);

% phi7= 1 - alpha7 e^(xR - xTp)
dphi7_dR= - alpha7*exp(xR - xTp);
dphi7_dTp= alpha7*exp(xR - xTp); 
% phi8= 1 - alpha8 e^(xTp)
dphi8_dTp= - alpha8* exp(xTp);

%% gradX= dL/d( xR,xH,xTp,xY,xZ )
gradX= zeros(5,1);
gradX(1)= dE_dxR ...
    + mu(2)* dphi2_dR + mu(3)* dphi3_dR + mu(4)* dphi4_dR ...
    + mu(5)* dphi5_dR + mu(6)* dphi6_dR + mu(7)* dphi7_dR;
gradX(2)= dE_dxH ...
    + mu(3)* dphi3_dH + mu(4)* dphi4_dH + mu(5)* dphi5_dH;
gradX(3)= dE_dxTp ...
    + mu(2)* dphi2_dTp + mu(6)* dphi6_dTp + mu(7)* dphi7_dTp + mu(8)* dphi8_dTp;
gradX(4)= dE_dxY ...
    + mu(1)* dphi1_dY + mu(2)* dphi2_dY;
gradX(5)= dE_dxZ ...
    + mu(1)* dphi1_dZ;

end

function Xproj = projectXlogFeasible(Xcand, P)
% xR <= log(rmax)
% xH in [log(Hmin), log(Hmax)]
% xTp in [log(Tp_min), log(Tp_max)]
% xY> -inf, xZ> -inf (ponemos un tope -20)

xR= Xcand(1);
xH= Xcand(2);
xTp= Xcand(3);
xY= Xcand(4);
xZ= Xcand(5);

% rmax (por la v <= v_max y R <= Rc si gustas)
% de manera rápida:
rmax = min( P.Rc, (P.vmax * P.Tp_max)/(2*pi) );
xR   = min(xR, log(rmax)); 

% altitud
xH = max(xH, log(P.Hmin));
xH = min(xH, log(P.Hmax));

% T_p en [ Tp_min, Tp_max ]
xTp = max(xTp, log(P.Tp_min));
xTp = min(xTp, log(P.Tp_max));

xY = max(xY, -20);
xZ = max(xZ, -20);

Xproj= [xR;xH;xTp;xY;xZ];
end

function [Rfin,Hfin,Tpfin,Efin] = computeFinalPrimal(X_log, P)
xR= X_log(1); xH= X_log(2); xTp= X_log(3);
xY= X_log(4); xZ= X_log(5);

Rfin= exp(xR); Hfin= exp(xH); Tpfin= exp(xTp);

c1= P.gamma1*exp(xTp);
c2= P.gamma1*P.gamma2*(2*pi)^2*exp(2*xR - xTp);
c3= P.gamma3*(2*pi)^3*exp(3*xR -2*xTp);
c4= P.gamma4*exp(xY + xTp - xZ);
Efin= c1 + c2 + c3 + c4;
end
