clear all
close all


load Traj;   %  charger .......
plot(trajectoire(1,:),trajectoire(2,:),'o-b')
pause(0.1)
hold on

ts=1; % ..............
sigmau=2; % ..............
sigmav=0.0000001; % ..............
A=[1 0 ts 0;0 1 0 ts;0 0 1 0;0 0 0 1]; % ..............
B=[ts^2/2 0;0 ts^2/2;ts 0;0 ts]; % ..............


xi=[2000;3000]; % initialisation du vecteur .......................
plot(xi(1),xi(2),'*-r');hold on;
V=zeros(2,1);% ..............
Xi=[xi;V];% vecteur ..............   correspond à z(0|0) 
Mi=1000*eye(4);% matrice ..............   correspond à ................

Mk1=[-1500;1500];  % position de .......................
Mk2=[20000;20000]; % position de .......................
Mk3=[25000;500];   % position de .......................

% visualisation des ........................
plot(Mk1(1),Mk1(2),'s-k');hold on;
plot(Mk2(1),Mk2(2),'s-k');hold on;
plot(Mk3(1),Mk3(2),'s-k');hold on;
plot(Mk1(1),Mk1(2),'*-k');hold on;
plot(Mk2(1),Mk2(2),'*-k');hold on;
plot(Mk3(1),Mk3(2),'*-k');hold on;

c=3*10^8; % ............................
c=1/c;

for l=1:length(trajectoire),

% étape de ................................    
Xpredit=A*Xi; % vecteur de ........... correspond à ...(..|..) dans l'algorithme de Kalman
Mpredit=A*Mi*A'+sigmau^2*B*B';% matrice de ........... correspond à ...(..|..) dans l'algorithme de Kalman

% calcul du ........   de l'observation ......... car on a besoin de ...
% l'équation d'observation afin d'appliquer le filtre de Kalman
h1=c*(Xpredit(1:2)-Mk1)/(norm(Xpredit(1:2)-Mk1)) ;
h2=c*(Xpredit(1:2)-Mk2)/(norm(Xpredit(1:2)-Mk2)) ;
h3=c*(Xpredit(1:2)-Mk3)/(norm(Xpredit(1:2)-Mk3)) ;
% étendre les ...... par 2 zéros car ......
h1=[h1;0;0];
h2=[h2;0;0];
h3=[h3;0;0];
% remplir la ........
h=[h1 h2 h3];

%calcul de ........ correspond à ....(..|..) dans le filtre de Kalman 
Ypredit=c*[norm(Xpredit(1:2)-Mk1);norm(Xpredit(1:2)-Mk2);norm(Xpredit(1:2)-Mk3)];
%calcul de ......... correspond à ....(..|..) dans le filtre de Kalman
var_sortie=h'*Mpredit*h+sigmav^2*eye(3);


n1=sigmav*randn;   % génération du .......
n2=sigmav*randn;
n3=sigmav*randn;

% génération des vraies observations qu'on va utiliser pour la localisation
% correspond é ...... dans le modèle d'état
Yk=[c*norm(trajectoire(:,l)-Mk1)+n1 ; c*norm(trajectoire(:,l)-Mk2)+n2 ; c*norm(trajectoire(:,l)-Mk3)+n3] ;%Yk(n)

%TOA1(l)=abs(c*norm(trajectoire(:,l)-Mk1));
%relative_err1(l)=abs(n1)/TOA1(l);

%étape de ................
gain=Mpredit*h*inv(var_sortie); % calcul du ......correspond ? .......dans l'alg.
err=Yk-Ypredit; % correspond ? ..(..|..)-..(..|..) dans l'alg.

Xestime=Xpredit+gain*err; %  vecteur de .......correspond à .......dans l'alg.
Mestime=Mpredit-gain*h'*Mpredit;%  matrice de .......correspond à .......dans l'alg.

% visualisation de ...... 
%if mod(l,3)==0, % display true and estimated trajectory every 5Ts (here 5 sec in real time)
hold on
plot(Xestime(1),Xestime(2),'*-r')
pause(0.05)
%end

en(l)=norm(trajectoire(:,l)-Xestime(1:2)); % calcul de .........
e(l)=en(l).^2;
DtoTP(l)=mean(en); % calcul de ...........
EQM(l)=mean(e); % calcul de ...............

%traceP(l)=trace(Mestime);
%tracePM(l)=mean(traceP);

Xi=Xestime;
Mi=Mestime;


end


figure(2)
plot(en);hold on;plot(EQM,'r');hold off