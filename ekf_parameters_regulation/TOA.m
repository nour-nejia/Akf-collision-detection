clear all 
close all

load Traj;
plot(trajectoire(1,:),trajectoire(2,:),'o-b')
pause(0.1)
hold on

ts=1;
sigmau=2;
sigmav = 1e-8;

A=[1 0 ts 0;0 1 0 ts;0 0 1 0;0 0 0 1];
B=[ts^2/2 0;0 ts^2/2;ts 0;0 ts];

xi=[15000;15000]; % centre du carré
plot(xi(1),xi(2),'*-r');hold on;

V=zeros(2,1);
Xi=[xi;V];
Mi=0.1*eye(4);

%% 🔥 Stations (4 stations optimales)
Mk1 = [-5000; 0];       % référence
Mk2 = [25000; 5000];
Mk3 = [20000; 25000];
Mk4 = [-3000; 20000];

plot(Mk1(1),Mk1(2),'s-k');hold on;
plot(Mk2(1),Mk2(2),'s-k');hold on;
plot(Mk3(1),Mk3(2),'s-k');hold on;
plot(Mk4(1),Mk4(2),'s-k');hold on;

c=3*10^8;
c=1/c;

for l=1:size(trajectoire,2)

%% Prédiction    
Xpredit=A*Xi;
Mpredit=A*Mi*A'+sigmau^2*B*B';

pos = Xpredit(1:2);

%% Jacobien (4 stations)
h1=c*(pos-Mk1)/(norm(pos-Mk1));
h2=c*(pos-Mk2)/(norm(pos-Mk2));
h3=c*(pos-Mk3)/(norm(pos-Mk3));
h4=c*(pos-Mk4)/(norm(pos-Mk4));

h1=[h1;0;0];
h2=[h2;0;0];
h3=[h3;0;0];
h4=[h4;0;0];

h=[h1 h2 h3 h4]; % 4x4

%% Observation prédite
Ypredit=c*[norm(pos-Mk1);
           norm(pos-Mk2);
           norm(pos-Mk3);
           norm(pos-Mk4)];

%% Covariance
var_sortie=h'*Mpredit*h+sigmav^2*eye(4);

%% Bruit
n1=sigmav*randn;
n2=sigmav*randn;
n3=sigmav*randn;
n4=sigmav*randn;

%% Mesures TOA
Yk=[c*norm(trajectoire(:,l)-Mk1)+n1;
    c*norm(trajectoire(:,l)-Mk2)+n2;
    c*norm(trajectoire(:,l)-Mk3)+n3;
    c*norm(trajectoire(:,l)-Mk4)+n4];

%% Mise à jour
gain=Mpredit*h/var_sortie;
err=Yk-Ypredit;

Xestime=Xpredit+gain*err;
Mestime=Mpredit-gain*h'*Mpredit;

%% Plot
plot(Xestime(1),Xestime(2),'*-r')
pause(0.01)

%% Erreur
en(l)=norm(trajectoire(:,l)-Xestime(1:2));
e(l)=en(l).^2;
DtoTP(l)=mean(en);
EQM(l)=mean(e);

Xi=Xestime;
Mi=Mestime;

end

figure(2)
plot(en);hold on
plot(EQM,'r');hold off
legend('Erreur','EQM')
grid on