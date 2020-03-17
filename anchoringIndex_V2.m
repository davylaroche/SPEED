% Alexandre Naaim


% Description : Permet de choisir selon quelle direction les calculs seront
% fait. Si vector = 1 ela veut dire que c'est selon le plan transversal si
% c'est 2 c'est le plan frontal sagittal et si c'est 3 c'est le plan

% Hypothèse : Ce programme suppose un axe Y selon la direction de la marche
% (sens de la piste de marche) et un axe Z vertical.

% Input -c3dname : adresse du fichier
% c3d(C:\dossier\sousdossier\...\c3dname.c3d)
%   sous format d'une chaine de caractere
% -type : le type d'indice d'ancrage qui peut être roll ou yaw
%
% Output AIHead :Anchoring Index for the Head AIShoulder : Anchoring Index
% for the shoulder
%
% Les deux indices d'anchrages sont calculer selon la formule extraites de
% Locomotor Skills and Balance Strategies in Children with Internal
% Rotations of the Lower Limbs' Mallau 2007 " Ontongenesis of the head
% stabilization in space during locomotion in childre : influence of visual
% cues" Assaiante 1993 "Differential approach to strategies of segmental
% stabilisation in postural control Isableu" 2003

function [AIHead,lagHeadShoulder,AIShoulder,lagShouldPelvis,Angle] = anchoringIndex_V2(data,frq,type,calculationType)



% Lecture du ficher C3d et extraction des données points
%h = btkReadAcquisition(c3dname);
%data = btkGetPoints(h);
%frq = btkGetPointFrequency(h);
nbrFrame = size(data.RPSI,1);


vectorVertical = (data.LSHO(1,:)+data.RSHO(1,:))/2-(data.LPSI(1,:)+data.RPSI(1,:)+data.LASI(1,:)+data.RASI(1,:))/4;
[~,vectorVerticalPos] = max(abs(vectorVertical));

vectorAnteroPosterior = data.RPSI(end,:)-data.RPSI(1,:);
[~,vectorAnteroPosteriorPos] = max(abs(vectorAnteroPosterior));
%vectorAnteroPosterior

if vectorAnteroPosteriorPos~=2
    %'blabla'
end
% Choix du type d'indice d'ancrage calculé donné par une chaine de
% caractère (ici on peut l'ecrire avec ou sans majuscule cela ne change rien)
type = lower(type);
switch type
    % Si on veut calculer l'indice d'ancrage selon le Yaw = rotation
    % autour de l'axe vertical . Il va falloir enlever la composante selon
    % Y
    case 'yaw'
        % vector is the position we want to remove the information
        vector = vectorVerticalPos;
        %vectorLateral = data.LPSI(1,:)-data.RPSI(1,:);
        %[~,vectorLateralPos] = max(abs(data.LPSI(1,:)-data.RPSI(1,:)));
        for i = 1:3
            if i == vectorAnteroPosteriorPos
                VectGlobtemp(1,i) = vectorAnteroPosterior(1,i);
            else
                VectGlobtemp(1,i) = 0;
            end
        end
        
        ref_PIG = 3;
        VectGlobtemp = normrMaison(VectGlobtemp);
        VectGlob = repmat(VectGlobtemp,[nbrFrame,1]);
        
    % Si on veut calculer l'indice d'ancrage selon le roll = rotation
    % autour de l'axe antero posterior. Il va falloir enlever la composante selon
    % Y   
    case 'roll'
        vector = vectorAnteroPosteriorPos;
        for i = 1:3
            if i == vectorVerticalPos
                VectGlobtemp(1,i) = vectorVertical(1,i);
            else
                VectGlobtemp(1,i) = 0;
            end
        end
        
        ref_PIG = 2;
        VectGlobtemp = normrMaison(VectGlobtemp);
        VectGlob = repmat(VectGlobtemp,[nbrFrame,1]);
end

% Le nullcolumn servira a déterminer avec quel axe du repere global on
% calculera l'angle en fonction de si on a roll ou yaw Pour le roll quelque
% soit la direction principal de march on calcul toujours le roll par
% rapport à l'axe vertical donc on fixe toute les valeurs comme étant l'axe
% vertical et pour celle associé au yaw on modifie par celle du vector
% antero posterior
% nonnNullColumn = [vectorVerticalPos,vectorVerticalPos,vectorVerticalPos];
% nonnNullColumn(vectorVerticalPos) = vectorAnteroPosterior;
%nonnNullColumn

% Definition des differents droite permettant d'obtenir les differents axe
% des différents segment pour le calcul des angle absolu (par rapport au
% repere global) de ces différents segments
Shoulder = normrMaison(data.LSHO-data.RSHO);
%Pelvis = normrMaison(data.LPSI-data.RPSI);
Pelvis = normrMaison(data.LASI-data.RASI);
Head = normrMaison(data.LFHD-data.RFHD);

% On enleve ici la composante selon la direction de null collumn (on enleve
% Pour chaque vecteur pour pouvoir calculer les angles
Shoulder(:,vector) = zeros(nbrFrame,1);
Pelvis(:,vector) = zeros(nbrFrame,1);
Head(:,vector) = zeros(nbrFrame,1);

% 
% % Determination du vecteur avec lequel on calculera l'angle
% %creation d'une matrice remplis de zero
% VectGlob = zeros(nbrFrame,3);
% 
% % On remplace par des 1 la collonne nullvector(vector) = creation de l'axe
% % Z si on calcul le roll ou de l'axe Y si on calcule le yaw
% 
% VectGlob(:,nonnNullColumn(vector)) = ones(nbrFrame,1);


%calcul de l'angle entre les deux vector (cos alpha = dot(u,v)/norm(u.v))
%ici les vecteurs etant deja unitaire pas besoin de divisé par la norme
AngleShouldGlob = dot(Shoulder,VectGlob,2);
AnglePelvisGlob = dot(Pelvis,VectGlob,2);
AngleHeadGlob = dot(Head,VectGlob,2);


%Angle = dot(Shoulder,Pelvis,2);
AngleShouldGlob = 90*ones(nbrFrame,1) - acos(AngleShouldGlob)*180/pi();
AnglePelvisGlob = 90*ones(nbrFrame,1) - acos(AnglePelvisGlob)*180/pi();
AngleHeadGlob = 90*ones(nbrFrame,1) - acos(AngleHeadGlob)*180/pi();

% Calcul des indices d'ancrage basé sur la formule de 'Locomotor Skills
% and Balance Strategies in Children with Internal Rotations of the Lower
% Limbs' Mallau 2007
AIShoulder = (std(AngleShouldGlob-AnglePelvisGlob)^2-std(AngleShouldGlob)^2)/(std(AngleShouldGlob-AnglePelvisGlob)^2+std(AngleShouldGlob)^2);
AIHead = (std(AngleHeadGlob-AngleShouldGlob)^2-std(AngleHeadGlob)^2)/(std(AngleHeadGlob-AngleShouldGlob)^2+std(AngleHeadGlob)^2);

% Calcul des valeurs des angles auxquels on a enlevé leur valeur moyenne
% divisé par la std pour normalisé

% AnglePelvisGlobMean = (AnglePelvisGlob-mean(AnglePelvisGlob))/std(AnglePelvisGlob);
% AngleShouldGlobMean = (AngleShouldGlob-mean(AngleShouldGlob))/std(AngleShouldGlob);
% AngleHeadGlobMean = (AngleHeadGlob-mean(AngleHeadGlob))/std(AngleHeadGlob);

% AnglePelvisGlobMean = (AnglePelvisGlob)/std(AnglePelvisGlob);
% AngleShouldGlobMean = (AngleShouldGlob)/std(AngleShouldGlob);
% AngleHeadGlobMean = (AngleHeadGlob)/std(AngleHeadGlob);

AnglePelvisGlobMean = (2*(AnglePelvisGlob-min(AnglePelvisGlob))/(max(AnglePelvisGlob)-min(AnglePelvisGlob)))-1;
AngleShouldGlobMean = (2*(AngleShouldGlob-min(AngleShouldGlob))/(max(AngleShouldGlob)-min(AngleShouldGlob)))-1;
AngleHeadGlobMean = (2*(AngleHeadGlob-min(AngleHeadGlob))/(max(AngleHeadGlob)-min(AngleHeadGlob)))-1;

anglepelvis = data.pelvis(:,ref_PIG);
anglethorax = data.thorax(:,ref_PIG);

AnglePelvisGlobMean_2 = (2*(anglepelvis-min(anglepelvis))/(max(anglepelvis)-min(anglepelvis)))-1;
AngleThoraxGlobMean_2 = (2*(anglethorax-min(anglethorax))/(max(anglethorax)-min(anglethorax)))-1;
% On utilise ces valeurs la car lorsque l'on garde les valeurs initial (aux
% alentours de 90°) lorsque les courbes sont décalé les unes par rapport à
% l'autre (cross correlation) les valeurs diminuent forcément car les
% valeurs sont enorme par rapport a juste la translation de courbe

% type
% 'Head'


lagHeadShoulderFormula = wrapToPi(lagCalculation(AngleShouldGlobMean,AngleHeadGlobMean,'formula'));
lagShouldPelvisFormula = wrapToPi(lagCalculation(AnglePelvisGlobMean,AngleShouldGlobMean,'formula'));

lagHeadShoulderCrossCorr = wrapToPi(lagCalculation(AngleShouldGlobMean,AngleHeadGlobMean,'crosscorrelation'));
lagShouldPelvisCrossCorr = wrapToPi(lagCalculation(AnglePelvisGlobMean,AngleShouldGlobMean,'crosscorrelation'));

lagHeadShoulderFourier = wrapToPi(lagCalculation(AngleShouldGlobMean,AngleHeadGlobMean,'fourier'));
lagShouldPelvisFourier = wrapToPi(lagCalculation(AnglePelvisGlobMean,AngleShouldGlobMean,'fourier'));

lagHeadShoulderCRP = wrapToPi(lagCalculation(AngleShouldGlobMean,AngleHeadGlobMean,'CRP'));
lagShouldPelvisCRP = wrapToPi(lagCalculation(AnglePelvisGlobMean,AngleShouldGlobMean,'CRP'));
lagShouldPelvisCRP_2 = wrapToPi(lagCalculation(AnglePelvisGlobMean_2,AngleThoraxGlobMean_2,'CRP'));

switch calculationType
    case 'formula'
        lagHeadShoulder = lagHeadShoulderFormula;
        lagShouldPelvis = lagShouldPelvisFormula;
    case 'crosscorrelation'
        lagHeadShoulder = lagHeadShoulderCrossCorr;
        lagShouldPelvis = lagShouldPelvisCrossCorr;
    case 'fourier'
        lagHeadShoulder = lagHeadShoulderFourier;
        lagShouldPelvis = lagShouldPelvisFourier;
    case 'CRP'
        lagHeadShoulder = lagHeadShoulderCRP;
        % lagShouldPelvis = lagShouldPelvisCRP; ancienne version
        lagShouldPelvis = lagShouldPelvisCRP_2; 
end


Angle.lag.Formula.HeadShoulder = lagHeadShoulderFormula;
Angle.lag.Formula.ShouldPelvis = lagShouldPelvisFormula;

Angle.lag.CrossCorr.HeadShoulder = lagHeadShoulderCrossCorr;
Angle.lag.CrossCorr.ShouldPelvis = lagShouldPelvisCrossCorr;

Angle.lag.Fourier.HeadShoulder = lagHeadShoulderFourier;
Angle.lag.Fourier.ShouldPelvis = lagShouldPelvisFourier;

Angle.lag.CRP.HeadShoulder = lagHeadShoulderCRP;
Angle.lag.CRP.ShouldPelvis = lagShouldPelvisCRP;






% n = nbrFrame;
% Fs=120; % sample frequency
% Fc=2; %cutoff frequency
% [B,A]=butter(4,Fc/(Fs/2));
% 
% 
% AngleShouldGlob_Filt=filtfilt(B,A,AngleShouldGlob);
% AngleShouldGlob_Filt = AngleShouldGlob_Filt/std(AngleShouldGlob_Filt);
% 
% AnglePelvisGlob_Filt=filtfilt(B,A,AnglePelvisGlob);
% AnglePelvisGlob_Filt = AnglePelvisGlob_Filt/std(AnglePelvisGlob_Filt);
% 
% AngleHeadGlob_Filt=filtfilt(B,A,AngleHeadGlob);
% AngleHeadGlob_Filt = AngleHeadGlob_Filt/std(AngleHeadGlob_Filt);
% 
% 
% 
% Angle.AngleShouldGlob = interp1((1:n)',AngleShouldGlob_Filt,(linspace(1,n,100))','spline');
% Angle.AnglePelvisGlob = interp1((1:n)',AnglePelvisGlob_Filt,(linspace(1,n,100))','spline');
% Angle.AngleHeadGlob = interp1((1:n)',AngleHeadGlob_Filt,(linspace(1,n,100))','spline');
% 
% Angle.AmplitudeShould = max(AngleShouldGlob_Filt)-min(AngleShouldGlob_Filt);
% Angle.AmplitudePelvis = max(AnglePelvisGlob_Filt)-min(AnglePelvisGlob_Filt);
% Angle.AmplitudeHead = max(AngleHeadGlob_Filt)-min(AngleHeadGlob_Filt);
% 
% vitesseTemp = (data.LPSI(end,:)+data.RPSI(end,:)+data.LASI(end,:)+data.RASI(end,:))/4-(data.LPSI(1,:)+data.RPSI(1,:)+data.LASI(1,:)+data.RASI(1,:))/4;
% % Attention on divise par 1000 pour avoir des m/s
% Angle.Vitesse = max(abs(vitesseTemp))*Fs/n/1000;

end







