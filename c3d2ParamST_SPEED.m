% FONCTION c3d2ParamST
% Fonction retournant pour 1 fichier c3d l'ensemble des valeurs des
% paramètres spatio temporel non moyenné (1 valeur pour chaque cycle) de
% chaque coté
% Entrées : - acq : fichier c3d transformé par btkReadAcquisition(nomfichierC3d);
% Sortie : paramST : structure contenant les paramètres spatio temporels à
% gauche et à droite pour chaque cycle (ex: paramST.Left.SpeedCoM = [1.1,1.2,1.15,..,1.01]

% Alexandre Naaim : 19/09/2017 (Mise au propre)

function [paramST] = c3d2ParamST_SPEED(acq) % ajout de 2 paramètres pour calculIMU
%% Transformation du referentiel temps :
% Dans les c3d les événements peuvent être mis par rapport au moment ou
% l'acquisition a débuté et non le moment ou elle a été coupé. Alors que
% les données type point (ex position de marqueurs etc..) sont référencé
% par rapport à l'endroit ou le fichier est coupé. Grace à cette partie ou
% recoordonne les deux pour pouvoir faire les calculs.
[Events ~] = btkGetEvents(acq);
FF = btkGetFirstFrame(acq);
freq = btkGetPointFrequency(acq);
EventType = fieldnames(Events);
Point = btkGetPoints(acq);
count = 0;

% Event are integrated in the number of frame of the acqusition:
for i = 1:size(EventType,1)
    Events.(EventType{i}) = round(Events.(EventType{i})*freq)-FF+1;
end

try
    %% Initialisation
    % Left
    % distance and time
    paramST.Left.stepLenght = [];
    paramST.Left.stepTime = [];
    
    paramST.Left.strideLenght = [];
    paramST.Left.strideWidth = [];
    paramST.Left.strideTime = [];
    
    paramST.Left.CoM_vert = [];
    
    paramST.Left.speedCoM = [];
    paramST.Left.cadence = [];
    paramST.Left.speedCadenceStrideLenght = [];
    
    %percentage
    paramST.Left.stancePhase = [];
    paramST.Left.swingPhase = [];
    paramST.Left.totalDoubleSupport = [];
    paramST.Left.totalSingleSupport = [];
    paramST.Left.DS1 = [];
    paramST.Left.DS2 = [];
    
    paramST.Left.SingleSupportTime = [];
    paramST.Left.DoubleSupportTime = [];
    paramST.Left.stanceTime = [];
    paramST.Left.swingTime = [];
    
    paramST.Left.traj3d = [];
    paramST.Left.Footstrike_a = [];
    paramST.Left.Footstrike_r = [];
    paramST.Left.Footoff_a = [];
    paramST.Left.Footoff_r = [];
    paramST.Left.footclearance = [];
    
    %foot angle
    paramST.Left.footstrikeangle = [];
    paramST.Left.footoffangle = [];
    
    %Calcul des angles
    
    paramST.Left.AnkleAngles_min_x = [];
    paramST.Left.AnkleAngles_max_x = [];
    paramST.Left.AnkleAngles_rom_x = [];
    paramST.Left.KneeAngles_min_x = [];
    paramST.Left.KneeAngles_max_x = [];
    paramST.Left.KneeAngles_rom_x = [];
    paramST.Left.HipAngles_min_x = [];
    paramST.Left.HipAngles_max_x = [];
    paramST.Left.HipAngles_rom_x = [];
    paramST.Left.HipAngles_min_y = [];
    paramST.Left.HipAngles_max_y = [];
    paramST.Left.HipAngles_rom_y = [];
    paramST.Left.PelvisAngles_min_x = [];
    paramST.Left.PelvisAngles_max_x = [];
    paramST.Left.PelvisAngles_rom_x = [];
    paramST.Left.PelvisAngles_min_y = [];
    paramST.Left.PelvisAngles_max_y = [];
    paramST.Left.PelvisAngles_rom_y = [];
    paramST.Left.PelvisAngles_min_z = [];
    paramST.Left.PelvisAngles_max_z = [];
    paramST.Left.PelvisAngles_rom_z = [];
    paramST.Left.ShoulderAngles_min_x = [];
    paramST.Left.ShoulderAngles_max_x = [];
    paramST.Left.ShoulderAngles_rom_x = [];
    paramST.Left.ShoulderAngles_min_y = [];
    paramST.Left.ShoulderAngles_max_y = [];
    paramST.Left.ShoulderAngles_rom_y = [];
    paramST.Left.ShoulderAngles_min_z = [];
    paramST.Left.ShoulderAngles_max_z = [];
    paramST.Left.ShoulderAngles_rom_z = [];
    paramST.Left.SpineAngles_min_x = [];
    paramST.Left.SpineAngles_max_x = [];
    paramST.Left.SpineAngles_rom_x = [];
    paramST.Left.SpineAngles_min_y = [];
    paramST.Left.SpineAngles_max_y = [];
    paramST.Left.SpineAngles_rom_y = [];
    paramST.Left.SpineAngles_min_z = [];
    paramST.Left.SpineAngles_max_z = [];
    paramST.Left.SpineAngles_rom_z = [];
    paramST.Left.ThoraxAngles_min_x = [];
    paramST.Left.ThoraxAngles_max_x = [];
    paramST.Left.ThoraxAngles_rom_x = [];
    paramST.Left.ThoraxAngles_min_y = [];
    paramST.Left.ThoraxAngles_max_y = [];
    paramST.Left.ThoraxAngles_rom_y = [];
    paramST.Left.ThoraxAngles_min_z = [];
    paramST.Left.ThoraxAngles_max_z = [];
    paramST.Left.ThoraxAngles_rom_z = [];
    paramST.Left.FootprogressAngles_min_z = [];
    paramST.Left.FootprogressAngles_max_z = [];
    paramST.Left.FootprogressAngles_rom_z = [];
    
    % Similar parameter on the left and the right
    paramST.Right = paramST.Left;
    
    
    %% Analyse de chaque coté
    % Définition des coté et des coté controlatéral associées
    cote = {'Right','Left'};
    coteAbr = {'R','L'};
    controLateral = {'Left','Right'};
    controLateralAbr = {'L','R'};
    
    % Pour chaque coté :
    for i = 1:2
        
        % Pour chaque pas on calcul
        
        % Foot Strike
        FSstr = strcat(cote(i),'_Foot_Strike');
        % Cell2Str before we have a cell with strcat using str{} allow to get a
        % string that can be used in dynamic structure calling
        FS = Events.(FSstr{1});
        
        
        % Controlateral FS
        C_FSstr = strcat(controLateral(i),'_Foot_Strike');
        C_FS = Events.(C_FSstr{1});
        
        
        % Foot off
        FOstr = strcat(cote(i),'_Foot_Off');
        FO = Events.(FOstr{1});
        
        
        % Controlateral Foot strike
        C_FOstr =  strcat(controLateral(i),'_Foot_Off');
        C_FO = Events.(C_FOstr{1});
        
        
        % Point used
        Heestr =  strcat(coteAbr(i),'HEE');
        C_Heestr = strcat(controLateralAbr(i),'HEE');
        
        Hee = Point.(Heestr{1});
        C_Hee = Point.(C_Heestr{1});
        % COM barycent of LASI RASI LPSI RPSI
        CoM = Point.LASI+Point.RASI+Point.LPSI+Point.RPSI;
        CoM = CoM/4;
        
        % Calcul de la direction dans laquelle le patient ce déplace
        [~,indDeplMax] = max(abs(CoM(end,:)-CoM(1,:)));
        
        % We remove from all the events the one before the first ispsolateral
        % foot strike
        FO = FO(FO>FS(1));
        C_FS = C_FS(C_FS>FS(1));
        C_FO = C_FO(C_FO>FS(1));
        
        % calcul des temps de FS et FO en secondes
        FSa = FS/freq;
        % FSr = FSa - début pic de syncho
        C_FSa = C_FS/freq;
        % C_FSr = C_FSa - début pic de syncho
        FOa = FO/freq;
        % FOr = FOa - début pic de syncho
        C_FOa = C_FO/freq;
        % C_FOr = C_FOa - début pic de syncho
        
        for c1 = 1:(size(FS,2)-1)
            % We need to test that between 2 foot strike there is a foot off and
            % a controlateral foot strike and off
            testFO = isempty(FO(FO>FS(c1) & FO<FS(c1+1)));
            testC_FO = isempty(C_FO(C_FO>FS(c1) & C_FO<FS(c1+1)));
            testC_FS = isempty(C_FS(C_FS>FS(c1) & C_FS<FS(c1+1)));
            % If one of this condition is not respected we add at the beginning
            % of the Event vector a 0 value in order to keep the correspondence
            % between the foot strike event and
            if testFO
                FO = [0,FO];
            end
            if testC_FO
                C_FO = [0,C_FO];
            end
            if testC_FS
                C_FS = [0,C_FS];
            end
            % If no test is empty (which mean that there is the event between
            % the 2 foot strike) it is possible to compute all the different
            % spation temporal parameter.
            
            if not(testFO | testC_FO | testC_FS)
                %% Step lenght and time
                %   Lenght m
                Steptemp = Hee(FS(c1+1),:) - C_Hee(C_FS(c1),:);
                StepLenght = abs(Steptemp(indDeplMax)/1000);
                
                %   Time = swing phase?(we remove one frame because we consider that
                %   the cycle is finished before the foot strike
                StepTime = (FS(c1+1)-C_FS(c1))/freq;
                
                paramST.(cote{i}).stepLenght = [paramST.(cote{i}).stepLenght,StepLenght];
                paramST.(cote{i}).stepTime = [paramST.(cote{i}).stepTime,StepTime];
                
                
                %% Stride lenght and time
                %   Lenght
                stridetemp = Hee(FS(c1+1),:) - Hee(FS(c1),:);
                strideLenght = abs(stridetemp(indDeplMax)/1000);
                
                % Distance between a droite and a point
                % https://fr.wikipedia.org/wiki/Distance_d%27un_point_%C3%A0_une_droite
                u = Hee(FS(c1+1),:) - Hee(FS(c1),:);
                BA = C_Hee(C_FS(c1),:) - Hee(FS(c1),:);
                strideWidth = norm(cross(BA,u))/norm(u)/1000;
                
                % Time
                strideTime = (FS(c1+1)-FS(c1))/freq;
                
                paramST.(cote{i}).strideLenght = [paramST.(cote{i}).strideLenght,strideLenght];
                paramST.(cote{i}).strideWidth = [paramST.(cote{i}).strideWidth,strideWidth];
                paramST.(cote{i}).strideTime = [paramST.(cote{i}).strideTime,strideTime];
                
                %% Cadence (step/min)
                cadence = 2*60/strideTime;
                
                % Vitesse : 2 méthodes
                
                % Méthode : cadence*Stride_lenght/120
                speed = cadence*strideLenght/120;
                
                % Speed CoM : The horizontal difference between the middle of the pelvis
                % markers at the beginning and the end of the acqusition divided by the
                % time of the acqusition:
                CoMtemp = CoM(FS(c1+1),:) - CoM(FS(c1),:);
                CoMtemp = abs(CoMtemp(indDeplMax)/1000);
                speedCoM = CoMtemp/strideTime;
                
                paramST.(cote{i}).speedCoM = [paramST.(cote{i}).speedCoM,speedCoM];
                paramST.(cote{i}).cadence = [paramST.(cote{i}).cadence,cadence];
                paramST.(cote{i}).speedCadenceStrideLenght = [paramST.(cote{i}).speedCadenceStrideLenght,speed];
                
                
                %% ajout MG pour LOCOX Frontiers
                CoM_cible = CoM(FS(c1):FS(c1+1),:);
                
                % Amplitude max mouvements verticaux du CoM pdt cycle de marche
                CoM_vert = max(CoM_cible(:,3))-abs(min(CoM_cible(:,3)));
                paramST.(cote{i}).CoM_vert = [paramST.(cote{i}).CoM_vert,CoM_vert];
                
                
                %% Paramètre de cycle
                strideTot = FS(c1+1)-FS(c1);
                
                % End and Begin of the first and second double support
                firstDoubleSupport = 100*(C_FO(c1)-FS(c1))/strideTot;
                secondeDoubleSupport = 100*(C_FS(c1)-FS(c1))/strideTot;
                
                % Pourcentage/time Stance et Swing Phase
                stanceFrame = FO(c1)-FS(c1);
                swingFrame = FS(c1+1)-FO(c1);
                
                stancePerc = 100*stanceFrame/strideTot;
                stanceTime = stanceFrame/freq;
                swingPerc = 100*swingFrame/strideTot;
                swingTime = swingFrame/freq;
                
                % Percentage end 1st and 2nd double Support :
                firstDoubleSupportNbrFrame = C_FO(c1)-FS(c1);
                secondeDoubleSupportNbrFrame = FO(c1)-C_FS(c1);
                
                firstDoubleSupportPerc = 100*firstDoubleSupportNbrFrame/strideTot;
                secondeDoubleSupportPerc = 100*secondeDoubleSupportNbrFrame/strideTot;
                
                doubleSupportPerc = firstDoubleSupportPerc+secondeDoubleSupportPerc;
                doubleSupportTime = (firstDoubleSupportNbrFrame+secondeDoubleSupportNbrFrame)/freq;
                
                % Total single and total double support :
                singleSupportNbrFrame = C_FS(c1)-C_FO(c1);
                singleSupportPerc = 100*singleSupportNbrFrame/strideTot;
                singleSupportTime = singleSupportNbrFrame/freq;
                
                %singleSupport = stancePerc-doubleSupport;
                
                paramST.(cote{i}).stancePhase = [paramST.(cote{i}).stancePhase,stancePerc];
                paramST.(cote{i}).swingPhase = [paramST.(cote{i}).swingPhase,swingPerc];
                
                paramST.(cote{i}).DS1 = [paramST.(cote{i}).DS1,firstDoubleSupport];
                paramST.(cote{i}).DS2 = [paramST.(cote{i}).DS2,secondeDoubleSupport];
                
                paramST.(cote{i}).totalDoubleSupport = [paramST.(cote{i}).totalDoubleSupport,doubleSupportPerc];
                paramST.(cote{i}).totalSingleSupport = [paramST.(cote{i}).totalSingleSupport,singleSupportPerc];
                
                paramST.(cote{i}).SingleSupportTime = [paramST.(cote{i}).SingleSupportTime,singleSupportTime];
                paramST.(cote{i}).DoubleSupportTime = [paramST.(cote{i}).DoubleSupportTime,doubleSupportTime];
                paramST.(cote{i}).stanceTime = [paramST.(cote{i}).stanceTime,stanceTime];
                paramST.(cote{i}).swingTime = [paramST.(cote{i}).swingTime,swingTime];
                
                %% ajout des Footstrike et footoff en s, valeur absolue et
                % relative au pic de synchoIMU
                paramST.(cote{i}).Footstrike_a = [FSa];
                %paramST.(cote{i}).Footstrike_r = [FSr];
                paramST.(cote{i}).Footoff_a = [FOa];
                %paramST.(cote{i}).Footoff_r = [FOr];
                
                %% calcul des angles footstrike et foot off
                
                try
                    
                    footo = Point.([coteAbr{i},'FOOTO'])(FS(c1):FS(c1+1),:); % : correspond à toutes les valeurs de la matrice dans la dimension colonne
                    foota = Point.([coteAbr{i},'FOOTA'])(FS(c1):FS(c1+1),:);
                    footl = Point.([coteAbr{i},'FOOTL'])(FS(c1):FS(c1+1),:);
                    footp = Point.([coteAbr{i},'FOOTP'])(FS(c1):FS(c1+1),:);
                    
                    
                    for count1 = 1:size(footo,1) % création de vecteur pour créer une référence
                        vectx = foota(count1,:)-footo(count1,:);
                        vecty = footl(count1,:)-footo(count1,:);
                        vectz = footp(count1,:)-footo(count1,:);
                        normevectx = vectx./norm(vectx); %./ permet de diviser chaque terme de la matrice
                        normevecty = vecty./norm(vecty);
                        normevectz = vectz./norm(vectz);
                        
                        referencefoot(count1,:) = [normevectx,normevecty,normevectz]; % reference du pied
                        referencespace(count1,:) = [1,0,0,0,1,0,0,0,1]; % référence du labo
                        
                        
                    end
                    
                    
                    [angles, T] = angles_solver(referencespace, referencefoot, 'YXZ'); % fonction de résolution des rotations matricielles
                    a = mean(angles(:,2)); % moyenne de la courbe angles
                    angles(:,2) = - angles(:,2) + a; % mise en place de la courbe les bonnes valeurs
                    
                    anglefootstrike = angles(1,2); % angle du foot strike = 1er angle de la dimension (qui commence au FS)
                    
                    FOtimeabsolu = FO(1,c1); % recherche de la frame avec le FO
                    FOtimerelative = FOtimeabsolu - FS(c1); % soustraction du FS précédent
                    
                    anglefootoff = angles(FOtimerelative,2); % angle du footoff au moment du footoff
                    
                    paramST.(cote{i}).footstrikeangle = [paramST.(cote{i}).footstrikeangle,anglefootstrike]; % enregistrement des valeurs
                    paramST.(cote{i}).footoffangle = [paramST.(cote{i}).footoffangle,anglefootoff]; % enregistrement des valeurs
                end
                %% calcul trajectoire
                try
                    
                    positionMAnckle = Point.([coteAbr{i},'ANK'])(FS(c1):FS(c1+1),:);
                    
                    
                    traj3d = 0;
                    
                    for count3 = 1:(size(positionMAnckle,1)-1)
                        traj = positionMAnckle(count3+1,:) - positionMAnckle(count3,:);
                        trajnorm = norm(traj);
                        traj3d = traj3d + trajnorm;
                    end
                    
                    paramST.(cote{i}).traj3d = [paramST.(cote{i}).traj3d, traj3d];
                    
                end
                
                %% calcul du foot clearance
                
                try
                    pzFMH = Point.([coteAbr{i},'FMH'])(FO(c1):FS(c1+1),3); % position du marqueur FMH sur z, entre le Foot off et le Foot Strike suivant
                    p2zFMH = Point.([coteAbr{i},'FMH'])(FS(c1):FO(c1),3); % position du marqueur FMH sur z, entre le Foot strike et le Foot off
                    
                    
                    [ind,peaks] = findpeaks(pzFMH); % fonction permettant de récupérer le pic
                    
                    ind = [12,15];
                    
                    if length(ind) == 1
                        footclearance = nanmin(pzFMH(ind(1):end)) - nanmin(p2zFMH); % récupération du foot clearance ( hauteur minimum après le foot off - hauteur minumum pendant le cycle)
                    else
                        footclearance = nanmin(pzFMH(ind(length(ind)):end)) - nanmin(p2zFMH);
                    end
                    
                    
                    paramST.(cote{i}).footclearance = [paramST.(cote{i}).footclearance, footclearance]; % enregistrement du foot clearance dans les paramST
                end
                %% Calcul des différentes angulations à chaque cycle
                try
                    
                    angles_ankle = Point.([coteAbr{i},'AnkleAngles'])(FS(c1):FS(c1+1),:);
                    angle_min_ankle = min(angles_ankle(:,1));
                    angle_max_ankle = max(angles_ankle(:,1));
                    angle_rom_ankle = angle_max_ankle-angle_min_ankle;
                    
                    angles_knee = Point.([coteAbr{i},'KneeAngles'])(FS(c1):FS(c1+1),:);
                    angle_min_knee = min(angles_knee(:,1));
                    angle_max_knee = max(angles_knee(:,1));
                    angle_rom_knee = angle_max_knee - angle_min_knee;
                    
                    angles_hip = Point.([coteAbr{i},'HipAngles'])(FS(c1):FS(c1+1),:);
                    angle_min_hip_x = min(angles_hip(:,1));
                    angle_max_hip_x = max(angles_hip(:,1));
                    angle_rom_hip_x = angle_max_hip_x - angle_min_hip_x;
                    angle_min_hip_y = min(angles_hip(:,2));
                    angle_max_hip_y = max(angles_hip(:,2));
                    angle_rom_hip_y = angle_max_hip_y - angle_min_hip_y;
                    
                    angles_pelvis = Point.([coteAbr{i},'PelvisAngles'])(FS(c1):FS(c1+1),:);
                    angle_min_pelvis_x = min(angles_pelvis(:,1));
                    angle_max_pelvis_x = max(angles_pelvis(:,1));
                    angle_rom_pelvis_x = angle_max_pelvis_x - angle_min_pelvis_x;
                    angle_min_pelvis_y = min(angles_pelvis(:,2));
                    angle_max_pelvis_y = max(angles_pelvis(:,2));
                    angle_rom_pelvis_y = angle_max_pelvis_y - angle_min_pelvis_y;
                    angle_min_pelvis_z = min(angles_pelvis(:,3));
                    angle_max_pelvis_z = max(angles_pelvis(:,3));
                    angle_rom_pelvis_z = angle_max_pelvis_z - angle_min_pelvis_z;
                    
                    angles_shoulder = Point.([coteAbr{i},'ShoulderAngles'])(FS(c1):FS(c1+1),:);
                    angle_min_shoulder_x = min(angles_shoulder(:,1));
                    angle_max_shoulder_x = max(angles_shoulder(:,1));
                    angle_rom_shoulder_x = angle_max_shoulder_x - angle_min_shoulder_x;
                    angle_min_shoulder_y = min(angles_shoulder(:,2));
                    angle_max_shoulder_y = max(angles_shoulder(:,2));
                    angle_rom_shoulder_y = angle_max_shoulder_y - angle_min_shoulder_y;
                    angle_min_shoulder_z = min(angles_shoulder(:,3));
                    angle_max_shoulder_z = max(angles_shoulder(:,3));
                    angle_rom_shoulder_z = angle_max_shoulder_z - angle_min_shoulder_z;
                    
                    angles_spine = Point.([coteAbr{i},'SpineAngles'])(FS(c1):FS(c1+1),:);
                    angle_min_spine_x = min(angles_spine(:,1));
                    angle_max_spine_x = max(angles_spine(:,1));
                    angle_rom_spine_x = angle_max_spine_x - angle_min_spine_x;
                    angle_min_spine_y = min(angles_spine(:,2));
                    angle_max_spine_y = max(angles_spine(:,2));
                    angle_rom_spine_y = angle_max_spine_y - angle_min_spine_y;
                    angle_min_spine_z = min(angles_spine(:,3));
                    angle_max_spine_z = max(angles_spine(:,3));
                    angle_rom_spine_z = angle_max_spine_z - angle_min_spine_z;
                    
                    angles_thorax = Point.([coteAbr{i},'ThoraxAngles'])(FS(c1):FS(c1+1),:);
                    angle_min_thorax_x = min(angles_thorax(:,1));
                    angle_max_thorax_x = max(angles_thorax(:,1));
                    angle_rom_thorax_x = angle_max_thorax_x - angle_min_thorax_x;
                    angle_min_thorax_y = min(angles_thorax(:,2));
                    angle_max_thorax_y = max(angles_thorax(:,2));
                    angle_rom_thorax_y = angle_max_thorax_y - angle_min_thorax_y;
                    angle_min_thorax_z = min(angles_thorax(:,3));
                    angle_max_thorax_z = max(angles_thorax(:,3));
                    angle_rom_thorax_z = angle_max_thorax_z - angle_min_thorax_z;
                    
                    angles_footprogress = Point.([coteAbr{i},'FootProgressAngles'])(FS(c1):FS(c1+1),:);
                    angle_min_footprogress_z = min(angles_footprogress(:,3));
                    angle_max_footprogress_z = max(angles_footprogress(:,3));
                    angle_rom_footprogress_z = angle_max_footprogress_z - angle_min_footprogress_z;
                    
                    paramST.(cote{i}).AnkleAngles_min_x = [paramST.(cote{i}).AnkleAngles_min_x, angle_min_ankle];
                    paramST.(cote{i}).AnkleAngles_max_x = [paramST.(cote{i}).AnkleAngles_max_x, angle_max_ankle];
                    paramST.(cote{i}).AnkleAngles_rom_x = [paramST.(cote{i}).AnkleAngles_rom_x, angle_rom_ankle];
                    paramST.(cote{i}).KneeAngles_min_x = [paramST.(cote{i}).KneeAngles_min_x, angle_min_knee];
                    paramST.(cote{i}).KneeAngles_max_x = [paramST.(cote{i}).KneeAngles_max_x, angle_max_knee];
                    paramST.(cote{i}).KneeAngles_rom_x = [paramST.(cote{i}).KneeAngles_rom_x, angle_rom_knee];
                    paramST.(cote{i}).HipAngles_min_x = [paramST.(cote{i}).HipAngles_min_x, angle_min_hip_x];
                    paramST.(cote{i}).HipAngles_max_x = [paramST.(cote{i}).HipAngles_max_x, angle_max_hip_x];
                    paramST.(cote{i}).HipAngles_rom_x = [paramST.(cote{i}).HipAngles_rom_x, angle_rom_hip_x];
                    paramST.(cote{i}).HipAngles_min_y = [paramST.(cote{i}).HipAngles_min_y, angle_min_hip_y];
                    paramST.(cote{i}).HipAngles_max_y = [paramST.(cote{i}).HipAngles_max_y, angle_max_hip_y];
                    paramST.(cote{i}).HipAngles_rom_y = [paramST.(cote{i}).HipAngles_rom_y, angle_rom_hip_y];
                    paramST.(cote{i}).PelvisAngles_min_x = [paramST.(cote{i}).PelvisAngles_min_x, angle_min_pelvis_x];
                    paramST.(cote{i}).PelvisAngles_max_x = [paramST.(cote{i}).PelvisAngles_max_x, angle_max_pelvis_x];
                    paramST.(cote{i}).PelvisAngles_rom_x = [paramST.(cote{i}).PelvisAngles_rom_x, angle_rom_pelvis_x];
                    paramST.(cote{i}).PelvisAngles_min_y = [paramST.(cote{i}).PelvisAngles_min_y, angle_min_pelvis_y];
                    paramST.(cote{i}).PelvisAngles_max_y = [paramST.(cote{i}).PelvisAngles_max_y, angle_max_pelvis_y];
                    paramST.(cote{i}).PelvisAngles_rom_y = [paramST.(cote{i}).PelvisAngles_rom_y, angle_rom_pelvis_y];
                    paramST.(cote{i}).PelvisAngles_min_z = [paramST.(cote{i}).PelvisAngles_min_z, angle_min_pelvis_z];
                    paramST.(cote{i}).PelvisAngles_max_z = [paramST.(cote{i}).PelvisAngles_max_z, angle_max_pelvis_z];
                    paramST.(cote{i}).PelvisAngles_rom_z = [paramST.(cote{i}).PelvisAngles_rom_z, angle_rom_pelvis_z];
                    paramST.(cote{i}).ShoulderAngles_min_x = [paramST.(cote{i}).ShoulderAngles_min_x, angle_min_shoulder_x];
                    paramST.(cote{i}).ShoulderAngles_max_x = [paramST.(cote{i}).ShoulderAngles_max_x, angle_max_shoulder_x];
                    paramST.(cote{i}).ShoulderAngles_rom_x = [paramST.(cote{i}).ShoulderAngles_rom_x, angle_rom_shoulder_x];
                    paramST.(cote{i}).ShoulderAngles_min_y = [paramST.(cote{i}).ShoulderAngles_min_y, angle_min_shoulder_y];
                    paramST.(cote{i}).ShoulderAngles_max_y = [paramST.(cote{i}).ShoulderAngles_max_y, angle_max_shoulder_y];
                    paramST.(cote{i}).ShoulderAngles_rom_y = [paramST.(cote{i}).ShoulderAngles_rom_y, angle_rom_shoulder_y];
                    paramST.(cote{i}).ShoulderAngles_min_z = [paramST.(cote{i}).ShoulderAngles_min_z, angle_min_shoulder_z];
                    paramST.(cote{i}).ShoulderAngles_max_z = [paramST.(cote{i}).ShoulderAngles_max_z, angle_max_shoulder_z];
                    paramST.(cote{i}).ShoulderAngles_rom_z = [paramST.(cote{i}).ShoulderAngles_rom_z, angle_rom_shoulder_z];
                    paramST.(cote{i}).SpineAngles_min_x = [paramST.(cote{i}).SpineAngles_min_x, angle_min_spine_x];
                    paramST.(cote{i}).SpineAngles_max_x = [paramST.(cote{i}).SpineAngles_max_x, angle_max_spine_x];
                    paramST.(cote{i}).SpineAngles_rom_x = [paramST.(cote{i}).SpineAngles_rom_x, angle_rom_spine_x];
                    paramST.(cote{i}).SpineAngles_min_y = [paramST.(cote{i}).SpineAngles_min_y, angle_min_spine_y];
                    paramST.(cote{i}).SpineAngles_max_y = [paramST.(cote{i}).SpineAngles_max_y, angle_max_spine_y];
                    paramST.(cote{i}).SpineAngles_rom_y = [paramST.(cote{i}).SpineAngles_rom_y, angle_rom_spine_y];
                    paramST.(cote{i}).SpineAngles_min_z = [paramST.(cote{i}).SpineAngles_min_z, angle_min_spine_z];
                    paramST.(cote{i}).SpineAngles_max_z = [paramST.(cote{i}).SpineAngles_max_z, angle_max_spine_z];
                    paramST.(cote{i}).SpineAngles_rom_z = [paramST.(cote{i}).SpineAngles_rom_z, angle_rom_spine_z];
                    paramST.(cote{i}).ThoraxAngles_min_x = [paramST.(cote{i}).ThoraxAngles_min_x, angle_min_thorax_x];
                    paramST.(cote{i}).ThoraxAngles_max_x = [paramST.(cote{i}).ThoraxAngles_max_x, angle_max_thorax_x];
                    paramST.(cote{i}).ThoraxAngles_rom_x = [paramST.(cote{i}).ThoraxAngles_rom_x, angle_rom_thorax_x];
                    paramST.(cote{i}).ThoraxAngles_min_y = [paramST.(cote{i}).ThoraxAngles_min_y, angle_min_thorax_y];
                    paramST.(cote{i}).ThoraxAngles_max_y = [paramST.(cote{i}).ThoraxAngles_max_y, angle_max_thorax_y];
                    paramST.(cote{i}).ThoraxAngles_rom_y = [paramST.(cote{i}).ThoraxAngles_rom_y, angle_rom_thorax_y];
                    paramST.(cote{i}).ThoraxAngles_min_z = [paramST.(cote{i}).ThoraxAngles_min_z, angle_min_thorax_z];
                    paramST.(cote{i}).ThoraxAngles_max_z = [paramST.(cote{i}).ThoraxAngles_max_z, angle_max_thorax_z];
                    paramST.(cote{i}).ThoraxAngles_rom_z = [paramST.(cote{i}).ThoraxAngles_rom_z, angle_rom_thorax_z];
                    paramST.(cote{i}).FootprogressAngles_min_z = [paramST.(cote{i}).FootprogressAngles_min_z, angle_min_footprogress_z];
                    paramST.(cote{i}).FootprogressAngles_max_z = [paramST.(cote{i}).FootprogressAngles_max_z, angle_max_footprogress_z];
                    paramST.(cote{i}).FootprogressAngles_rom_z = [paramST.(cote{i}).FootprogressAngles_rom_z, angle_rom_footprogress_z];
                catch
                    warning('problème calcul angle')
                end
            end
        end
    end
catch
    count = count+1;
end

end