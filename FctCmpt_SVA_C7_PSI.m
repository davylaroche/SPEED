
function [C7dist,LASIdist,RASIdist,LPSIdist,RPSIdist,SVAdyn] = FctCmpt_SVA_C7_PSI(c3dfilename)

%% Transformation du referentiel temps :
% Dans les c3d, les événements peuvent être mis par rapport au moment où
% l'acquisition a débuté et non au moment où elle a été coupée. Alors que
% les données type point (ex position de marqueurs etc..) sont référencées
% par rapport à l'endroit où le fichier est coupé. Grace à cette partie, on
% recoordonne les deux pour pouvoir faire les calculs.

acq = btkReadAcquisition(c3dfilename);
[Events ~] = btkGetEvents(acq);
FF = btkGetFirstFrame(acq);
freq = btkGetPointFrequency(acq);
EventType = fieldnames(Events);
point = btkGetPoints(acq);


%% Récupération des moments du cycle : Choix sur cycle GAUCHE (par défaut) - recherche FS, FO, C_FO & C_FS
% Event are integrated in the number of frame of the acquisition:
for i = 1:size(EventType,1)
    Events.(EventType{i}) = round(Events.(EventType{i})*freq)-FF+1;
end

% Analyse des deux côtés
for ind2 = 1:2;
    clear cote coteAbr controLateral controLateral FS C_FSstr C_FS FOstr FO C_FO C_FOstr Heestr C_Heestr
    if ind2 == 1 ;
        
        % Analyse du côté gauche
        % Définition des coté et des coté controlatéral associées
        cote = {'Left'};
        coteAbr = {'L'};
        controLateral = {'Right'};
        controLateralAbr = {'R'};
        SVAdyn = struct(char(cote), []);
        
    elseif ind2 == 2 ;
        
        % Analyse du côté droit
        % Définition des coté et des coté controlatéral associées
        cote = {'Right'};
        coteAbr = {'R'};
        controLateral = {'Left'};
        controLateralAbr = {'L'};
        SVAdyn.(char(cote)) = [];
    end
    
    
    % Pour chaque pas on calcule :
    % Foot Strike
    FSstr = strcat(cote,'_Foot_Strike');
    % Cell2Str before we have a cell with strcat using str{} allow to get a
    % string that can be used in dynamic structure calling
    FS = Events.(FSstr{1});
    
    % Controlateral FS
    C_FSstr = strcat(controLateral,'_Foot_Strike');
    C_FS = Events.(C_FSstr{1});
    
    % Foot off
    FOstr = strcat(cote,'_Foot_Off');
    FO = Events.(FOstr{1});
    
    % Controlateral Foot strike
    C_FOstr =  strcat(controLateral,'_Foot_Off');
    C_FO = Events.(C_FOstr{1});
    
    % Point used
    Heestr =  strcat(coteAbr,'HEE');
    C_Heestr = strcat(controLateralAbr,'HEE');
    
    Hee = point.(Heestr{1});
    C_Hee = point.(C_Heestr{1});
    
    % We remove from all the events the one before the first ispsilateral
    % foot strike
    FO = FO(FO>FS(1));
    C_FS = C_FS(C_FS>FS(1));
    C_FO = C_FO(C_FO>FS(1));
    
    for c1 = 1:(size(FS(1:2),2)-1)
        clear X_cycle XHor XHornorm origineBassin_cycle C7temp_cycle LASITemp_cycle RASITemp_cycle LPSITemp_cycle
        clear C7dist LASIdist RASIdist LPSIdist RPSIdist PSI C7PSIdist
        
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
            % définition des frames du cycle
            beg_cycle = FS(c1);
            end_cycle = FS(c1+1);
            
            % Normale au plan RASI-LASI
            % pour essai MARCHE : axe médio-lat = axe y
            %                     axe antéro-post = axe x
            % donc normalisation sur l'axe X (norme 1)
            X_cycle = (point.RASI(beg_cycle:end_cycle,:) + point.LASI(beg_cycle:end_cycle,:))/2 - (point.RPSI(beg_cycle:end_cycle,:) + point.LPSI(beg_cycle:end_cycle,:))/2;
            
            XHor = X_cycle;
            XHor(:,3) = zeros(size(XHor,1),1);
            
            % Normalisation du vecteur
            XHornorm = XHor./(repmat(sqrt(dot(XHor,XHor,2)),1,3));
            
            % origine Milieu du bassin (RASI+LASI+RPSI+LPSI)/4
            origineBassin_cycle = (point.RASI(beg_cycle:end_cycle,:)+point.LASI(beg_cycle:end_cycle,:)+point.RPSI(beg_cycle:end_cycle,:)+point.LPSI(beg_cycle:end_cycle,:))./4;
            
            % Vérification & Points à projeter*
            if point.C7(beg_cycle:end_cycle,:)==0
                C7temp_cycle = nan;
            else
                C7temp_cycle = point.C7(beg_cycle:end_cycle,:) - origineBassin_cycle;
            end
            
            if point.LASI(beg_cycle:end_cycle,:)==0
                LASITemp_cycle = nan;
            else
                LASITemp_cycle = point.LASI(beg_cycle:end_cycle,:) -  origineBassin_cycle;
            end
            
            if point.RASI(beg_cycle:end_cycle,:)==0
                RASITemp_cycle = nan;
            else
                RASITemp_cycle = point.RASI(beg_cycle:end_cycle,:) -  origineBassin_cycle;
            end
            
            if point.LPSI(beg_cycle:end_cycle,:)==0
                LPSITemp_cycle = nan;
            else
                LPSITemp_cycle = point.LPSI(beg_cycle:end_cycle,:) -  origineBassin_cycle;
            end
            
            if point.RPSI(beg_cycle:end_cycle,:)==0
                RPSITemp_cycle = nan;
            else
                RPSITemp_cycle = point.RPSI(beg_cycle:end_cycle,:) -  origineBassin_cycle;
            end
            
            % Projection sur axe normé X (axe antéro-post)
            C7dist = dot(C7temp_cycle,XHornorm,2);
            LASIdist = dot(LASITemp_cycle,XHornorm,2);
            RASIdist = dot(RASITemp_cycle,XHornorm,2);
            LPSIdist = dot(LPSITemp_cycle,XHornorm,2);
            RPSIdist = dot(RPSITemp_cycle,XHornorm,2);
            
            PSI = mean([LPSIdist RPSIdist], 2);
            
            % Distance projetée entre C7 et milieu épine iliaque
            C7PSIdist = C7dist - PSI;
            
            pts2interpol = 100;
            [SVAdyn.(char(cote))(1:pts2interpol, end+1), ~] = sgnnrml(C7PSIdist, pts2interpol);
        end
    end
end
