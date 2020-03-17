function exportPSTexcel_SPEED

%% Header
% Hello Davy
% Fonction qui permet d'enregistrer les valeurs des structures de la
% fonction calculPSTSPEED pour tout regrouper dans un seul et m?me tableau


cheminsourceSPEED = '\\Nas-cic1432\pit\Backup PIT\Backup SPEED\'; % cr?ation des PATH (chemin source)
volontaires = dir(cheminsourceSPEED); % listing des dossiers ou fichiers
nomfichiersyncho = 'pst_angles.mat'; % Nom du fichier

tableau ={'Volontaire','session','C?t?','Essai','Cycle','Footstrike_a','Footsrike_r','Footstrike2_a','Foostrike2_r','strideTime','cadence','stancePhase','swingPhase','totalDoubleSupport','strideLenght','speedCoM','footstrikeangle','footoffangle','traj3d','footclearance','Footoff_a','Footoff_r','AnkleAngles_min_x','AnkleAngles_max_x','AnkleAngles_rom_x','KneeAngles_min_x','KneeAngles_max_x','KneeAngles_rom_x','HipAngles_min_x','HipAngles_max_x','HipAngles_rom_x','HipAngles_min_y','HipAngles_max_y','HipAngles_rom_y', 'PelvisAngles_min_x','PelvisAngles_max_x','PelvisAngles_rom_x','PelvisAngles_min_y','PelvisAngles_max_y','PelvisAngles_rom_y','PelvisAngles_min_z','PelvisAngles_max_z','PelvisAngles_rom_z','ShoulderAngles_min_x','ShoulderAngles_max_x','ShoulderAngles_rom_x','ShoulderAngles_min_y','ShoulderAngles_max_y','ShoulderAngles_rom_y','ShoulderAngles_min_z','ShoulderAngles_max_z','ShoulderAngles_rom_z','SpineAngles_min_x','SpineAngles_max_x','SpineAngles_rom_x','SpineAngles_min_y','SpineAngles_max_y','SpineAngles_rom_y','SpineAngles_min_z','SpineAngles_max_z','SpineAngles_rom_z','ThoraxAngles_min_x','ThoraxAngles_max_x','ThoraxAngles_rom_x','ThoraxAngles_min_y','ThoraxAngles_max_y','ThoraxAngles_rom_y','ThoraxAngles_min_z','ThoraxAngles_max_z','ThoraxAngles_rom_z','FootprogressAngles_min_z','FootprogressAngles_max_z','FootprogressAngles_rom_z'};
counter=2;
for c1 = 1:size(volontaires,1) % boucle par sujet
    if ~strcmpi(volontaires(c1).name,'.') && ~strcmpi(volontaires(c1).name,'..') &&  ~strcmpi(volontaires(c1).name,'HUJE - Copie')% test de la non ?galit? des caract?res
        
        volontaire = volontaires(c1).name; % D?finition du nom du sujet
        
        listesessions = dir(fullfile(cheminsourceSPEED,volontaire)); % r?cup?ration de la liste des sessions
        
        for c2 = 1:size(listesessions,1) % Boucle par session
            clear session  fichierpstangles listecotes nomfichier cellule cycle% mise ? z?ro des variables
            % d?finiton du compteur de ligne pour excel
            
            if strcmpi(listesessions(c2).name,'M0') || strcmpi(listesessions(c2).name,'M6')
                
                session = listesessions(c2).name; % D?finition du nom de la session
                
                
                try
                    
                    fichierpst_angles = load(fullfile(cheminsourceSPEED,volontaire,session,nomfichiersyncho)); % chargement du fichier
                    
                    pstangles = fichierpst_angles.pst_angles; % Structure avec les deux c?t?s
                    listecotes = fieldnames(pstangles); % r?cup?ration de la liste des 2 c?t?s
                    
                    for c3 = 1:size(listecotes,1) % boucle par c?t?
                        clear nomcote nomcote_t listeessais
                        
                        
                        nomcote = listecotes(c3); % D?finition du nom du c?t?
                        nomcote_t = char(nomcote); % conversion du nom en charact?re
                        
                        listeessais = fieldnames(pstangles.(nomcote_t)); % R?cup?ration de la liste des essais
                        
                        for c4 =1:size(listeessais,1) % Boucle par essai
                            clear nomessai nomessai_t listevariables  nombredelignes nombremaxdelignes counter3
                            nombredelignes = [];
                            nomessai = listeessais(c4); % D?finition du nom de l'essai
                            nomessai_t = char(nomessai); % conversion du nom en charact?re
                            
                            listevariables = fieldnames(pstangles.(nomcote_t).(nomessai_t)); % D?finiton de la liste des variables
                            
                            counter3 = 6; % D?marrage 6?me colonne
                            
                            for c5=1:size(listevariables,1) % Boucle pour chaque variable
                                clear nomvariable nomvariable_t valeurs cellule counter2 counter4
                                
                                nomvariable = listevariables(c5); % D?finition du nom de la variable
                                nomvariable_t = char(nomvariable); % Conversion du nom en charact?re
                                
                                for c8=counter3:length(tableau) % boucle pour faire correspondre la variable avec la colonne
                                    if strcmpi((tableau(1,counter3)),nomvariable_t)
                                        break
                                    elseif ~strcmpi((tableau(1,counter3)),nomvariable_t)
                                        counter3 = counter3+1;
                                    end
                                end
                                
                                valeurs = pstangles.(nomcote_t).(nomessai_t).(nomvariable_t); % R?cup?ration de la valeur dans la matrice
                                counter2 = counter;
                                
                                try % Essai de remplissage des valeurs dans le tableau en fonction du nom de la variable
                                    
                                    if strcmpi(nomvariable_t,'Footstrike_a') % Cr?ation de 2 footstrike par cycle
                                        for c9=1:(length(valeurs)-1) % Boucle pour remplir chaque valeur
                                            tableau(counter2,counter3) = {valeurs(c9)};
                                            tableau(counter2,(counter3 + 2)) = {valeurs(c9+1)};
                                            counter2 = counter2 +1;
                                        end
                                        nombredelignes(1,c5) = c9;
                                        nombrefootstrike = c9;
                                    elseif strcmpi(nomvariable_t,'Footoff_a') %Obligation de ne pas avoir plus de footoff que de footstrike
                                        for c10 =1:nombrefootstrike % Boucle pour remplir chaque valeur
                                            tableau(counter2,counter3) = {valeurs(c10)};
                                            counter2 = counter2 + 1;
                                        end
                                        
                                        nombredelignes(1,c5) = c10;
                                    else
                                        for c6 =1:length(valeurs) % Boucle pour remplir chaque valeur
                                            tableau(counter2,counter3) = {valeurs(c6)};
                                            counter2 = counter2 + 1;
                                        end
                                        
                                        nombredelignes(1,c5) = c6;
                                    end
                                catch % Si le try n'est pas possible
                                    warning('valeur manquante pour l essai')
                                    nomessai_t;
                                end
                                
                                counter3 = counter3 + 1;
                            end
                            
                            %% Apr?s le remplissage des l'ensemble des valeurs remplissage des lignes avec essais, cycle, c?t?
                            nombremaxdelignes = max(nombredelignes(:));
                            
                            counter4 = counter;
                            for c7 = 1:nombremaxdelignes % boucle pour ?crire les cycles
                                tableau(counter4,1) = {volontaire};
                                tableau(counter4,2) = {session};
                                tableau(counter4,4) = {nomessai_t};
                                tableau(counter4,3) = {nomcote_t};
                                c7_t = int2str(c7);
                                cycle = ['Cycle' c7_t];
                                tableau(counter4,5) = {cycle};
                                counter4 = counter4 + 1;
                            end
                            
                            
                            if isempty(nombremaxdelignes)
                            else
                                counter = counter + nombremaxdelignes;%incr?mentation du compteur
                            end
                            
                            
                            
                            
                        end
                    end
                    
                catch
                    warning('Le fichier de la session ne peut ?tre charg? pour le volontaire');
                    volontaire
                    session
                    
                end
                
                
                
                
            end
        end
    end
end

nomfichier = [fullfile(cheminsourceSPEED,'pst_angles.xlsx')];
xlswrite(nomfichier,tableau);
end

