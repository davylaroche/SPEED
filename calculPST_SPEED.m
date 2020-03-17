function calculPST_SPEED
%% header
% Fonction qui permet d'enregistrer l'ensemble des PST pour l'intégralité
% des volontaires SPEED

cheminsourceVicon = '\\Nas-cic1432\pit\Backup PIT\Backup Vicon\Acquisition\SPEED\SPEED\';
cheminsourceSPEED = '\\Nas-cic1432\pit\Backup PIT\\Backup SPEED\';
volontaires = dir(cheminsourceVicon);

for c1=1:size(volontaires) % Boucle par volontaire
    clear volontaire sessions
    if ~strcmpi(volontaires(c1).name,'.') && ~strcmpi(volontaires(c1).name,'..') && ~strcmpi(volontaires(c1).name, 'MOMA19320629 - Copie') % on exclu les faux dossiers et la copie
        volontaire = volontaires(c1).name;
        sessions = dir(fullfile(cheminsourceVicon,volontaire));
        
        for c2=1:size(sessions) % boucle par session
            clear counter1 session enregistrements calcul pst_angles volontaire_t calcul essais
            if strcmpi(sessions(c2).name,'M0') || strcmpi(sessions(c2).name,'M6')
                
                session = sessions(c2).name;
                
                enregistrements = dir(fullfile(cheminsourceVicon,volontaire,session,'*.c3d'));
                
                counter1 = 1;
                for c3=1:size(enregistrements) % Boucle pour retrouver tous les essais
                    clear essai_c3d
                    if strcmpi(enregistrements(c3).name(1:6),'marche')
                        essai_c3d = enregistrements(c3).name;                       
                        essais(counter1,1) = {essai_c3d};
                        counter1= counter1 + 1;
                        
                    end
                end
                
                calcul = fctcalcul_SPEED(cheminsourceVicon, essais, session, volontaire);
                
                pst_angles =[];
                pst_angles=[pst_angles,calcul];
                
                volontaire_t = volontaire(1:4);
                save(fullfile(cheminsourceSPEED,volontaire_t,session,'pst_angles'), 'pst_angles');
                volontaire
                session
            end
        end
        
    end
    
end


end

