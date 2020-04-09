function fct_VO2

path = '\\\Nas-cic1432\pit\Backup PIT\Backup SPEED';
subject = dir(path);
subfolders = {'M0', 'M6'};
export = {'ID', 'Eval', 'Filename', 'VO2_integral', 'O2 pulse_mean', 'O2 pulse_SD', 'O2 pulse_min', 'O2 pulse_max'...
    'O2 pulse_mean_30', 'O2 pulse_SD_30', 'O2 pulse_min_30', 'O2 pulse_max_30'};

for c1 = 1:length(subject)
    if ~strcmpi(subject(c1).name, '.') && ~strcmpi(subject(c1).name, '..')
        subject(c1).name
        
        for c2 = 1:2
            clear subfolder
            subfolder = fullfile(path, subject(c1).name, subfolders{c2});
            subfolders{c2}
            
            files = dir(fullfile(subfolder, '*.xls*'));
            
            for c3 = 1:length(files)
               clear raw   
               [~, ~, raw] = xlsread(fullfile(subfolder, files(c3).name));
                 
               try
                % Récupération des numéros et contenus (sans entêtes) des colonnes t,
                % VO2, VCO2, FC et Marqueur
                verif_var = {'t', 'FrequResp', 'VO2', 'FC', 'Marqueur'};
                verif_var(2, :) = repmat({0}, length(verif_var), 1);
                for j=1:size(raw, 2)
                    
                    if strcmpi(raw{1,j},'t')
                        time=j;
                        verif_var(2,1) = {1};
                    elseif strcmpi(raw{1,j},'F.R')
                        VO2=j;
                        verif_var(2,2) = {1};
                    elseif strcmpi(raw{1,j},'VO2')
                        VCO2=j;
                        verif_var(2,3) = {1};
                    elseif strcmpi(raw{1,j},'VO2/FC')
                        FC=j;
                        verif_var(2,4) = {1};
                    elseif strcmpi(raw{1,j},'Marqueur')
                        marqueur=j;
                        verif_var(2,5) = {1};
                    end
                    
                end

                %%%% Verification de la création des variables
                if sum(cell2mat(verif_var(2,:))) ~= size(verif_var, 2);
                    temp = cell2mat(verif_var(2,:));
                    if temp(1) == 1, tim=cell2mat(raw(4:end,time)); else tim = NaN; error 'timeline not present, please correct', end
                    if temp(2) == 1, FR=cell2mat(raw(4:end,VO2)); else vo = NaN; error 'No VO2, baseline computation impossible', end
                    if temp(3) == 1, vo=cell2mat(raw(4:end,VCO2)); else vco = NaN; end
                    if temp(4) == 1, freqcar=cell2mat(raw(4:end,FC)); else freqcar = NaN; end
                    if temp(5) == 1, mar=raw(4:end,marqueur); size_mar = length(mar);else mar = NaN; error 'no marker defined, please correct', end
                else
                    tim=cell2mat(raw(4:end,time));
                    FR=cell2mat(raw(4:end,VO2));
                    vo=cell2mat(raw(4:end,VCO2));
                    freqcar=cell2mat(raw(4:end,FC));
                    mar=raw(4:end,marqueur); size_mar = length(mar);
                end

                % Création d'un vecteur avec les numéros de ligne où on trouve 'mark'
                mar(cellfun(@(mar) any(isnan(mar)),mar)) = [];
                if length(mar)~=size_mar
                    mar=raw(4:end,marqueur);
                    mar(cellfun(@(mar) any(isnan(mar)),mar)) = {''};
                end

                mark=find(ismember(lower(mar),{'exercice'})==1 | ismember(lower(mar),{'récupérati'})==1 | ismember(lower(mar),{'mark'})==1 | ismember(lower(mar),{'marker'})==1);

                % Conversion des temps HH:MM:SS de la colonne t en secondes
                [~, ~, ~, H, MN, S] = datevec(tim);
                timeline =  H*3600+MN*60+S;
                
                % Calcul de la consommation d'O2 instantannée et totale
                if mark(1)>10
                    rest = nanmean(vo(mark(1)-10:mark(1)-1)./FR(mark(1)-10:mark(1)-1));
                else
                    rest = nanmean(vo(2:mark(1)-1)./FR(vo(2:mark(1)-1)));
                end
                VO2intake = (vo./FR)-rest;
                VO2intake_int = sum(VO2intake(mark(1):mark(2)));

                % Moyenne, SD, max, min du pouls d'O2
                O2pulse_mean = nanmean(freqcar(mark(1):mark(2)));
                O2pulse_sd = nanstd(freqcar(mark(1):mark(2)));
                O2pulse_min = nanmin(freqcar(mark(1):mark(2)));
                O2pulse_max = nanmax(freqcar(mark(1):mark(2)));
                
                %%% last 30 seconds
                steady = find(timeline>timeline(mark(2))-30, 1, 'first');
                O2pulse_mean_30 = nanmean(freqcar(steady:mark(2)));
                O2pulse_sd_30 = nanstd(freqcar(steady:mark(2)));
                O2pulse_min_30 = nanmin(freqcar(steady:mark(2)));
                O2pulse_max_30 = nanmax(freqcar(steady:mark(2)));  
                
                export(end+1, :) = [{subject(c1).name}, subfolders(c2), {files(c3).name}, num2cell([VO2intake_int...
                    O2pulse_mean O2pulse_sd O2pulse_min O2pulse_max O2pulse_mean_30 O2pulse_sd_30 O2pulse_min_30 O2pulse_max_30])];
               catch
                export(end+1, :) = [{subject(c1).name}, subfolders(c2), {files(c3).name}, num2cell([NaN NaN NaN NaN NaN NaN NaN NaN NaN])];
               end
            end
        end
    end
end

save('export.mat', 'export');

end