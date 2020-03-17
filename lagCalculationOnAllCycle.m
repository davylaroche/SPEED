

function [lagYaw,anchoringYaw,lagRoll,anchoringRoll,Angle] = lagCalculationOnAllCycle(fileName,side,calculationType)

pointsLabel = {'LSHO', 'RSHO', 'LPSI','RPSI','LASI','RASI','LFHD','RFHD','RASI','LASI'};
num = length(pointsLabel);
indglob = 1;

if strcmp(side,'Right')
    C_side = 'Left';
    C_side_t = 'R';
    forceName = 'RGroundReactionForce';
else
    C_side ='Right';
    C_side_t = 'L';
    forceName = 'LGroundReactionForce';
end


lagYaw = [];
lagRoll = [];
anchoringYaw = [];
anchoringRoll = [];

Angle.Yaw.AngleShouldGlob = [];
Angle.Yaw.AnglePelvisGlob = [];
Angle.Yaw.AngleHeadGlob = [];

Angle.Roll.AngleShouldGlob = [];
Angle.Roll.AnglePelvisGlob = [];
Angle.Roll.AngleHeadGlob = [];


Angle.Yaw.AmplitudeShould = [];
Angle.Yaw.AmplitudePelvis = [];
Angle.Yaw.AmplitudeHead = [];

Angle.Roll.AmplitudeShould = [];
Angle.Roll.AmplitudePelvis = [];
Angle.Roll.AmplitudeHead = [];


Angle.Yaw.lag.Formula.HeadShoulder = [];
Angle.Yaw.lag.Formula.ShouldPelvis = [];

Angle.Yaw.lag.CrossCorr.HeadShoulder = [];
Angle.Yaw.lag.CrossCorr.ShouldPelvis = [];

Angle.Yaw.lag.Fourier.HeadShoulder = [];
Angle.Yaw.lag.Fourier.ShouldPelvis = [];

Angle.Yaw.lag.CRP.HeadShoulder = [];
Angle.Yaw.lag.CRP.ShouldPelvis = [];

Angle.Roll.lag.Formula.HeadShoulder = [];
Angle.Roll.lag.Formula.ShouldPelvis = [];

Angle.Roll.lag.CrossCorr.HeadShoulder = [];
Angle.Roll.lag.CrossCorr.ShouldPelvis = [];

Angle.Roll.lag.Fourier.HeadShoulder = [];
Angle.Roll.lag.Fourier.ShouldPelvis = [];

Angle.Roll.lag.CRP.HeadShoulder = [];
Angle.Roll.lag.CRP.ShouldPelvis = [];


Angle.Vitesse = [];


acq = btkReadAcquisition(fileName);
points = btkGetPoints(acq);
Events = btkGetEvents(acq);
frq = btkGetPointFrequency(acq);
% On va normalisé ca sur un cycle de Right Heel Strike a Right Heel Strike



firstTime = btkGetFirstFrame(acq)/btkGetPointFrequency(acq);
lastTime = btkGetLastFrame(acq)/btkGetPointFrequency(acq);

if isfield(points,'LSHO') && isfield(points,'RSHO') ...
        && isfield(points,'LPSI') && isfield(points,'RPSI')...
        && isfield(points,'LASI') && isfield(points,'RASI')...
        && isfield(points,'LFHD') && isfield(points,'RFHD')...
        
    %     if all(all(~isnan(points.LSHO)))&& all(all(~isnan(points.RSHO)))...
    %             && all(all(~isnan(points.LPSI)))&& all(all(~isnan(points.RPSI)))...
    %             && all(all(~isnan(points.LFHD)))&& all(all(~isnan(points.RFHD)))...
    %             && all(all(~isnan(points.LASI)))&& all(all(~isnan(points.RASI)))
    if all(any(points.LSHO,2))&& all(any(points.RSHO,2))...
            && all(any(points.LPSI,2))&& all(any(points.RPSI,2))...
            && all(any(points.LFHD,2))&& all(any(points.RFHD,2))...
            && all(any(points.LASI,2))&& all(any(points.RASI,2))
        
        if isfield(Events,'Right_Foot_Strike') && isfield(Events,'Left_Foot_Strike') ...
                && isfield(Events,'Right_Foot_Off') && isfield(Events,'Left_Foot_Off')
            
            
            % On enleve les evenement avant la premiere frame
            Events.Right_Foot_Off = Events.Right_Foot_Off (Events.Right_Foot_Off > firstTime & Events.Right_Foot_Off < lastTime);
            Events.Right_Foot_Strike = Events.Right_Foot_Strike (Events.Right_Foot_Strike > firstTime & Events.Right_Foot_Strike < lastTime);
            Events.Left_Foot_Strike = Events.Left_Foot_Strike (Events.Left_Foot_Strike > firstTime &  Events.Left_Foot_Strike < lastTime);
            Events.Left_Foot_Off = Events.Left_Foot_Off (Events.Left_Foot_Off > firstTime & Events.Left_Foot_Off < lastTime);
            
            % On reteste si c'est pas vide
            if isfield(Events,'Right_Foot_Strike') && isfield(Events,'Left_Foot_Strike') ...
                    && isfield(Events,'Right_Foot_Off') && isfield(Events,'Left_Foot_Off')
                
                
                % Test sur tout les cycles
                % Foot Strike
                %FSstr = 'Right_Foot_Strike';
                
                FSstr = [side,'_Foot_Strike'];
                % Cell2Str before we have a cell with strcat using str{} allow to get a
                % string that can be used in dynamic structure calling
                FS = Events.(FSstr);
                
                % Controlateral FS
                %C_FSstr = 'Left_Foot_Strike';
                C_FSstr = [C_side,'_Foot_Strike'];
                C_FS = Events.(C_FSstr);
                
                % Foot off
                %FOstr = 'Right_Foot_Off';
                FOstr = [side,'_Foot_Off'];
                FO = Events.(FOstr);
                
                % Controlateral Foot strike
                %C_FOstr =  'Left_Foot_Off';
                C_FOstr = [C_side,'_Foot_Off'];
                C_FO = Events.(C_FOstr);
                
                FO = FO(FO>FS(1));
                C_FS = C_FS(C_FS>FS(1));
                C_FO = C_FO(C_FO>FS(1));
                
                for ind4 = 1:(size(FS,2)-1)
                    % We need to test that between 2 foot strike there is a foot of and
                    % a controlateral foot strike and off
                    testFO = isempty(FO(FO>FS(ind4) & FO<FS(ind4+1)));
                    %                     testC_FO = isempty(C_FO(C_FO>FS(ind4) & C_FO<FS(ind4+1)));
                    %                     testC_FS = isempty(C_FS(C_FS>FS(ind4) & C_FS<FS(ind4+1)));
                    % If one of this condition is not respected we add at the beginning
                    % of the Event vector a 0 value in order to keep the correspondence
                    % between the foot strike event and
                    if testFO
                        FO = [0,FO];
                    end
                    %                     if testC_FO
                    %                         C_FO = [0,C_FO];
                    %                     end
                    %                     if testC_FS
                    %                         C_FS = [0,C_FS];
                    %                     end
                    
                    % If no test is empty (which mean that there is the event between
                    % the 2 foot strike) it is possible to compute all the different
                    % spation temporal parameter.
                    
                    %if not(testFO | testC_FO | testC_FS)
                    if not(testFO)
                        
                        %                         RHS1 = round(Events.Right_Foot_Strike(ind4)*frq);
                        %                         RHS2 = round(Events.Right_Foot_Strike(ind4+1)*frq);
                        
                        RHS1 = round(FS(ind4)*frq);
                        RHS2 = round(FS(ind4+1)*frq);
                        
                        RHS1 = RHS1-btkGetFirstFrame(acq)+1;
                        RHS2 = RHS2-btkGetFirstFrame(acq)+1;
                        
                        for k = 1:num
                            dataExtracted.(pointsLabel{k}) = points.(pointsLabel{k})(RHS1:RHS2,:);
                        end
                        
                        dataExtracted.thorax = points.([C_side_t,'ThoraxAngles'])(RHS1:RHS2,:);
                        dataExtracted.pelvis = points.([C_side_t,'PelvisAngles'])(RHS1:RHS2,:);
                        forcePlateforme = btkGetGroundReactionWrenches(acq);
                        factor = btkGetAnalogFrequency(acq)/btkGetPointFrequency(acq);
                       % if size(forcePlateforme,1)>1
                          %  if any(any(forcePlateforme(1).F(RHS1*factor:RHS2*factor,:),2)) || any(any(forcePlateforme(2).F(RHS1*factor:RHS2*factor,:),2))
                                
                                [AIHead,lagHeadShoulder,AIShoulder,lagShouldPelvis,AngleYaw] = anchoringIndex_V2(dataExtracted,frq,'yaw',calculationType);
                                anchoringYaw(indglob,:) = [AIHead,AIShoulder];
                                lagYaw(indglob,:) = [lagHeadShoulder,lagShouldPelvis];
                                
                                
                                [AIHead,lagHeadShoulder,AIShoulder,lagShouldPelvis,AngleRoll] = anchoringIndex_V2(dataExtracted,frq,'roll',calculationType);
                                anchoringRoll(indglob,:) = [AIHead,AIShoulder];
                                lagRoll(indglob,:) = [lagHeadShoulder,lagShouldPelvis];
                                
                                
                                
%                                 Angle.Yaw.AngleShouldGlob = [Angle.Yaw.AngleShouldGlob,AngleYaw.AngleShouldGlob];
%                                 Angle.Yaw.AnglePelvisGlob = [Angle.Yaw.AnglePelvisGlob,AngleYaw.AnglePelvisGlob];
%                                 Angle.Yaw.AngleHeadGlob = [Angle.Yaw.AngleHeadGlob,AngleYaw.AngleHeadGlob];
%                                 
%                                 Angle.Yaw.AmplitudeShould = [Angle.Yaw.AmplitudeShould,AngleYaw.AmplitudeShould];
%                                 Angle.Yaw.AmplitudePelvis = [Angle.Yaw.AmplitudePelvis,AngleYaw.AmplitudePelvis];
%                                 Angle.Yaw.AmplitudeHead = [Angle.Yaw.AmplitudeHead,AngleYaw.AmplitudeHead];
                                
                                
%                                 Angle.Yaw.lag.Formula.HeadShoulder = [Angle.Yaw.lag.Formula.HeadShoulder,AngleYaw.lag.Formula.HeadShoulder];
%                                 Angle.Yaw.lag.Formula.ShouldPelvis = [Angle.Yaw.lag.Formula.ShouldPelvis,AngleYaw.lag.Formula.ShouldPelvis];
%                                 
%                                 Angle.Yaw.lag.CrossCorr.HeadShoulder = [Angle.Yaw.lag.CrossCorr.HeadShoulder,AngleYaw.lag.CrossCorr.HeadShoulder];
%                                 Angle.Yaw.lag.CrossCorr.ShouldPelvis = [Angle.Yaw.lag.CrossCorr.ShouldPelvis,AngleYaw.lag.CrossCorr.ShouldPelvis];
%                                 
%                                 Angle.Yaw.lag.Fourier.HeadShoulder = [Angle.Yaw.lag.Fourier.HeadShoulder,AngleYaw.lag.Fourier.HeadShoulder];
%                                 Angle.Yaw.lag.Fourier.ShouldPelvis = [Angle.Yaw.lag.Fourier.ShouldPelvis,AngleYaw.lag.Fourier.ShouldPelvis];
                                
%                                 Angle.Yaw.lag.CRP.HeadShoulder = [Angle.Yaw.lag.CRP.HeadShoulder,AngleYaw.lag.CRP.HeadShoulder];
                            %    Angle.Yaw.lag.CRP.ShouldPelvis = [Angle.Yaw.lag.CRP.ShouldPelvis,AngleYaw.lag.CRP.ShouldPelvis];
                                
                                
                                
                                
%                                 Angle.Vitesse = [Angle.Vitesse,AngleYaw.Vitesse];
                                
%                                 Angle.Roll.AngleShouldGlob = [Angle.Roll.AngleShouldGlob,AngleRoll.AngleShouldGlob];
%                                 Angle.Roll.AnglePelvisGlob = [Angle.Roll.AnglePelvisGlob,AngleRoll.AnglePelvisGlob];
%                                 Angle.Roll.AngleHeadGlob = [Angle.Roll.AngleHeadGlob,AngleRoll.AngleHeadGlob];
                                
%                                 Angle.Roll.AmplitudeShould = [Angle.Roll.AmplitudeShould,AngleRoll.AmplitudeShould];
%                                 Angle.Roll.AmplitudePelvis = [Angle.Roll.AmplitudePelvis,AngleRoll.AmplitudePelvis];
%                                 Angle.Roll.AmplitudeHead = [Angle.Roll.AmplitudeHead,AngleRoll.AmplitudeHead];
%                                 
%                                 Angle.Roll.lag.Formula.HeadShoulder = [Angle.Roll.lag.Formula.HeadShoulder,AngleRoll.lag.Formula.HeadShoulder];
%                                 Angle.Roll.lag.Formula.ShouldPelvis = [Angle.Roll.lag.Formula.ShouldPelvis,AngleRoll.lag.Formula.ShouldPelvis];
%                                 
%                                 Angle.Roll.lag.CrossCorr.HeadShoulder = [Angle.Roll.lag.CrossCorr.HeadShoulder,AngleRoll.lag.CrossCorr.HeadShoulder];
%                                 Angle.Roll.lag.CrossCorr.ShouldPelvis = [Angle.Roll.lag.CrossCorr.ShouldPelvis,AngleRoll.lag.CrossCorr.ShouldPelvis];
%                                 
%                                 Angle.Roll.lag.Fourier.HeadShoulder = [Angle.Roll.lag.Fourier.HeadShoulder,AngleRoll.lag.Fourier.HeadShoulder];
%                                 Angle.Roll.lag.Fourier.ShouldPelvis = [Angle.Roll.lag.Fourier.ShouldPelvis,AngleRoll.lag.Fourier.ShouldPelvis];
                                
%                                 Angle.Roll.lag.CRP.HeadShoulder = [Angle.Roll.lag.CRP.HeadShoulder,AngleRoll.lag.CRP.HeadShoulder];
                          %      Angle.Roll.lag.CRP.ShouldPelvis = [Angle.Roll.lag.CRP.ShouldPelvis,AngleRoll.lag.CRP.ShouldPelvis];
                                
                                indglob = indglob+1;
                          %  end
                       % end
                    end
                end
            end
        end
    end
end

