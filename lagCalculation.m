% Alexandre Naaim
% 14/03/2016

% Calcul de déphasage entre deux signaux avec différentes méthodes
% disponible ici fourrier et cross correlation. Attention ce programme ne
% peut être utilisé que sur un cycle. En effet, par exemple avec la méthode
% de cross correlation on obtient un decalage en frame et on considere le
% temps

% Description :
% Hypothèse :


% Input :
%   SignalBase : signal de base par rapport du quel on veut connaitre le
%   depahasage
%   signalShifted : signal dephasé par rapport au premier. 

% Output :
%   lag
% Source :
% http://stackoverflow.com/questions/27545171/identifying-phase-shift-between-signals


function [lag] = lagCalculation(signalBase,signalShifted,calculationType)

% On prend la taille maximal du signal comme ca quelque soit l'orientation
% du fichier on peut le faire.
nbrFrame = max(size(signalBase,1));

calculationType = lower(calculationType);

switch calculationType
    case 'formula'
        %% Formule cf lien vers stackoverflow
        lag = acos(dot(signalBase,signalShifted)/(norm(signalBase)*norm(signalShifted)));
        
    case 'crosscorrelation'
        %% Cross Correlation
        % Calcul de la cross correlation
        %[crossCorr,lag] = xcorr(signalBase,signalShifted);
        [crossCorr,lag] = xcorrCircShift(signalBase,signalShifted);
        %[lag] = PicToPicCircShift(signalBase,signalShifted);
        
        
        % Normalisation
        %normVect = sqrt(sum(signalBase.^2)*sum(signalShifted.^2));
        %crossCorrNorm = crossCorr/normVect;
        
        % La fonction xcorr calcul le coefficient de correlation avec tout les
        % décalage possible. Ainsi l'absisse 0 correspond aux moment ou l'indice
        % est de m la taille du ficher
        
        %[~,I] = max(abs(crossCorrNorm)); % on determine le lag ou le cross correlation est maximum
        % On calcule le decalage transférer sur l'espace -Pi=>Pi en effet desfois
        % on trouver des decalafe
        %lag(I)
%         lag
%         nbrFrame
        %lag = lag(I)*2*pi/nbrFrame;
        %lag = lag*2*pi/nbrFrame;
%         wrapToPi(lag)
% %         
%         close(figure(1))
%         figure(1)
%         hold on
%         plot(signalBase,'r')
%         plot(signalShifted,'b')
        % xcorr donne le nbre de frame ou la deuxieme courbe doit etre
        % deplacer pour correspondre au mieux à la premiere on obtient donc
        % l'opposé du lag 
%         lag = -lag;
    case 'fourier'
        %% Transformé de Fourier
        m = length(signalBase); % Window length
        n = pow2(nextpow2(m)); % Transform length
        %f = (0:n-1)*(frq/n); % Frequency range
        
        % Calcul de la transformée de fourier des deux signaux
        baseFourier = fft(signalBase,n);
        shiftFourier = fft(signalShifted,n);
        
        shiftPower = shiftFourier.*conj(shiftFourier)/n; % Power of the DFT
        basePower = baseFourier.*conj(baseFourier)/n;
%         figure
%         hold on
%         plot(shiftPower)
%         plot(basePower)
%         
        % Détermination de la position du premier pic = la fréquence principal du
        % signal et donc celle sur laquelle on calculera le dephasage
        [~,shiftFourierInd] = max(shiftPower(1:round(end/2)));
        [~,baseFourierInd] = max(basePower(1:round(end/2)));
        
        % On utilise l'indice calculer précédemment pour déterminer le dephasage de
        % la fréquence principal contenu à la position. Angle donnant le dephasage
        % pour une serie complexe
        shiftAngleFourier = angle(shiftFourier);
        baseAngleFourier = angle(baseFourier);
        shiftAngleFourier = unwrap(angle(shiftFourier));
        baseAngleFourier = unwrap(angle(baseFourier));
        
        
        shiftPhaseShift = shiftAngleFourier(shiftFourierInd);
        basePhaseShift = baseAngleFourier(baseFourierInd);
        
        lag = shiftPhaseShift-basePhaseShift;

    case 'hilbert'
        %% Transformé d'Hilbert
        
        % Hilbert Transform
        shiftHilbert = hilbert(signalShifted);
        baseHilbert = hilbert(signalBase);
        
        % les deux chopses apres sont les meme Donnerait ici l'instantaneous phase
        % difference.
        lagHilbert = angle(shiftHilbert ./ baseHilbert);
        %p = unwrap(angle(shiftHilbert))-unwrap(angle(baseHilbert));
        
        lag = mean(lagHilbert);
        
    case 'crp'
        
        
        normBase = signalBase;
        %(2*(signalBase-min(signalBase))/(max(signalBase)-min(signalBase)))-1;
        diffBase = diff(normBase);
        
        diffBaseTempDebut = [diffBase(1,1);diffBase];
        diffBaseTempFin = [diffBase;diffBase(end,1)];
        
        diffnormBase = (diffBaseTempDebut+diffBaseTempFin)/2;
               
        
        normShift =signalShifted;
        %(2*(signalShifted-min(signalShifted))/(max(signalShifted)-min(signalShifted)))-1;
        diffShift = diff(normShift);
        
        diffShiftTempDebut = [diffShift(1,1);diffShift];
        diffShiftTempFin = [diffShift;diffShift(end,1)];
        
        diffnormShift = (diffShiftTempDebut+diffShiftTempFin)/2;
        
        CRPBase = atan2(diffnormBase,normBase);
        CRPShift = atan2(diffnormShift,normShift);
%         lag = mean(CRPShift-CRPBase);
        
        lagtemp = atan2(diffnormShift.*normBase-diffnormBase.*normShift,normBase.*normShift+diffnormShift.*diffnormBase);
        lag = circularMeanStd(lagtemp);
%         'blabla'
%         figure
%         subplot(1,2,2)
%         hold on
%         
%         plot(CRPBase,'k')
%         plot(CRPShift,'r')
%         plot(CRPShift-CRPBase,'g')
%         plot(lagtemp,'b')
%         subplot(1,2,1)
%         hold on
%         plot(normBase,'k')
%         plot(normShift,'r')
end

end
