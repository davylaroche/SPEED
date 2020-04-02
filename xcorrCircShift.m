

function [crossCorr,lag] = xcorrCircShift(signalBase,signalShifted)

%% filtrer les données

Fs=100; % sample frequency
Fc=2; %cutoff frequency
[B,A]=butter(4,Fc/(Fs/2));


signalBase_filt=filtfilt(B,A,signalBase);

signalShifted_filt=filtfilt(B,A,signalShifted);



% x = 1:size(signalBase,1);
% x = x';
% y = signalBase;
% yu = max(y);
% yl = min(y);
% yr = (yu-yl);                               % Range of ‘y’
% 
% per = size(signalBase,1);                     % Estimate period
% ym = mean(y);                               % Estimate offset
% 
% fit = @(b,x)  b(1).*(sin(2*pi*x./b(2) + 2*pi/b(3))) + b(4);    % Function to fit
% fcn = @(b) sum((fit(b,x) - y).^2);                              % Least-Squares cost function
% s = fminsearch(fcn, [yr;  per;  -1;  ym]);                       % Minimise Least-Squares
% 
% xp = linspace(min(x),max(x));
% signalBase_filt = fit(s,xp);
% 
% 
% x = 1:size(signalShifted,1);
% x = x';
% y = signalShifted;
% yu = max(y);
% yl = min(y);
% yr = (yu-yl);                               % Range of ‘y’
% 
% per = size(signalShifted,1);                     % Estimate period
% ym = mean(y);                               % Estimate offset
% 
% fit = @(b,x)  b(1).*(sin(2*pi*x./b(2) + 2*pi/b(3))) + b(4);    % Function to fit
% fcn = @(b) sum((fit(b,x) - y).^2);                              % Least-Squares cost function
% s = fminsearch(fcn, [yr;  per;  -1;  ym]);                       % Minimise Least-Squares
% 
% xp = linspace(min(x),max(x));
% signalShifted_filt = fit(s,xp);





crossCorr(1) = sum(signalBase_filt.*signalShifted_filt);

for ind1 = 1:size(signalBase,1)
    crossCorr(ind1+1) = sum(circshift(signalBase_filt,ind1,1).*signalShifted_filt);
end

if abs(max(crossCorr))> abs(min(crossCorr))
    [~,lag] = max(crossCorr);
    lag = (lag-1)*2*pi/size(signalBase,1);
else
    [~,lag] = min(crossCorr);
    lag = ((lag-1)*2*pi/size(signalBase,1))+pi;
end

% signalBase_filt=filtfilt(B,A,signalBase);
% 
% signalShifted_filt=filtfilt(B,A,signalShifted);
% 
% 
% [~,posBase] = max(signalBase_filt);
% 
% [~,posShifted] = max(signalShifted_filt);

% lagpic = posShifted-posBase
% 
% lagpic = ((lagpic-1)*2*pi/size(signalBase,1));


% figure(1)
% plot(crossCorr)
% % 
% lag
% 
% close(figure(2))
% figure(2)
% hold on
% plot(signalBase,'r')
% plot(signalShifted,'b')
% plot(signalBase_filt,'g')
% plot(signalShifted_filt,'k')
% 
% lag
% lagpic
% 'test'
