% Alexandre Naaim
% 14/03/2016

% Circ Stat  : A Matlab toolbox for circular statistics 2009 Philips berens
% Journal of Statistical Software 
% http://webspace.ship.edu/pgmarr/Geo441/Lectures/Lec%2016%20-%20Directional%20Statistics.pdf
% http://www.ncss.com/wp-content/themes/ncss/pdf/Procedures/NCSS/Circular_Data_Analysis.pdf

% Calcul de la moyenne en utilisant des statistiques circulaires c'est à
% dire prenant en compte le faite que lorsque l'on fait la moyenne sur un
% cercle trigonometrique deux valeurs proche de 0 et 2pi sont proche (la
% moyenne de 1° et 359° n'est pas 180° mais bien 0°)

%Input
% Angle : série d'angle dont on souhaite obtenir la moyenne variance et
% standards deviation (en radian)

% Ouput
% angularMean : moyenne calculée en utilisant les statistiques circulaire
% angularVariance : variance.............................................
% angularStd : standard deviation........................................

function [angularMean,angularVariance,angularStd] = circularMeanStd(angle)

Y = sin(angle);
Y = mean(Y);

X = cos(angle);
X = mean(X);

r = sqrt(X^2+Y^2);

Y = Y/r;
X = X/r;

%% Mean 
% atan2 allow to determine in which quadrant of the trigonometric circle is
% contain the value allowing to obtain the real angle 
angularMean = atan2(Y,X);

%% Variance
angularVariance = 1-r;
%% Angular deviation
% ranged from 0 to sqrt(2) => donne une standard deviation entre 0 et 1.41
% rad( 0 et 81°)
angularStd = sqrt(2*(1-r));

% ranged from 0 to +infini
% angularStd = sqrt(-2*ln(r))

end