function [interpol, newscale] = sgnnrml(data, pts2interpol)
%%%%%%%%%%%%%%%%% This function normalize the data set to n pts length
%%%%%%%%%%%%  Data must be interpolated

[m n] = size(data);
interpol= [];
    
for i = 1:n
    
    line = size(data(isnan(data(:, i))==0, i), 1);
    oldscale = 1:line;
    newscale = linspace(1,line,pts2interpol);

%%%%%%%%%%%%%%%%%% Find the function and recompute function values for
%%%%%%%%%%%%%%%%%% newscale
    if line>10
        pp = spline(oldscale, data(isnan(data(:, i))==0, i));
        myfun = @(oldscale, y) ppval(pp, oldscale);
        interpol(1:pts2interpol, i) = myfun(newscale)';
    else
        interpol(1:pts2interpol, i) = NaN;
    end
     
end

end