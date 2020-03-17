
function [C] = normrMaison(A)

B = sqrt(sum(A.*A,2));
C = [];
for id = 1: size(B,1)
    C = [C; A(id,:)*1/B(id)];
end
%function y = normr_(x)

end