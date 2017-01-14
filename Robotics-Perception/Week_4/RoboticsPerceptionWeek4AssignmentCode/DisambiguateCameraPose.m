function [C R X] = DisambiguateCameraPose(Cset, Rset, Xset)

for i = 1 : length(Cset)
   D = Rset{i}*(Xset{i}'-Cset{i}*ones(1,size(Xset{i},1)));
   n(i) = length(find(D(3,:)>0 & Xset{i}(:,3)'>0));
end

[maxn, maxi] = max(n);
C = Cset{maxi};
R = Rset{maxi};
X = Xset{maxi};