function [Cset Rset] = ExtractCameraPose(E)

W = [0 -1 0;1 0 0;0 0 1];
[U,D,V] = svd(E);
R = U*W*V';
C = U(:,3);
if det(R) < 0
    R = -R;
    C = -C;
end
Rset{1} = R;
Cset{1} = -R'*C;

R = U*W*V';
C = -U(:,3);
if det(R) < 0
    R = -R;
    C = -C;
end
Rset{2} = R;
Cset{2} = -R'*C;

R = U*W'*V';
C = U(:,3);
if det(R) < 0
    R = -R;
    C = -C;
end
Rset{3} = R;
Cset{3} = -R'*C;

R = U*W'*V';
C = -U(:,3);
if det(R) < 0
    R = -R;
    C = -C;
end
Rset{4} = R;
Cset{4} = -R'*C;
