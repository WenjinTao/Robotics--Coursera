function Display3D(Cset, Rset, X)

for i = 1 : length(Cset)
    hold on
    DisplayCameraPlane(Cset{i}, Rset{i}, 0.5);
end
hold on
plot3(X(:,1), X(:,2), X(:,3), 'b.');
axis equal

