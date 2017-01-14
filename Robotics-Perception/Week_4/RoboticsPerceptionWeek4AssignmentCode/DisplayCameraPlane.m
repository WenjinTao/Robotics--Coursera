function handleP = DisplayCameraPlane(C, R, windowScale, handleP)

R = R';
window11 = windowScale*[1;1;1]; window12 = windowScale*[-1; 1; 1];   window21 = windowScale*[-1;-1;1];    window22 = windowScale*[1; -1; 1];
windowPrime11 = R*window11+C; windowPrime12 = R*window12+C; windowPrime21 = R*window21+C; windowPrime22 = R*window22+C;

plot3(C(1), C(2), C(3), 'ko');
hold on
plot3([C(1) C(1)+windowScale*R(1,2)], [C(2) C(2)+windowScale*R(2,2)], [C(3) C(3)+windowScale*R(3,2)], 'g-');
hold on
plot3([C(1) C(1)+windowScale*R(1,1)], [C(2) C(2)+windowScale*R(2,1)], [C(3) C(3)+windowScale*R(3,1)], 'r-');
if nargin == 3
    handleP = plot3([windowPrime11(1), windowPrime12(1), windowPrime21(1), windowPrime22(1), windowPrime11(1), windowPrime21(1), windowPrime12(1), windowPrime22(1)],...
        [windowPrime11(2), windowPrime12(2), windowPrime21(2), windowPrime22(2), windowPrime11(2), windowPrime21(2), windowPrime12(2), windowPrime22(2)], ...
        [windowPrime11(3), windowPrime12(3), windowPrime21(3), windowPrime22(3), windowPrime11(3), windowPrime21(3), windowPrime12(3), windowPrime22(3)], 'k-');
    
else
    set(handleP, 'XData', [windowPrime11(1), windowPrime12(1), windowPrime21(1), windowPrime22(1), windowPrime11(1), windowPrime21(1), windowPrime12(1), windowPrime22(1)],...
        'YData', [windowPrime11(2), windowPrime12(2), windowPrime21(2), windowPrime22(2), windowPrime11(2), windowPrime21(2), windowPrime12(2), windowPrime22(2)], ...
        'ZData', [windowPrime11(3), windowPrime12(3), windowPrime21(3), windowPrime22(3), windowPrime11(3), windowPrime21(3), windowPrime12(3), windowPrime22(3)]);
end
grid on
% axis equal