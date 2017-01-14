function DisplayCorrespondence(img, x, xp)
%% Display correspondence points between SIFT keypoints and reprojection
% img: image to display
% x: size of (n, 2). SIFT keypoints locations
% xp: size of (n, 2). Reprojection locations

figure; imshow(img);
hold on;
err = 0;
for i = 1 : size(x, 1)
    % Draw lines between corresponding SIFT keypoints and reprojection
    plot([xp(i,1), x(i,1)], [xp(i,2), x(i,2)], ...
        'LineWidth', 1, ...
        'Color', 'red');
    % Draw SIFT keypoints
    scatter(x(i,1), x(i,2), 'o', ...
        'LineWidth',1, ...
        'MarkerEdgeColor', 'blue', ...
        'MarkerFaceColor', 'blue');
    err = err + (x(i,:) - xp(i,:)).^2;
end
% Display RMS error
text(1170, 20, ...
        sprintf('Error: %1.2f', sqrt(sum(err)/size(x,1))), ...
        'Color', 'white', ...
        'FontSize', 14);
hold off;



end

