function plot3_position_camera(track,displayname)

track=double(track);
plot3(track(1,:),track(2,:), track(3,:),'-o', 'LineWidth', 0.1, 'DisplayName', displayname);
grid on;
legend('Location', 'NorthEast');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('camera track');
axis equal;
end