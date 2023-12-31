function plot_error(error)

mean_error =mean(error)*ones(1,length(error));
time= 1:length(error);
plot(time,error,'-', 'LineWidth', 0.5);

hold on;
line(time,mean_error, 'Color', 'red', 'LineStyle', '--');
text(time(floor(length(time)/2)), mean(error), sprintf('mean_error: %.2f', mean(error)), 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', 'Color', 'red');

hold off;
grid on;
xlabel('frames_i-1/frame_i');
ylabel('error');
title('Reprojection Error，T_i-1/T_i');
end