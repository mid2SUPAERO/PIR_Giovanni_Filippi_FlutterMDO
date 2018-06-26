function myplot_signal(varargin)
% myplot_signal format the plot using a few standard formatting

figure;

plot(varargin{:}, 'LineWidth', 2);
grid on;
box on;

set(gca, 'FontSize', 16);
xlabel('t (secs)', 'FontSize', 16);

end