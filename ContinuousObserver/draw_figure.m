function [] = draw_figure(x, y, axis_range, label)
    plot(x, y,'LineWidth',1.5,'LineStyle','-.');
    axis(axis_range);
    % ylabel(label,'Interpreter','latex', 'FontSize', 11);
    grid on;
    set(gca,'GridLineStyle','-.', 'FontSize', 11);
    set(gcf,'position',[200,200,400,300]);
    xlabel('time (s)','FontSize', 11);
    legend('Joint #1', 'Joint #2', 'Joint #3');
    title(label,'Interpreter','latex', 'FontSize', 11);
end

