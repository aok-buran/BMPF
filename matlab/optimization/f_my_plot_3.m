function f_my_plot_3( yData1,yData2,yData3,legendY,pathname,position,titles,mxlabel,mylabel)
f = figure('pos',[30 60 1200 1200]);  

disp(titles(1));

subplot(2, 4, [1 2]);
plot(yData1,'LineWidth',2);
title(titles(1));
xlabel(mxlabel);
ylabel(mylabel);
legend_handle = legend(legendY,'Interpreter','latex','Location',position);
grid on;

subplot(2, 4, [3 4]);
plot(yData2,'LineWidth',2);
title(titles(2));
xlabel(mxlabel);
ylabel(mylabel);
legend_handle = legend(legendY,'Interpreter','latex','Location',position);
grid on;

subplot(2, 4, [6 7]);
plot(yData3,'LineWidth',2);
title(titles(3));
xlabel(mxlabel);
ylabel(mylabel);
legend_handle = legend(legendY,'Interpreter','latex','Location',position);
grid on;

saveas(f,pathname);

end

