function f_my_plot_2( yData1,yData2,legendY,pathname,position,titles,mxlabel,mylabel)
f = figure('pos',[30 60 1200 1200]);  

disp(titles(1));

subplot(2,1, 1);
plot(yData1,'LineWidth',2);
title(titles(1));
xlabel(mxlabel);
ylabel(mylabel);
legend_handle = legend(legendY,'Interpreter','latex','Location',position);
grid on;

subplot(2, 1,2);
plot(yData2,'LineWidth',2);
title(titles(2));
xlabel(mxlabel);
ylabel(mylabel);
legend_handle = legend(legendY,'Interpreter','latex','Location',position);
grid on;

saveas(f,pathname);

end

