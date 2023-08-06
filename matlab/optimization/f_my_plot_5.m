function f_my_plot_5( yData1,yData2,yData3,yData4,yData5,legendY,pathname,position,titles,mxlabel,mylabel)
f = figure('pos',[30 60 1200 1200]);  

disp(titles(1));

subplot(3, 4, [1 2]);
plot(yData1,'LineWidth',2);
title(titles(1));
xlabel(mxlabel);
ylabel(mylabel);
legend_handle = legend(legendY,'Interpreter','latex','Location',position);
grid on;

subplot(3, 4, [3 4]);
plot(yData2,'LineWidth',2);
title(titles(2));
xlabel(mxlabel);
ylabel(mylabel);
grid on;

subplot(3, 4, [5 6]);
plot(yData3,'LineWidth',2);
title(titles(3));
xlabel(mxlabel);
ylabel(mylabel);
legend_handle = legend(legendY,'Interpreter','latex','Location',position);
grid on;

subplot(3, 4, [7 8]);
plot(yData4,'LineWidth',2);
title(titles(4));
xlabel(mxlabel);
ylabel(mylabel);
legend_handle = legend(legendY,'Interpreter','latex','Location',position);
grid on;

subplot(3, 4, [10 11]);
plot(yData5,'LineWidth',2);
title(titles(5));
xlabel(mxlabel);
ylabel(mylabel);
legend_handle = legend(legendY,'Interpreter','latex','Location',position);
grid on;


saveas(f,pathname);

end

