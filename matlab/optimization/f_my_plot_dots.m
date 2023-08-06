function f_my_plot_dots( my,legendY,pathname,position,tit,mxlabel,mylabel)
f = figure('pos',[30 60 1600 900]); 
plot(my,'-*','LineWidth',2,'MarkerSize',20);
title(tit);
ylabel(mylabel);
xlabel(mxlabel);
legend_handle = legend(legendY,'Interpreter','latex','Location',position);
legend_handle.FontSize = 16;

set(legend_handle);
grid on;
saveas(f,pathname);

end

