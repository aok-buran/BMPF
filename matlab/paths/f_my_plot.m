function f_my_plot( my,legendY,pathname,position,tit,mxlabel,mylabel)
f = figure('pos',[30 60 1600 900]);  
plot(my,'LineWidth',2);
title(tit);
xlabel(mxlabel);
ylabel(mylabel);
legend_handle = legend(legendY,'Interpreter','latex','Location',position);
legend_handle.FontSize = 16;

set(legend_handle);
grid on;
saveas(f,pathname);

end

