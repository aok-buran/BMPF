% рисование графиков первого эксперимента
close all;
clear all;



reports = [
    "..\..\out\reports\exp_2_report.json",
    "..\..\out\reports\exp_2_2_report.json",
    "..\..\out\reports\exp_2_3_report.json",
    "..\..\out\reports\exp_2_4_report.json",
    "..\..\out\reports\exp_2_5_report.json",
    "..\..\out\reports\exp_2_6_report.json",
    "..\..\out\reports\exp_2_7_report.json"
];


tms = [];
tms2 = [];
tms3 = [];

for i=1:1:size(reports)
    json = readJSON(reports(i));
    for j=1:1:size(json.data,1)
        disp(json.data(j));
        isValid = json.data(j).isValid();   
        if (all(isValid==1))
            tm = json.data(j).time();    
            tms = [tms;tm(3:4)'];
            tms2 = [tms2;tm(2:3)'];
            tms3 = [tms3;tm(1:3)'];
        end
    end
end

f_my_plot_dots(tms3,{'one direction','ordered one direction', ...
    'multirobot'}, ...
    'out/exp_2_time.png','northeast', ...
   'Сцена с четырьмя роботами', ...
    'номер эксперимента', 'время планирования (с.)' ...
)

f_my_plot_dots(tms2,{'ordered one direction', 'multirobot'}, ...
    'out/exp_2_time_2.png','northeast', ...
   'Сцена с четырьмя роботами', ...
    'номер эксперимента', 'время планирования (с.)' ...
)

f_my_plot_dots(tms,{'multirobot', 'continuous'}, ...
    'out/exp_2_time_3.png','northeast', ...
   'Сцена с четырьмя роботами', ...
    'номер эксперимента', 'время планирования (с.)' ...
)


json2 = readJSON("..\..\out\paths\exp_2_paths.json"); % путь к файлу с результатами экспериментов
expNum = 1; % номер эксперимента
disp(size(json2.data));

expCnt = size(json2.data,1);
algorithmCnt = size(json2.data,2);

algorithmNum = 1;  
statesStruct = json2.data(expNum,algorithmNum).states();
yData1 = [];
for i=1:1:size(statesStruct,1)
   yData1 = [yData1; struct2array(statesStruct(i))'];
end
    
algorithmNum = 2;  
statesStruct = json2.data(expNum,algorithmNum).states();
yData2 = [];
for i=1:1:size(statesStruct,1)
   yData2 = [yData2; struct2array(statesStruct(i))'];
end

algorithmNum = 3;  
statesStruct = json2.data(expNum,algorithmNum).states();
yData3 = [];
for i=1:1:size(statesStruct,1)
   yData3 = [yData3; struct2array(statesStruct(i))'];
end


algorithmNum = 4;  
statesStruct = json2.data(expNum,algorithmNum).states();
yData4 = [];
for i=1:1:size(statesStruct,1)
   yData4 = [yData4; struct2array(statesStruct(i))'];
end



algorithmsCaptions = [
   "one direction","ordered one direction", "multirobot", "continuous"
];

captionData = [];
captionData3 = [];

for algorithmNum = 1:1:algorithmCnt % номер алгоритма   
    json3 = readJSON( "..\..\out\reports\exp_2_report.json");
    tm = json3.data(expNum).time(algorithmNum);
    if algorithmNum<=3
        captionData3 = [captionData3;algorithmsCaptions(algorithmNum)+sprintf(": %.1f мс.",tm)];
    end
    if algorithmNum>=3
        captionData = [captionData;algorithmsCaptions(algorithmNum)+sprintf(": %.1f мс.",tm)];
    end
end

disp(captionData3);

f_my_plot_3(yData1,yData2,yData3, ...
    {'$q^1_1$','$q^1_2$','$q^1_3$', '$q^1_4$', '$q^1_5$', '$q^1_6$', ...
    '$q^2_1$','$q^2_2$','$q^2_3$', '$q^2_4$', '$q^2_5$', '$q^2_6$', ...
    '$q^3_1$','$q^3_2$','$q^3_3$', '$q^3_4$', '$q^3_5$', '$q^3_6$', ...
    '$q^4_1$','$q^4_2$','$q^4_3$', '$q^4_4$', '$q^4_5$', '$q^4_6$'}, ...
    'out/exp_2_paths.png','northwest', captionData3,...
    'номер шага планирования', 'время планирования (с.)' ...
)


f_my_plot_2(yData3,yData4, ...
    {'$q^1_1$','$q^1_2$','$q^1_3$', '$q^1_4$', '$q^1_5$', '$q^1_6$', ...
    '$q^2_1$','$q^2_2$','$q^2_3$', '$q^2_4$', '$q^2_5$', '$q^2_6$', ...
    '$q^3_1$','$q^3_2$','$q^3_3$', '$q^3_4$', '$q^3_5$', '$q^3_6$', ...
    '$q^4_1$','$q^4_2$','$q^4_3$', '$q^4_4$', '$q^4_5$', '$q^4_6$'}, ...
    'out/exp_2_paths_2.png','northwest', captionData,...
    'номер шага планирования', 'время планирования (с.)' ...
)




