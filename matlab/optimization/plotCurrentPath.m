% Рисование графика конкретного пути

close all;
clear all;

json = readJSON('exp_1_paths.json'); % путь к файлу с результатами экспериментов
jsonO = readJSON('exp_1_paths_optimized.json'); % путь к файлу с результатами экспериментов

expNum = 3; % номер эксперимента
algorithmNum = 1; % номер алгоритма

%states = table2array(json.data);

%disp(json);
%disp(json(1))
%disp(json(1,1))

statesStruct = json.data(expNum,algorithmNum).states();

states = [];

for i=1:1:size(statesStruct,1)
    statesStruct(i);
    states = [states; struct2array(statesStruct(i))'];
end

% median 1_1
states1_1 = [];
statesStruct1_1 = jsonO(expNum,algorithmNum).states();

for i=1:1:size(statesStruct1_1,1)
    statesStruct1_1(i);
    states1_1 = [states1_1; struct2array(statesStruct1_1(i))'];
end

states1_3 = [];
statesStruct1_3 = jsonO(expNum+3,algorithmNum).states();

for i=1:1:size(statesStruct1_3,1)
    statesStruct1_3(i);
    states1_3 = [states1_3; struct2array(statesStruct1_3(i))'];
end


states1_5 = [];
statesStruct1_5 = jsonO(expNum+6,algorithmNum).states();

for i=1:1:size(statesStruct1_5,1)
    statesStruct1_5(i);
    states1_5 = [states1_5; struct2array(statesStruct1_5(i))'];
end


states1_7 = [];
statesStruct1_7 = jsonO(expNum+9,algorithmNum).states();

for i=1:1:size(statesStruct1_7,1)
    statesStruct1_7(i);
    states1_7 = [states1_7; struct2array(statesStruct1_7(i))'];
end


json = readJSON('exp_1_reports.json');

tm = json(expNum).time(algorithmNum);

tms = [
   json(expNum).time(algorithmNum),
   json(expNum+3).time(algorithmNum),
   json(expNum+6).time(algorithmNum),
   json(expNum+9).time(algorithmNum)
];

algorithmsCaptions = [
   "исходный путь","1 проход", "3 прохода", "5 проходов", "7 проходов"
];

captionData = ["исходный путь"];

for i=2:1:5
    captionData = [captionData;algorithmsCaptions(i)+sprintf(", затрачено времени: %.1f мс.",tms(i-1)*1000)];
end


f_my_plot_5(states, states1_1,states1_3,states1_5,states1_7, ...
    {'$q_1$','$q_2$','$q_3$', '$q_4$', '$q_5$', '$q_6$'}, ...
    'out/median_1_.png','northwest', captionData,...
    'номер шага планирования', 'угол поворота (рад.)' ...
)






