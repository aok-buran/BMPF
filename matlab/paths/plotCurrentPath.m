% Рисование графика конкретного пути

close all;
clear all;

json = readJSON('exp_1_path.json'); % путь к файлу с результатами экспериментов
expNum = 1; % номер эксперимента
algorithmNum = 1; % номер алгоритма

%states = table2array(json.data);

disp(json);

expCnt = size(json.data,1);
algorithmCnt = size(json.data,2);

statesStruct = json.data(expNum,algorithmNum).states();

states = [];

for i=1:1:size(statesStruct,1)
    statesStruct(i)
    states = [states; struct2array(statesStruct(i))'];
end

json = readJSON('exp_1_report.json');
tm = json.data(expNum).time(algorithmNum);


f_my_plot(states,{'$q_1$','$q_2$','$q_3$', '$q_4$', '$q_5$', '$q_6$'}, ...
    'out/test.png','northeast', ...
    sprintf("затраченное время на планирование: %.3f сек.",tm), ...
    'номер шага планирования',    'угол поворота (рад.)' ...
)

