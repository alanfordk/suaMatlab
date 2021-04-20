function results = genResultsTable(stepinfo_control,...
    stepinfo_guidance, bw_control, bw_guidance)
%GENRESULTSTABLE generates a table summarizing results of guidance model
%tuning
metric = {'Rise time(sec)';'Settling time(sec)';'Bandwidth(Hz)'};
control = [stepinfo_control.RiseTime; stepinfo_control.SettlingTime;
    bw_control];
guidance = [stepinfo_guidance.RiseTime;stepinfo_guidance.SettlingTime;
    bw_guidance];
percent_difference = (control - guidance)./control*100;
results = table(metric, control, guidance, percent_difference);
end

