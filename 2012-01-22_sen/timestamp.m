fields = fieldnames(sensor_data);
for i = 1:length(fields)
    disp(['---- ', fields{i}, ' ----']);
    disp(sensor_data.(fields{i})(1:3, :)); 
end
