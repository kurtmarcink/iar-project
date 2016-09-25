function sensorVals = readAmbient(s)
fprintf(s,'O');
sensorString = fscanf(s);
splitString = regexp(sensorString,',','split');
sensorVals = cellfun(@str2num,splitString(2:end));
end
