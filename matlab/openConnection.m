function s = openConnection
s = serial('/dev/ttyS0');
fopen(s);
end
