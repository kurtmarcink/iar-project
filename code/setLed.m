function setLed(s,ledNumber,actionNumber)
fprintf(s,['L,' num2str(ledNumber) ',' num2str(actionNumber)]);
fscanf(s);
end