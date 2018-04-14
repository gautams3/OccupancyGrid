function output_txt = myfunction(obj,event_obj)
% Display the position of the data cursor
% obj          Currently not used (empty)
% event_obj    Handle to event object
% output_txt   Data cursor text string (string or cell array of strings).

pos = get(event_obj,'Position');
output_txt = {['X: ',num2str(pos(1),4), ' Y: ',num2str(pos(2),4)]};

c = pos(1);
r = pos(2);
logodds = event_obj.Target.CData(r,c);
probability = 1 - 1./(1 + exp(logodds));
output_txt{end+1} = ['LogOdds: ' num2str(logodds,4)];
output_txt{end+1} = ['Prob: ' num2str(probability,4)];
end