function logodds = logOddsFromProb(prob)
% Calculate logOdds from a given probability

             validateattributes(prob, {'numeric'}, ...
                {'finite', '<=', 1, '>=' 0}, '');
            
            logodds = log(prob./(1-prob));
end