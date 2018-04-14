function probability = probFromLogOdds(logodds)
%Calculate probability of occupancy from logOdds
validateattributes(logodds, {'numeric'}, {}, '');

probability = 1 - 1./(1 + exp(logodds));
end