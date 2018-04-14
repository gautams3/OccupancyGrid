function inflatedMap = inflate(map, se)
%This function is for internal use only. It may be removed in the future.

%INFLATE Inflate occupancy grid
%   IM = inflate(MAP, SE) returns an N-by-M logical array of inflated map
%   from the N-by-M array of logical input map and a P-by-Q array of
%   logical structuring element SE.

%   Copyright 2014 The MathWorks, Inc.

%#codegen
% assert(isa(map, 'logical'));

inflatedMap = map;

[numRows, numCols] = size(map);

% Find the index of the center of the structuring element
seCenter = [ceil(size(se,1)/2), ceil(size(se,2)/2)];

% Get indices for structuring element
[rowIdx, colIdx] = ind2sub(size(se), 1:length(se(:)));

% Convert to logical(Bin Occ Grid) or probabilistic (Occ Grid)
if isa(map,'logical')
    isBOG = true;
    se = logical(se(:));
else
    isBOG = false;
end

for i = 1:numRows
    for j = 1:numCols
        % Skip if the cell is not occupied
        if ~map(i,j)
            continue;
        end
        
        % Translated indices of the structuring element
        shiftedRowIdx = i-seCenter(1)+ rowIdx;
        shiftedColIdx = j-seCenter(2)+ colIdx;
        
        % Generate logicals for indices that are within grid limits
        idx  = (shiftedRowIdx(:) > 0) & ...
            (shiftedColIdx(:) > 0) & ...
            (shiftedRowIdx(:) <= numRows) & ...
            (shiftedColIdx(:) <= numCols);
        
        index = sub2ind([numRows, numCols], ...
            shiftedRowIdx(idx) , shiftedColIdx(idx));
        
        if isBOG
            subse = se(idx); %logical 1 or 0
            inflatedMap(index) = inflatedMap(index) | subse';
        else
            subse = se(idx) * map(index); % 1 or 0 multiplied by the value of the original cell (not using inflatedMap(index) because its value changes)
            inflatedMap(index) = max(inflatedMap(index), subse');
        end
        
    end
end
end

