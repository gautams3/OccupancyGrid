classdef (Sealed)OccupancyGrid < robotics.algs.internal.OccupancyGridBase
  %% OCCUPANCY GRID PROTOTYPE
  %% GAUTAM SALHOTRA
    %OCCUPANCYGRID Create a probabilistic occupancy grid
    %   OCCUPANCYGRID creates an occupancy grid map. Each cell has
    %   a probability representing the occupancy status of that cell,
    %   ranging from 0 (100% probability free ) to 1 (100% probability
    %   occupied)
    %
    %   MAP = robotics.OccupancyGrid(W, H) creates a 2D probabilistic
    %   occupancy grid object representing a world space of width(W) and
    %   height(H) in meters. The default grid resolution is 1 cell per meter.
    %
    %   MAP = robotics.OccupancyGrid(W, H, RES) creates a OccupancyGrid
    %   object with resolution(RES) specified in cells per meter.
    %
    %   MAP = robotics.OccupancyGrid(W, H, RES, 'world') creates a
    %   OccupancyGrid object and  specifies the map size (W and H)
    %   in the world coordinates. This is also the default value.
    %
    %   MAP = robotics.OccupancyGrid(M, N, RES, 'grid') returns a
    %   OccupancyGrid object and specifies a grid size of M rows and N columns.
    %   RES specifies the cells per meter resolution.
    %
    %   MAP = robotics.OccupancyGrid(P) creates a probabilistic
    %   occupancy grid object from the values in the matrix,
    %   P. The size of the grid matches the matrix with each cell value
    %   interpreted from that matrix location. Matrix, P, may contain
    %   any numeric type with zeros(0) and ones(1).
    %
    %   MAP = robotics.OccupancyGrid(P, RES) creates a OccupancyGrid
    %   object from matrix, P, with RES specified in cells per meter.
    %
    %   OccupancyGrid properties:
    %       GridSize            - Size of the grid in [rows, cols] (number of cells)
    %       Resolution          - Grid resolution in cells per meter
    %       XWorldLimits        - Minimum and maximum values of X
    %       YWorldLimits        - Minimum and maximum values of Y
    %       GridLocationInWorld - Location of grid in world coordinates
    %
    %
    %   OccupancyGrid methods:
    %       setOccupancy    - Set occupancy of a location
    %       getOccupancy    - Get occupancy of a location
    %       grid2world      - Convert grid indices to world coordinates
    %       world2grid      - Convert world coordinates to grid indices
    %       show            - Show grid probabilities in a figure
    %       inflate         - Inflate each occupied grid location
    %       copy            - Create a copy of the object
    %
    %
    %   Example:
    %
    %       % Create a 2m x 2m empty map
    %       map = robotics.OccupancyGrid(2,2);
    %
    %       % Create a 10m x 10m empty map with resolution 20
    %       map = robotics.OccupancyGrid(10, 10, 20);
    %
    %       % Create a map from a matrix with resolution 20
    %       p = eye(100);
    %       map = robotics.OccupancyGrid(p, 20);
    %
    %       % Check occupancy of the world location (0.3, 0.2)
    %       value = getOccupancy(map, [0.3 0.2]);
    %
    %       % Set world position (1.5, 2.1) as occupied
    %       setOccupancy(map, [1.5 2.1], 1);
    %
    %       % Get the grid cell indices for world position (2.5, 2.1)
    %       ij = world2grid(map, [2.5 2.1]);
    %
    %       % Set the grid cell indices to unoccupied
    %       setOccupancy(map, [1 1], 0, 'grid');
    %
    %       % Set the grid cell indices using partial string argument
    %       setOccupancy(map, [1 1], 0, 'g');
    %
    %       % Show probabilistic occupancy grid in Graphics figure
    %       show(map);
    %
    %   See also robotics.PRM, robotics.PurePursuit.
    
    % Small TODOs
    % Delete Hits and Missed parameters (if only logOdds is used)
    % What should be default for creating occGrid for mockup? 0.5
    % (unknown) everywhere? 
    % show(): Update error messages from binaryoccgrid to occgridcommon
    %   Copyright 2014 The MathWorks, Inc.
    
    %% Properties
    % All public for prototype
    properties (Access = public) %Counter for hits and misses.
        Hits
        Misses
        LogOdds
    end
    properties  (Access = public) %(Access = {?robotics.algs.internal.GridAccess})
        %Grid The occupancy grid data
        %   A matrix that stores the values for grid cells.
        Grid
    end
    properties (Access = public)
        OccupiedThreshold = 0.65; %ROS standards from map_server
        FreeThreshold = 0.2;
        
        l0 = 0; %logodds(0.5)
        lFree = -1.3863; % logodds(FreeThreshold)
        lOcc = 0.6190; %logodds(OccupiedThreshold)
    end
    
    %% Pubic Methods
    methods
        %% Constructors
        function obj = OccupancyGrid(varargin)
            %OccupancyGrid Constructor
            
            % Parse input arguments
            [inputs, isMat] = obj.parseConstructorInputs(varargin{:});
            
            obj.Resolution = inputs.Resolution;
            
            % Construct grid from a matrix input
            if isMat
                obj.Grid = double(inputs.Mat);
                obj.GridSize = size(obj.Grid);
                return;
            end
            
            % Construct empty grid from width and height
            if inputs.isGrid
                obj.Grid = zeros(inputs.Rows, inputs.Columns);
            else
                gridsize = ceil([inputs.Height, inputs.Width]*obj.Resolution);
                % Throw a warning if we round off the grid size
                if any(gridsize ~= ([inputs.Height, inputs.Width]*obj.Resolution))
                    warning(message('robotics:robotalgs:occgridcommon:RoundoffWarning'))
                end
                
                obj.Grid = ones(gridsize(1), gridsize(2))*0.5;
            end
            obj.GridSize = size(obj.Grid);
            % Creating hits and misses matrices
                obj.Hits = ones(obj.GridSize)*5;
                obj.Misses = ones(obj.GridSize)*5;
                % Creating log odds matrix
                obj.LogOdds = getLogOdds(obj, obj.Grid);
        end
        
        %% setOccupancy, getOccupancy
        function value = getOccupancy(obj, pos, frame)
            %getOccupancy Get occupancy value for one or more positions
            %   VAL = getOccupancy(MAP, XY) returns an N-by-1 array of
            %   occupancy values for N-by-2 array, XY. Each row of the
            %   array XY corresponds to a point with [X Y] world coordinates.
            %
            %   VAL = getOccupancy(MAP, XY, 'world') returns an N-by-1 array of
            %   occupancy values for N-by-2 array XY in world coordinates.
            %   This is the default value.
            %
            %   VAL = getOccupancy(MAP, IJ, 'grid') returns an N-by-1
            %   array of occupancy values for N-by-2 array IJ. Each row of
            %   the array IJ refers to a grid cell index [X,Y].
            %
            %   Example:
            %       % Create a probabilistic occupancy grid and get occupancy
            %       % values for a position
            %       map = robotics.OccupancyGrid(10, 10);
            %
            %       % Get occupancy of the world coordinate (0, 0)
            %       value = getOccupancy(map, [0 0]);
            %
            %       % Get occupancy of multiple coordinates
            %       [X, Y] = meshgrid(0:0.5:5);
            %       values = getOccupancy(map, [X(:) Y(:)]);
            %
            %       % Get occupancy of the grid cell (1, 1)
            %       value = getOccupancy(map, [1 1], 'grid');
            %
            %       % Get occupancy of multiple grid cells
            %       [I, J] = meshgrid(1:5);
            %       values = getOccupancy(map, [I(:) J(:)], 'grid');
            %
            %   See also robotics.BinaryOccupancyGrid, setOccupancy
            
            
            isGrid = false;
            % If optional argument present then parse it separately
            if nargin > 2
                isGrid = obj.parseOptionalFrameInput(frame);
            end
            
            % Validate position or subscripts and convert it to indices
            indices = obj.getIndices(pos, isGrid);
            value = obj.Grid(indices);
        end
        
        function setOccupancy(obj, pos, value, frame)
            %setOccupancy Set occupancy value for one or more positions
            %   setOccupancy(MAP, XY, VAL) assigns the scalar occupancy
            %   value, VAL to each coordinate specified in the N-by-2 array,
            %   XY. Each row of the array XY corresponds to a point with
            %   [X Y] world coordinates.
            %
            %   setOccupancy(MAP, XY, VAL) assigns each element of the
            %   N-by-1 vector, VAL to the coordinate position of the
            %   corresponding row of the N-by-2 array, XY.
            %
            %   setOccupancy(MAP, XY, VAL, 'world') specifies the N-by-2
            %   array XY as world coordinates. This is also the default
            %   value.
            %
            %   setOccupancy(MAP, IJ, VAL, 'grid') assigns occupancy values
            %   to the grid positions specified by each row of the N-by-2
            %   array, IJ, which refers to the [row, col] index from each row
            %   in the array.
            %
            %   Example:
            %       % Create a probabilistic occupancy grid and set occupancy
            %       % values for a position
            %       map = robotics.OccupancyGrid(10, 10);
            %
            %       % Set occupancy of the world coordinate (0, 0)
            %       setOccupancy(map, [0 0], 1);
            %
            %       % Set occupancy of multiple coordinates
            %       [X, Y] = meshgrid(0:0.5:5);
            %       values = ones(numel(X),1);
            %       setOccupancy(map, [X(:) Y(:)], values);
            %
            %       % Set occupancy of the grid cell (1, 1)
            %       setOccupancy(map, [1 1], 1, 'grid');
            %
            %       % Set occupancy of multiple grid cells
            %       [I, J] = meshgrid(1:5);
            %       setOccupancy(map, [I(:) J(:)], 1, 'grid');
            %
            %       % Set occupancy of multiple grid cells
            %       [I, J] = meshgrid(1:5);
            %       values = ones(numel(I),1);
            %       setOccupancy(map, [I(:) J(:)], values, 'grid');
            %
            %   See also robotics.BinaryOccupancyGrid, getOccupancy
            
            
            isGrid = false;
            % If optional argument present then parse it separately
            if nargin > 3
                isGrid = obj.parseOptionalFrameInput(frame);
            end
            
            % Validate values
            obj.validateOccupancyValues(value, size(pos, 1), 'VAL');
            
            % Validate position or subscripts and convert it to indices
            index = obj.getIndices(pos, isGrid);
            obj.Grid(index) = double(value(:));
        end
        
        %% inflate
        function inflate(obj, inflationRad, frame)
            %inflate Inflate the occupied positions by a given amount
            %   inflate(MAP, R) inflates each occupied position of the probabilistic
            %   occupancy grid by at least R meters. Each cell of the probabilistic
            %   occupancy grid is inflated by number of cells which is the
            %   closest integer higher than the value MAP.Resolution*R.
            %
            %   inflate(MAP, R, 'grid') inflates each cell of the probabilistic
            %   occupancy grid by R cells.
            %
            %   Note that the inflate function does not inflate the
            %   positions past the limits of the grid.
            %
            %   Example:
            %       % Create a probabilistic occupancy grid and inflate map
            %       bmat = eye(100);
            %       map = robotics.OccupancyGrid(bmat);
            %
            %       % Create a copy of the map for inflation
            %       cpMap = copy(map);
            %
            %       % Inflate occupied cells using inflation radius in
            %       % meters
            %       inflate(cpMap, 0.1);
            %
            %       % Inflate occupied cells using inflation radius in
            %       % number of cells
            %       inflate(cpMap, 2, 'grid');
            %
            %   See also robotics.BinaryOccupancyGrid, copy
            
            isGrid = false;
            % If optional argument present then parse it separately
            if nargin > 2
                isGrid = obj.parseOptionalFrameInput(frame);
            end
            
            % Validate inflation radius and conversion to grid
            radius = obj.validateInflationRadius(inflationRad, obj.Resolution, ...
                isGrid, obj.GridSize, 'R');
            
            se = robotics.algs.internal.diskstrel(radius);
            obj.Grid = robotics.algs.internal.impl.probinflate(obj.Grid, se);
        end
        
        %% show POG
        function imageHandle = show(obj, frame, varargin)
            %show Display the probabilistic occupancy grid in a figure
            %   show(MAP) displays the MAP probabilistic occupancy grid in the
            %   current axes with the axes labels representing the world
            %   coordinates.
            %
            %   show(MAP, 'world') displays the MAP with the axes labels
            %   representing the world coordinates of the MAP. This is the
            %   default.
            %
            %   show(MAP, 'grid') displays the MAP probabilistic occupancy grid in
            %   the current axes with the axes of the figure representing
            %   the grid indices.
            %
            %   HIMAGE = show(MAP, ___) returns the handle to the image
            %   object created by show.
            %
            %   show(MAP,___,Name,Value) provides additional options specified
            %   by one or more Name,Value pair arguments. Name must appear
            %   inside single quotes (''). You can specify several name-value 
            %   pair arguments in any order as Name1,Value1,...,NameN,ValueN:
            %
            %       'Parent'        - Handle of an axes that specifies
            %                         the parent of the image object
            %                         created by show.
            %
            %   Example:
            %       % Create a probabilistic occupancy grid and display
            %       map = robotics.OccupancyGrid(eye(5));
            %
            %       % Display the occupancy with axes showing the world
            %       % coordinates
            %       gh = show(map);
            %
            %       % Display the occupancy with axes showing the grid
            %       % indices
            %       gh = show(map, 'grid');
            %
            %       % Display the occupancy with axes showing the world
            %       % coordinates and specify a parent axes
            %       fh = figure;
            %       ah = axes('Parent', fh);
            %       gh = show(map, 'world', 'Parent', ah);
            %
            %   See also robotics.BinaryOccupancyGrid
            
            
            % Parse inputs
            isGrid = false;
            if nargin > 1
                isGrid = obj.parseOptionalFrameInput(frame);
            end
            
            axHandle = obj.showInputParser(varargin{:});
            
            % If axes not given, create an axes
            % The newplot function does the right thing
            if isempty(axHandle)
                axHandle = newplot;
            end
            
            % Use gray colormap for correct visualization
            cmap = colormap('gray');
            % Flip color map to make unoccupied (free) cells white
            cmap = flip(cmap);
            
            % Display the grid map
            imghandle = imshow(obj.Grid, 'Parent', axHandle, ...
                'InitialMagnification', 'fit');
%             imghandle = imagesc(obj.Grid);
            colormap(cmap)
            
            
            title(axHandle, ...
                message('robotics:robotalgs:binaryoccgrid:FigureTitle').getString);
            
            % Change the axes limits, X data and Y data to show world
            % coordinates or grid indices on the figure
            if isGrid
                % Get the grid size
                xgdata = [1, obj.GridSize(2)];
                ygdata = [1, obj.GridSize(1)];
                
                % Set XData and YData
                imghandle.XData = xgdata;
                imghandle.YData = ygdata;
                
                % Compute the grid limits
                xlimits = [0.5, obj.GridSize(2)+0.5];
                ylimits = [0.5, obj.GridSize(1)+0.5];
                
                xlabel(axHandle, ...
                    message('robotics:robotalgs:occgridcommon:FigureColLabel').getString);
                ylabel(axHandle, ...
                    message('robotics:robotalgs:occgridcommon:FigureRowLabel').getString);
                
                % Set the axes
                set(axHandle, 'YDir','reverse');
                
            else
                % Get the world limits
                xlimits = obj.XWorldLimits;
                ylimits = obj.YWorldLimits;
                
                correction = 1/(2*obj.Resolution);
                
                % Set XData and YData
                if (abs(xlimits(1)-xlimits(2)+2*correction) < eps)
                    % Special case when there is only one cell
                    imghandle.XData = [xlimits(1), xlimits(2)];
                else
                    imghandle.XData = [xlimits(1)+correction, xlimits(2)-correction];
                end
                
                if (abs(ylimits(1)-ylimits(2)+2*correction) < eps)
                    imghandle.YData = [ylimits(2), ylimits(1)];
                else
                    imghandle.YData = [ylimits(2)-correction, ylimits(1)+correction];
                end
                
                xlabel(axHandle, ...
                    message('robotics:robotalgs:binaryoccgrid:FigureXLabel').getString);
                ylabel(axHandle, ...
                    message('robotics:robotalgs:binaryoccgrid:FigureYLabel').getString);
                
                % Set the axes
                set(axHandle, 'YDir','normal');
            end
            
            grid(axHandle, 'off');
            
            % Set XLim and YLim
            axHandle.XLim = xlimits;
            axHandle.YLim = ylimits;
            
            % Make axes visible
            axHandle.Visible = 'on';
            
            % Only return handle if user requested it.
            if nargout > 0
                imageHandle = imghandle;
            end
        end
        
        %% set free and occupied thresholds
        function set.OccupiedThreshold(obj,threshold)
                        validateattributes(threshold, {'numeric'}, ...
                    {'>=',obj.FreeThreshold,'<=',1, 'scalar'}, 'OccupancyGrid', 'FreeThreshold');
            obj.OccupiedThreshold = threshold;
        end
        
        function set.FreeThreshold(obj,threshold)
            validateattributes(threshold, {'numeric'}, ...
                    {'>=',0,'<=',obj.OccupiedThreshold, 'scalar'}, 'OccupancyGrid', 'FreeThreshold');
            obj.FreeThreshold = threshold;
        end
       
        %% isOccupied, isFree, isUnknown
        function occupied = isOccupied(obj, pos, frame)
            try
            pOccupancy = getOccupancy(obj,pos,frame);
            catch exception
                throw(exception)
            end
            occupied = (pOccupancy >= obj.OccupiedThreshold); 
        end
        
        function occupied = isFree(obj, pos, frame)
            try
            pOccupancy = getOccupancy(obj,pos,frame);
            catch exception
                throw(exception)
            end
            occupied = (pOccupancy <= obj.FreeThreshold); 
        end
        
        function occupied = isUnknown(obj, pos, frame)
            try
            pOccupancy = getOccupancy(obj,pos,frame);
            catch exception
                throw(exception)
            end
            occupied = (pOccupancy < obj.OccupiedThreshold && pOccupancy > obj.FreeThreshold);
        end
        
        %% Hit & Miss: addHit, addMiss
        function addHit(obj,pos,frame)
            
            isGrid = false; %By default it's world coordinates, not grid
            % If optional argument present then parse it separately
            if nargin > 2
                isGrid = obj.parseOptionalFrameInput(frame);
            end
            
            % Validate position or subscripts and convert it to indices
            indices = obj.getIndices(pos, isGrid);
            obj.Hits(indices) = obj.Hits(indices) + 1;
            
            %Update pOccupied (./ is for multiple indictes)
            obj.Grid(indices) = obj.Hits(indices) ./ ...
                (obj.Hits(indices) + obj.Misses(indices));
        end
        
        function addMiss(obj,pos,frame)
            
            isGrid = false; %By default it's world coordinates, not grid
            % If optional argument present then parse it separately
            if nargin > 2
                isGrid = obj.parseOptionalFrameInput(frame);
            end
            
            % Validate position or subscripts and convert it to indices
            indices = obj.getIndices(pos, isGrid);
            obj.Misses(indices) = obj.Misses(indices) + 1;
            
            %Update pOccupied (./ is for multiple indictes)
            obj.Grid(indices) = obj.Hits(indices) ./ ...
                (obj.Hits(indices) + obj.Misses(indices));
        end
        
        %% LogOdds: getLogOdds, getProbfromLogOdds, updateLogOdds
        function logodds = getLogOdds(obj, prob)
             validateattributes(prob, {'numeric'}, ...
                {'finite', '<=', 1, '>=' 0}, '');
            
            logodds = log(prob./(1-prob));
        end
        
        function probability = getProbFromLogOdds(obj, logodds)
            validateattributes(logodds, {'numeric'}, {}, '');
            
            probability = 1 - 1./(1 + exp(logodds));
        end
        
        function updateLogOdds(obj, pos, prob_sens, frame)
            isGrid = false; %By default it's world coordinates
            
            % If optional argument present then parse it separately
            if nargin > 3
                isGrid = obj.parseOptionalFrameInput(frame);
            end
            
            % Validate position or subscripts and convert it to indices
            indices = obj.getIndices(pos, isGrid);
            
            LO_inv_sens = getLogOdds(obj,prob_sens);
            obj.LogOdds(indices) = obj.LogOdds(indices) + LO_inv_sens - obj.l0;
            obj.Grid(indices) = getProbFromLogOdds(obj,obj.LogOdds(indices));
        end
        
        %% Beam Model?
        function out = simulateRangeReading(obj,pose) % get simulated reading (and compare it with actual reading, for localization pf)
            out = 1;
            disp('Placeholder function for now');
        end
        
        %% Convert to ternary/BOG
        function ternary = applyOccupancyThresholds(obj)
            ternary = ones(size(obj.Grid))*0.5;
            occupied = (obj.Grid > obj.OccupiedThreshold);
            free = (obj.Grid < obj.FreeThreshold);
            ternary(occupied) = 1;
            ternary(free) = 0;
        end
        
    end
    
    %% Internal methods
    %================================================================
    methods(Access = private)
        function [inputs, isMat] = ...
                parseConstructorInputs(obj, firstarg, varargin)
            %parseConstructorInputs Input parser for constructor
            
            p = inputParser;
            defaultResolution = 1;
            defaultCoordFrame = 'world';
            validCoordFrames = {'world','grid'};
            
            % Parsing following input variations
            % OccupancyGrid(P)
            % OccupancyGrid(P, RES)
            % OccupancyGrid(W, H)
            % OccupancyGrid(W, H, RES)
            % OccupancyGrid(W, H, RES, 'world')
            % OccupancyGrid(M, N, RES, 'grid')
            
            % Differentiate required inputs based on the first argument
            % If first input is matrix or a scalar
            if isnumeric(firstarg) && isscalar(firstarg) && ~isempty(varargin)
                isMat = false;
            else
                isMat = true;
            end
            
            % Parse optional inputs using input parser
            if isMat
                addOptional(p,'Resolution',defaultResolution, ...
                    @(x)validateattributes(x, {'numeric'}, ...
                    {'scalar', 'positive','finite'}));
                parse(p, varargin{:});
            else
                addOptional(p,'Resolution',defaultResolution, ...
                    @(x)validateattributes(x, {'numeric'}, ...
                    {'scalar', 'positive', 'finite'}));
                addOptional(p,'CoordinateFrame', defaultCoordFrame, ...
                    @(x)any(validatestring(x,validCoordFrames)));
                parse(p,varargin{2:end});
                inputs.isGrid = ...
                    obj.parseOptionalFrameInput(p.Results.CoordinateFrame);
            end
            inputs.Resolution = double(p.Results.Resolution);
            
            % Validate required inputs
            if isMat  % If matrix is an input
                % Gautam: separate validate for logical as it doesn't
                % support >=0 and <=1
                if(islogical(firstarg))
                    validateattributes(firstarg,{'logical'}, ...
                    {'2d', 'nonempty'},'OccupancyGrid', 'P', 1);
                else
                    validateattributes(firstarg,{'numeric'},...
                        {'2d','nonempty','>=',0,'<=',1},'OccupancyGrid','P',1);
                end
                inputs.Mat = firstarg;
                
            elseif inputs.isGrid % Two scalar inputs with 'grid' option
                validateattributes(firstarg, {'numeric'}, ...
                    {}, 'OccupancyGrid', 'M', 1);
                validateattributes(varargin{1}, {'numeric'}, ...
                    {}, 'OccupancyGrid', 'N', 2);
                inputs.Rows = firstarg;
                inputs.Columns = varargin{1};
                
            else % Two scalar inputs with 'world' option which is also default
                validateattributes(firstarg, {'numeric'}, ...
                    {'finite','positive','scalar'}, 'OccupancyGrid', 'W', 1);
                validateattributes(varargin{1}, {'numeric'}, ...
                    {'finite','positive','scalar'}, 'OccupancyGrid', 'H', 2);
                inputs.Width = firstarg;
                inputs.Height = varargin{1};
            end
        end
        
        function isGrid = parseOptionalFrameInput(~, cframe)
            %parseOptionalFrameInput Input parser for optional string
            
            % Error if not a char input
            if ~ischar(cframe)
                error(message('robotics:robotalgs:occgridcommon:InvalidCoordType', ...
                    class(cframe)));
            end
            
            % If char then compare with expected inputs
            n = max(1,numel(cframe));
            if strncmpi(cframe,'grid', n)
                isGrid = true;
            elseif strncmpi(cframe,'world', n)
                isGrid = false;
            else
                error(message('robotics:robotalgs:occgridcommon:InvalidCoordString', ...
                    cframe));
            end
        end
        
        function index = getIndices(obj, pos, isGrid)
            %getIndices Validate and return grid indices from positions
            
            if isGrid               % Use grid indices
                obj.validateGridIndices(pos, obj.GridSize, 'IJ');
            else                    % Use world coordinates
                obj.validatePosition(pos, obj.XWorldLimits, obj.YWorldLimits, 'XY');
                % Convert to grid indices
                pos = obj.worldToGridPrivate(pos);
            end
            % X-Y is flipped because grid is row-major representation
            % index = sub2ind(size(obj.Grid), pos(:,2), pos(:,1));
            index = sub2ind(obj.GridSize, pos(:,1), pos(:,2));
        end
        
        function axHandle = showInputParser(~, varargin)
            %showInputParser Input parser for show function
            
            p = inputParser;
            p.addParameter('Parent', [], ...
                @(x)robotics.internal.validation.validateAxesHandle(x));
            
            p.parse(varargin{:});
            res = p.Results;
            axHandle = res.Parent;
        end
        
    end
    
    %================================================================
    methods (Static, Access = private)
        function radius = validateInflationRadius(inflationRad, ...
                resolution, isgrid, gridsize, name)
            %validateInflationRadius Validate inflation radius
            
            % Convert radius from meters to number of cells
            if isgrid && isnumeric(inflationRad)
                radius = inflationRad;
            else
                radius = ceil(inflationRad*resolution);
            end
            
            validateattributes(radius, {'numeric'}, ...
                {'scalar','positive','integer','finite', '<=', min(gridsize)}, ...
                'inflate', name);
        end
        
        function validateOccupancyValues(values, len, name)
            %validateOccupancyValues Validate occupancy value vector
            
            % check that the values are numbers in [0,1]
            validateattributes(values, {'logical','numeric'}, ...
                {'real','vector','<=',1,'>=',0}, 'setOccupancy', name);
            
            if length(values) ~= 1 && length(values) ~= len
                error(message('robotics:robotalgs:occgridcommon:InputSizeMismatch'));
            end
        end
    end
end