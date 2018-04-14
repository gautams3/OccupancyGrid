classdef (Hidden, Abstract) OccupancyGridBase < handle & matlab.mixin.Copyable
    %This class is for internal use only. It may be removed in the future.
    
    %OccupancyGridBase Define the OccupancyGridBase class.
    
    %   Copyright 2014 The MathWorks, Inc.
    
    properties (SetAccess = protected)
        %GridSize Size of the grid
        %   A vector [ROWS COLS] indicating the size of the grid as number of
        %   rows and columns.
        GridSize
        
        %Resolution Grid resolution in cells per meter
        %
        %   Default: 1
        Resolution
    end
    
    properties (Dependent, SetAccess = protected)
        %XWorldLimits Minimum and maximum values of X
        %   A vector [MIN MAX] representing the world limits of the grid
        %   along the X axis.
        XWorldLimits
        
        %YWorldLimits Minimum and maximum values of Y
        %   A vector [MIN MAX] representing the world limits of the grid
        %   along the Y axis.
        YWorldLimits
    end
    
    properties
        %GridLocationInWorld - Location of the grid in world coordinates
        %   A vector defining the [X, Y] world coordinates of the bottom-left
        %   corner of the grid.
        %   Default: [0 0]
        GridLocationInWorld = [0 0]
    end
    
    methods
        function xlims = get.XWorldLimits(obj)
            %get.XWorldLimits Getter for XWorldLimits property
            
            xlims(1,1) = obj.GridLocationInWorld(1);
            % Grid is in column-major format hence using 2nd dimension
            xlims(1,2) = obj.GridLocationInWorld(1) + ...
                obj.GridSize(1,2)/obj.Resolution;
        end
        
        function ylims = get.YWorldLimits(obj)
            %get.YWorldLimits Getter for YWorldLimits property
            
            ylims(1,1) = obj.GridLocationInWorld(2);
            % Grid is in column-major format hence using 1st dimension
            ylims(1,2) = obj.GridLocationInWorld(2) + ...
                obj.GridSize(1,1)/obj.Resolution;
        end
        
        function set.GridLocationInWorld(obj, loc)
            %set.GridLocationInWorld Setter for GridLocationInWorld property
            obj.validateGridLocationInput(loc, 'GridLocationInWorld');
            obj.GridLocationInWorld = loc;
        end
        
        function idx = world2grid(obj, pos)
            %world2grid Convert world coordinates to grid indices
            %   IJ = world2grid(MAP, XY) converts an N-by-2 array of world
            %   coordinates, XY, to an N-by-2 array of grid indices, IJ. The
            %   input, XY, is in [X Y] format. The output grid indices, IJ,
            %   are in [ROW COL] format.
            %
            %   Example:
            %       % Create an occupancy grid and convert world
            %       % coordinates to grid indices
            %       % Create a 10m x 10m world representation, e.g.
            %       map = robotics.BinaryOccupancyGrid(10, 10);
            %
            %       % Get grid indices from world coordinates
            %       ij = world2grid(map, [0 0])
            %
            %       % Get grid indices from world coordinates
            %       [x y] = meshgrid(0:0.5:2);
            %       ij = world2grid(map, [x(:) y(:)])
            
            obj.validatePosition(pos, obj.XWorldLimits, obj.YWorldLimits, 'XY');
            
            % Convert world coordinate to grid indices
            idx = worldToGridPrivate(obj, pos);
        end
        
        function pos = grid2world(obj, idx)
            %grid2world Convert grid indices to world coordinates
            %   XY = grid2world(MAP, IJ) converts an N-by-2 array of grid
            %   indices, IJ, to an N-by-2 array of world coordinates, XY. The
            %   input grid indices, IJ, are in [ROW COL] format. The output,
            %   XY, is in [X Y] format.
            %
            %   Example:
            %       % Create an occupancy grid and convert grid
            %       % indices to world coordinates
            %       % Create a 10m x 10m world representation
            %       map = robotics.BinaryOccupancyGrid(10, 10);
            %
            %       % Get world coordinates from grid indices
            %       xy = grid2world(map, [1 1])
            %
            %       % Get world coordinates from grid indices
            %       [i j] = meshgrid(1:5);
            %       xy = world2grid(map, [i(:) j(:)])
            
            obj.validateGridIndices(idx, obj.GridSize, 'IJ');
            
            % Convert grid index to world coordinate
            pos = gridToWorldPrivate(obj, idx);
        end
        
    end
    
    methods(Access = protected)
        function xy = gridToWorldPrivate(obj, ij)
            %gridToWorldPrivate Convert grid indices to world coordinates
            
            
            % ROW-COL is YX, so convert it to XY
            ji = flip(ij, 2);
            
            % Cell index can only increment by 1
            halfCell = 0.5;
            
            % Y-axis starts at top, reverse it to start at bottom
            ji(:,2) = obj.GridSize(1)+1 - ji(:,2);
            
            % Compute the transform to converts index to world coordinate
            tform = ([halfCell, halfCell])/obj.Resolution - ...
                obj.GridLocationInWorld;
            
            % Apply transform
            xy = ji/obj.Resolution - repmat(tform,size(ji,1),1);
        end
        
        function ij = worldToGridPrivate(obj, xy)
            %worldToGridPrivate Convert world coordinates to grid indices
            
            % Compute all cells indices
            translatedPos = (xy - repmat(obj.GridLocationInWorld,size(xy,1),1));
            
            % Convert to ROW-COL which is YX
            translatedPosYX = flip(translatedPos, 2);
            
            ij = ceil(translatedPosYX*obj.Resolution);
            
            % GridLocation is always cell [1 1]
            originIdx = abs(translatedPosYX) < eps;
            ij(originIdx) = 1;
            
            % Adjust grid indices to start at top
            ij(:,1) = obj.GridSize(1)+1 - ij(:,1);
        end
    end
    
    
    %================================================================
    methods (Static, Access = protected)
        function validatePosition(pos, xlimits, ylimits, name)
            %validatePosition Validate the world position column matrix
            
            % Validate the input format and type
            validateattributes(pos, {'double'}, ...
                {'real', 'finite', '2d', 'ncols', 2}, 'OccupancyGridBase', name);
            
            % Validate if the input is within the world limits
            maxpos = max(pos, [], 1);
            minpos = min(pos, [], 1);
            if (minpos(1,1) < xlimits(1,1) || minpos(2) < ylimits(1,1) ...
                    || maxpos(1,1) > xlimits(1,2) || maxpos(2) > ylimits(1,2));
                error(message('robotics:robotalgs:binaryoccgrid:CoordinateOutside', ...
                    num2str(xlimits(1,1),'%.2f'), num2str(xlimits(1,2),'%.2f'), ...
                    num2str(ylimits(1,1),'%.2f'), num2str(ylimits(1,2),'%.2f')));
            end
        end
        
        function validateGridIndices(pos, gridsize, name)
            %validateGridIndices Validate the grid indices column matrix
            
            % Validate the input format and type
            validateattributes(pos,{'double'}, ...
                {'integer', 'positive', '2d', 'ncols', 2}, ...
                'OccupancyGridBase', name)
            
            % Validate if the indices are within the limits
            maxpos = max(pos, [], 1);
            if (maxpos(1) > gridsize(1,1) || ...
                    maxpos(2) > gridsize(1,2));
                error(message('robotics:robotalgs:binaryoccgrid:IndexExceedsDim', ...
                    num2str(gridsize(1,1)), num2str(gridsize(1,2))));
            end
        end
        
        function validateGridLocationInput(loc, name)
            % Validate the input format and type
            validateattributes(loc,{'double'}, ...
                {'real', 'finite', 'size', [1 2]}, ...
                'OccupancyGridBase', name)
        end
    end
end