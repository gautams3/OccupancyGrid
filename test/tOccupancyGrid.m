classdef tOccupancyGrid < robotics_tests.DefaultSaveLoadCopy
    %tOccupancyGrid Test for the probabilistic occupancy grid class
    
    %   Copyright 2014 The MathWorks, Inc.
    
    properties (Constant)
        %DefaultGridLocation - Default value of grid location property
        DefaultGridLocation = [0 0];
        
        %DefaultResolution - Default value of resolution property
        DefaultResolution = 1;
        
        %DefaultCoordFrame - Default value of optional frame argument
        DefaultCoordFrame = 'world';
    end
    
    properties (TestParameter)
        %optionalArgW Parametrized tests for partial matching
        optionalArgW = {'w', 'World', 'wor', 'world'};
        
        %optionalArgG Parametrized tests for partial matching
        optionalArgG = {'g', 'Grid', 'gri', 'grid'};
        
        %matFilename Parametrized tests for different matrix data inputs
        matFilename = {'testMatDoubleBinary', 'testMatLogical', 'testMatUint8','testMatDoubleProbabalistic'};
        
        %showTestGridLocation Parametrized test for show method
        showTestGridLocation = {[0 0], [0 0], [-9 -9], [-9 -9]};
        
        %showTestResolution Parametrized test for show method
        showTestResolution = {0.1, 0.1, 1, 2};
        
        %showTestSize Parametrized test for show method
        showTestSize = {[10 100], [100 10], [5 5], [5 5]};
        
        %showTestSize Parametrized test for show method
        %   These are known values that produce right plot output
        showTestExpectedXData = {[0 10], [5 95], [-8.5 -4.5], [-8.75 -4.25]};
        
        %showTestSize Parametrized test for show method
        %   These are known values that produce right plot output
        showTestExpectedYData = {[95 5], [10 0], [-4.5 -8.5], [-4.25 -8.75]};
    end
    
    properties (Constant, Access = private)
        %DataDir - Directory where data files for tests are stored
        DataDir = fullfile(fileparts(mfilename('fullpath')), '..', 'data')
        
        %StrelBaseline - File name for baseline data
        %   The implementation of the diskstrel function is different than
        %   the "strel" function in IPT and hence there is no one-to-one
        %   comparison. The disk created using the "diskstrel" function
        %   should cover the entire circle with some integer radius R and
        %   any cell that is inside or touching this circle should be set
        %   to value 1. All other cells in the matrix are set to 0. This
        %   can only be verified interactively. A one-time interactive
        %   validation can be done using "hGenerateBaselineForDiskstrel.m"
        %   function. The data is stored in "dDiskstrelBaseline.mat" file.
        StrelBaseline = 'dDiskstrelBaseline.mat'
        
        %InflateBaseline - File name for baseline data
        %   Inflate function is exactly same as the "imdilate" function in
        %   IPT. In order to remove test dependency on IPT, the baseline is
        %   stored in a mat file. The baseline is generated as follows,
        %
        %   se = robotics.algs.internal.diskstrel(radius);
        %   expectedImage = imdilate(testImage, se);
        %
        %   The function "diskstrel" is tested separately. Following
        %   baseline includes radius 1, 2, 3 and 6 as four randomly picked
        %   values.
        InflateBaseline = 'dInflateBaseline.mat'
    end
    
    methods (Test, ParameterCombination='pairwise')
        %%
        function defaultConstructorTest(testCase)
            %defaultConstructorTest Test the default constructor
            
            map = robotics.OccupancyGrid(1, 1);
            
            % expectedGridSize = [ROW COL]
            expectedGridSize = [1, 1];
            
            % Verify all public properties
            testCase.verifyEqual(map.GridLocationInWorld, testCase.DefaultGridLocation);
            testCase.verifyEqual(map.Resolution, testCase.DefaultResolution);
            testCase.verifyEqual(map.GridSize, expectedGridSize);
            testCase.verifyEqual(map.XWorldLimits, [0 1]);
            testCase.verifyEqual(map.YWorldLimits, [0 1]);
        end
        
        function floatSizeConstructorTest(testCase)
            %floatSizeConstructorTest Test floating point size input
            
            testWidth = 10.25;
            testHeight = 20.14;
            testResolutions = [1 5 10 12.13];
            
            for i = 1:size(testResolutions,1)
                % Test default constructor
                
                testCase.verifyWarning(@()robotics.OccupancyGrid(testWidth, ...
                    testHeight, testResolutions(i)), ...
                    'robotics:robotalgs:occgridcommon:RoundoffWarning');
            end
            
%             function constructBOGObject(testWidth,testHeight, testResolutions)
%                 %constructBOGObject Subfunction to call by handle
%                 robotics.OccupancyGrid(testWidth, ...
%                     testHeight, testResolutions);
%             end
        end
        
        function optArgResConstructorTest(testCase)
            %optArgResConstructorTest Test resolution optional input
            
            testResolutions = [1, 5, 10]; % cells per meter
            
            for i = 1:length(testResolutions)
                % Test default constructor
                [tWidth, tHeight, map] = constructTestOGObj(testCase, ...
                    testResolutions(i));
                
                % expectedGridSize = [ROW COL]
                expectedGridSize = [tHeight, tWidth]*testResolutions(i);
                
                % Verify grid size
                testCase.verifyEqual(map.GridSize, expectedGridSize);
                testCase.verifyEqual(map.Resolution, testResolutions(i));
            end
        end
        
        function optArgFrameConstructorTest(testCase)
            %optArgFrameConstructorTest Test constructor with optional arg
            
            testCols = 10;
            testRows = 20;
            testCoordFrame = 'grid';
            
            % Grid requires resolution input
            testResolutions = [1, 5, 10]; % cells per meter
            
            expectedGridSize = [testRows, testCols];
            
            for i = 1:length(testResolutions)
                % Test default constructor
                map = robotics.OccupancyGrid(testRows, ...
                    testCols, testResolutions(i), testCoordFrame);
                
                % Verify grid size
                testCase.verifyEqual(map.GridSize, expectedGridSize);
                testCase.verifyEqual(map.Resolution, testResolutions(i));
            end
        end
        
        %%
        function matrixConstructorTest(testCase, matFilename)
            %matrixConstructorTest Test constructor for matrix inputs
            
            % 'matFilename' files are three identity matrices of types
            % 'uint8', 'double' and 'logical' (Gautam: add generic 'double'
            % matrix here?)
            % Matrix name is 'testMat' for all
            tmat = load(fullfile(testCase.DataDir, matFilename));
            
            % Expected grid size
            expectedSize = [size(tmat.testMat,1), size(tmat.testMat,2)];
            
            map = robotics.OccupancyGrid(tmat.testMat);
            
            testCase.verifyEqual(map.GridSize, expectedSize);
            
            % Test with resolution
            testRes = [1 5 10 13];
            
            % Resolution should not change the grid size for matrix input
            for i = 1:length(testRes)
                map = robotics.OccupancyGrid(tmat.testMat, testRes(i));
                testCase.verifyEqual(map.GridSize, expectedSize);
            end
        end
        
        function constructorNeg(testCase)
            %constructorNeg Test invalid inputs to constructor
            
            testInputs = {{}, ...
                {{1}}, ...
                {1, zeros(10)}, ...
                {zeros(10), 1,1}, ...
                {[10;10]}, ...
                {[-1;-1]}, ...
                {10+2i, 1}, ...
                {10,10,'g',1}, ...
                {zeros(10), 'w', 1}, ...
                {zeros(10), 1, 'g'}, ...
                {zeros(10), 't'}, ...
                {zeros(10,10,3)}};
            
            errorMsgs = {'MATLAB:minrhs', ...
                'MATLAB:OccupancyGrid:invalidType', ...
                'MATLAB:OccupancyGrid:expectedPositive', ...
                'MATLAB:InputParser:ParamMustBeChar', ...
                'MATLAB:OccupancyGrid:notLessEqual', ...
                'MATLAB:OccupancyGrid:notGreaterEqual', ...
                'MATLAB:OccupancyGrid:expectedPositive', ...
                'MATLAB:invalidType', ...
                'MATLAB:invalidType', ...
                'MATLAB:InputParser:ParamMissingValue', ...
                'MATLAB:invalidType',...
                'MATLAB:OccupancyGrid:expected2D'};
            
            for k = 1:length(errorMsgs)
                testCase.verifyError(@()robotics.OccupancyGrid(testInputs{k}{:}),...
                    errorMsgs{k});
            end
        end
        
        %%
        function setOccupiedWorldTest(testCase)
            %setOccupiedWorldTest Test setOccupancy method for world
            
            [~, ~, map] = constructTestOGObj(testCase, []);
            
            % Verify setOccupancy in world coordinate frame
            xlim = map.XWorldLimits;
            ylim = map.YWorldLimits;
            
            % Setting double , logical scalar
            testValues = {0, 0.25, 0.5, 0.75, 1};
            expectedValues = testValues;
            
            for i = 1:length(testValues)
                setOccupancy(map,[xlim(1),ylim(1)], testValues{i});
                testCase.verifyEqual(getOccupancy(map,[xlim(1),ylim(1)]), ...
                    expectedValues{i});
            end
            
            % Setting vector values
            xCoord = linspace(xlim(1),xlim(2), 10);
            yCoord = linspace(ylim(1),ylim(2), 10);
            expectedValues = 0.1:0.1:1; %[0.1, 0.2, ..., 1]
            setOccupancy(map,[xCoord(:),yCoord(:)], expectedValues);
            testCase.verifyEqual(getOccupancy(map,[xCoord(:),yCoord(:)]),...
                expectedValues');
        end
        
        function setOccupiedGridTest(testCase)
            %setOccupiedGridTest Test setOccupancy method for grid
            
            [~, ~, map] = constructTestOGObj(testCase, []);
            
            % Verify setOccupancy in grid coordinate frame
            gridsize = map.GridSize;
            
            differentFrameInputs = {'g', 'G', 'Grid', 'grid'};
            
            for i = 1:length(differentFrameInputs)
                % Setting double values - scalar
                setOccupancy(map,[1,1], 0, differentFrameInputs{i});
                testCase.verifyEqual(getOccupancy(map,[1,1]),0);
            end
            
            % Setting double , logical scalar
            testValues = {0, 0.25, 0.5, 0.75, 1};
            expectedValues = testValues;
            
            for i = 1:length(testValues)
                setOccupancy(map,[gridsize(1), gridsize(2)], ...
                    testValues{i}, 'grid');
                testCase.verifyEqual(getOccupancy(map,[gridsize(1), ...
                    gridsize(2)], 'grid'), expectedValues{i});
            end
        end
        
        function setOccupancyVectorInput(testCase)
            %setOccupancyVectorInput Test setOccupancy with vector inputs
            
            [~, ~, map] = constructTestOGObj(testCase, []);
            rIdx = 1:10;
            cIdx = 1:10;
            expectedValues = 0.1:0.1:1; %[0.1, 0.2, ..., 1]
            setOccupancy(map,[rIdx(:),cIdx(:)], expectedValues, 'grid');
            
            testCase.verifyEqual(getOccupancy(map,[rIdx(:),cIdx(:)],...
                'grid'), expectedValues');
            
            %Setting vector of cells to a scalar value
            [~, ~, map] = constructTestOGObj(testCase, []);
            setOccupancy(map,[rIdx(:),cIdx(:)], 1, 'grid');
            
            %Gautam: This checks if all columns are non-zero. Change to verify equal to 0.8?
            testCase.verifyTrue(all(getOccupancy(map,[rIdx(:),cIdx(:)], 'grid')));
        end
        
        function setOccupancyScalarValueInput(testCase)
            %setOccupancyScalarValueInput Test setOccupancy with scalar value input
            
            [~, ~, map] = constructTestOGObj(testCase, []);
            rIdx = 1:10;
            cIdx = 1:10;
            
            %Setting vector values with scalar value
            setOccupancy(map,[rIdx(:),cIdx(:)], 1, 'grid');
            
            testCase.verifyTrue(all(getOccupancy(map,[rIdx(:),cIdx(:)], 'grid')));
        end
        
        function setOccupancyNeg(testCase)
            %setOccupancyNeg Test invalid inputs to setOccupancy
            
            [~, ~, map] = constructTestOGObj(testCase, []);
            
            testInputs = {{{1, 1}, 1}, ...
                {[0, 0], [1;1]}, ...
                {[22, 1], 1, 'grid'}, ...
                {[22, 12], 1}, ...
                {[1, 1], 1, {2}}, ...
                {[1+2i,1+2i], 1}, ...
                {[1,1], 1+2i}, ...
                {[1, 1],1,'g',1}, ...
                {}, ...
                {[3.2, 1.2],1,'g'}};
            
            errorMsgs = {'MATLAB:OccupancyGridBase:invalidType', ...
                'robotics:robotalgs:occgridcommon:InputSizeMismatch', ...
                'robotics:robotalgs:occgridcommon:IndexExceedsDim', ...
                'robotics:robotalgs:occgridcommon:CoordinateOutside', ...
                'robotics:robotalgs:occgridcommon:InvalidCoordType', ...
                'MATLAB:OccupancyGridBase:expectedReal', ...
                'MATLAB:setOccupancy:expectedReal', ...
                'MATLAB:maxrhs', ...
                'MATLAB:minrhs', ...
                'MATLAB:OccupancyGridBase:expectedInteger'};
            
            % Make expected error message LXE-compliant
            errorMsgs = robotics_tests.LXEConversion.convertErrorID(errorMsgs);
            
            % Invalid inputs
            for k = 1:length(errorMsgs)
                testCase.verifyError(@()setOccupancy(map, testInputs{k}{:}),...
                    errorMsgs{k});
            end
        end
        
        %%
        function getOccupancyWorldTest(testCase)
            %getOccupancyWorldTest Test getOccupancy method for world
            
            % Test data to test "get" method independently of "set" method
            testMat = eye(10);
            
            map = robotics.OccupancyGrid(testMat);
            
            % Verify getOccupancy in world coordinate frame
            
            xlim = map.XWorldLimits;
            ylim = map.YWorldLimits;
            testCase.verifyEqual(getOccupancy(map,[xlim(1),ylim(1)]),0);
            testCase.verifyEqual(getOccupancy(map,[xlim(1),ylim(2)]),1);
            
            % Verify vector input
            
            % World coordinates for the diagonal of the identity matrix
            xCoord = linspace(xlim(1),xlim(2), 10);
            yCoord = linspace(ylim(2),ylim(1), 10);
            
            %Gautam: This checks if all columns are non-zero. Change to verify equal to 0.8?
            testCase.verifyTrue(all(getOccupancy(map,[xCoord(:),yCoord(:)])));
        end
        
        function getOccupancyGridTest(testCase)
            %getOccupancyGridTest Test getOccupancy method for grid
            
            % Test data to test "get" method independent of "set" method
            testMat = eye(10);
            
            map = robotics.OccupancyGrid(testMat);
            
            % Verify scalar output
            gridsize = map.GridSize;
            testCase.verifyEqual(getOccupancy(map,[1,1], 'grid'),1);
            testCase.verifyEqual(getOccupancy(map,[1, gridsize(2)], 'grid'),0);
            
            % Verify vector output
            
            % Indices for the diagonal of the identity matrix
            xIdx = linspace(1,gridsize(1), 10);
            yIdx = linspace(1,gridsize(2), 10);
            
            %Gautam: This checks if all columns are non-zero. Change to verify equal to 0.8?
            testCase.verifyTrue(all(getOccupancy(map,[xIdx(:),yIdx(:)], 'grid')));
        end
        
        function getOccupancyNeg(testCase)
            %getOccupancyNeg Test invalid inputs to getOccupancy
            
            [~, ~, map] = constructTestOGObj(testCase, []);
            
            testInputs = {{{1, 1}}, ...
                {[21, 1], 'grid'}, ...
                {[22, 12]}, ...
                {[1, 1], {2}}, ...
                {[1+2i,1+2i]}, ...
                {[1, 1],'g',1}, ...
                {}, ...
                {[3.2, 1.2], 'g'}};
            
            errorMsgs = {'MATLAB:OccupancyGridBase:invalidType', ...
                'robotics:robotalgs:occgridcommon:IndexExceedsDim', ...
                'robotics:robotalgs:occgridcommon:CoordinateOutside', ...
                'robotics:robotalgs:occgridcommon:InvalidCoordType', ...
                'MATLAB:OccupancyGridBase:expectedReal', ...
                'MATLAB:maxrhs', ...
                'MATLAB:minrhs', ...
                'MATLAB:OccupancyGridBase:expectedInteger'};
            
            % Make expected error message LXE-compliant
            errorMsgs = robotics_tests.LXEConversion.convertErrorID(errorMsgs);
            
            % Invalid inputs
            for k = 1:length(errorMsgs)
                testCase.verifyError(@()getOccupancy(map, testInputs{k}{:}),...
                    errorMsgs{k});
            end
        end
        
        %%
        function xyWorldLimitsTest(testCase)
            %xyWorldLimitsTest Test for XWorldLimits and YWorldLimits
            
            % Constructor with world size as inputs
            tResolution = 20;
            [tWidth, tHeight, map] = constructTestOGObj(testCase, tResolution);
            
            expectedXlimits = [0 tWidth];
            expectedYlimits = [0 tHeight];
            
            % Verify world limits
            testCase.verifyEqual(map.XWorldLimits, expectedXlimits,'RelTol', eps)
            testCase.verifyEqual(map.YWorldLimits, expectedYlimits,'RelTol', eps)
            
            % Move the grid location to a random point
            testGridLocation = [5.345, 10.34];
            map.GridLocationInWorld = testGridLocation;
            
            % Expected limits after moving the grid location
            expectedXlimits = expectedXlimits + testGridLocation(1);
            expectedYlimits = expectedYlimits + testGridLocation(2);
            
            % Verify new world limits
            testCase.verifyEqual(map.XWorldLimits, expectedXlimits,'RelTol', eps)
            testCase.verifyEqual(map.YWorldLimits, expectedYlimits,'RelTol', eps)
        end
        
        %%
        function gridLocationTest(testCase)
            %gridLocationTest Test setting and getting GridLocationInWorld
            
            [~, ~, map] = constructTestOGObj(testCase, []);
            
            % Move grid location to random points
            testGridLocations = [-32.23 -3.45;0 23.4; 43.345, 54.435];
            
            for i = 1:size(testGridLocations, 1)
                % Set grid location
                map.GridLocationInWorld = testGridLocations(i,:);
                
                % Verify grid location
                testCase.verifyEqual(map.GridLocationInWorld, ...
                    testGridLocations(i,:),'RelTol', eps)
            end
        end
        
        function gridLocationNeg(testCase)
            %gridLocationNeg Negative test for setting GridLocationInWorld
            
            [~, ~, map] = constructTestOGObj(testCase, []);
            
            testInputs = {{{1, 1}}, ...
                {[1+2i,1+2i]}, ...
                {'g'}, ...
                {}, ...
                {[1,2,3]}};
            
            errorMsgs = {'MATLAB:OccupancyGridBase:invalidType', ...
                'MATLAB:OccupancyGridBase:expectedReal', ...
                'MATLAB:OccupancyGridBase:invalidType', ...
                'MATLAB:wrongNumInDotAssign', ...
                'MATLAB:OccupancyGridBase:incorrectSize'};
            
            % Make expected error message LXE-compliant
            errorMsgs = robotics_tests.LXEConversion.convertErrorID(errorMsgs);
            
            % Invalid inputs
            for k = 1:length(errorMsgs)
                testCase.verifyError(@()gridLocationAssign(map, testInputs, k),...
                    errorMsgs{k});
            end
            
            function gridLocationAssign(map, testInputs, k)
                map.GridLocationInWorld = testInputs{k}{:};
            end
        end
        
        
        %%
        function grid2worldTest(testCase)
            %grid2worldTest Test the grid to world conversion
            
            testSize = 10;
            testInputs = [1 1; 1 10; 10 1; 10 10];
            expectedOutput = [0.5 9.5;9.5 9.5;0.5 0.5;9.5 0.5];
            
            testGridLocation = [-5 -5];
            expectedOutputWithLocation = [-4.5 4.5;4.5 4.5;-4.5 -4.5;4.5 -4.5];
            % Constructor
            map = robotics.OccupancyGrid(testSize, testSize);
            
            actualCoord = map.grid2world(testInputs);
            testCase.verifyEqual(actualCoord, expectedOutput, 'RelTol', eps);
            
            % Test with moved grid location
            map.GridLocationInWorld = testGridLocation;
            actualCoord = map.grid2world(testInputs);
            testCase.verifyEqual(actualCoord, expectedOutputWithLocation, ...
                'RelTol', eps);
        end
        
        function grid2worldResTest(testCase)
            %grid2worldResTest Test grid to world with resolution
            
            testSize = 10;
            testResolution = 2;
            testGridLocation = [-5 -5];
            testInputs = [1 1; 1 20; 20 1; 20 20];
            expectedOutput = [-4.75 4.75;4.75 4.75;-4.75 -4.75;4.75 -4.75];
            
            % Constructor
            map = robotics.OccupancyGrid(testSize, testSize, testResolution);
            map.GridLocationInWorld = testGridLocation;
            
            actualCoord = map.grid2world(testInputs);
            testCase.verifyEqual(actualCoord, expectedOutput, 'RelTol', eps);
        end
        
        function grid2worldNeg(testCase)
            %grid2worldNeg Negative test for grid2world
            
            [~, ~, map] = constructTestOGObj(testCase, []);
            
            testInputs = {{{1, 1}}, ...
                {[22, 1], 'grid'}, ...
                {[22, 2]}, ...
                {[1,1,1]}, ...
                {[1+2i,1+2i]}, ...
                {}, ...
                {[3.2, 1.2]}};
            
            errorMsgs = {'MATLAB:OccupancyGridBase:invalidType', ...
                'MATLAB:maxrhs', ...
                'robotics:robotalgs:occgridcommon:IndexExceedsDim', ...
                'MATLAB:OccupancyGridBase:incorrectNumcols', ...
                'MATLAB:OccupancyGridBase:expectedInteger', ...
                'MATLAB:minrhs', ...
                'MATLAB:OccupancyGridBase:expectedInteger'};
            
            % Make expected error message LXE-compliant
            errorMsgs = robotics_tests.LXEConversion.convertErrorID(errorMsgs);
            
            % Invalid inputs
            for k = 1:length(errorMsgs)
                testCase.verifyError(@()grid2world(map, testInputs{k}{:}),...
                    errorMsgs{k});
            end
        end
        
        %%
        function world2gridTest(testCase)
            %world2gridTest Test world to grid conversion
            
            testSize = 10;
            
            testInputs = [0 0;0.5 0.5;0.5 1;1 0.5;10 0.5;0.5 10;9.5 9.5];
            expectedOutput = [10 1;10 1;10 1;10 1;10 10;1 1;1 10];
            
            testGridLocation = [3.1 2.1];
            testInputsWithLocation = testInputs + ...
                repmat([3.1 2.1], size(testInputs, 1), 1);
            
            % Constructor
            map = robotics.OccupancyGrid(testSize,testSize);
            
            % Test with default grid location
            actualOutput = map.world2grid(testInputs);
            testCase.verifyEqual(actualOutput, expectedOutput, 'RelTol', eps);
            
            % Test with moved grid location
            map.GridLocationInWorld = testGridLocation;
            actualOutputWithLocation = map.world2grid(testInputsWithLocation);
            testCase.verifyEqual(actualOutputWithLocation, expectedOutput, 'RelTol', eps);
        end
        
        function world2gridResTest(testCase)
            %world2gridResTest Test world to grid with resolution
            
            testSize = 10;
            testResolution = 2;
            testGridLocation = [3 2];
            
            testInputs = [3.5 2.25;3.25 2.5;12.75 2.25;3.25 12;12.75 11.75];
            expectedOutput = [20 1;20 1;20 20;1 1;1 20];
            
            % Constructor
            map = robotics.OccupancyGrid(testSize,testSize, testResolution);
            map.GridLocationInWorld = testGridLocation;
            
            actualOutput = map.world2grid(testInputs);
            testCase.verifyEqual(actualOutput, expectedOutput, 'RelTol', eps);
        end
        
        function world2gridNeg(testCase)
            %world2gridNeg Negative test for world2grid
            
            [~, ~, map] = constructTestOGObj(testCase, []);
            
            testInputs = {{{1, 1}}, ...
                {[22, 1], 'grid'}, ...
                {[22, 2]}, ...
                {[1,1,1]}, ...
                {[1+2i,1+2i]}, ...
                {}};
            
            errorMsgs = {'MATLAB:OccupancyGridBase:invalidType', ...
                'MATLAB:maxrhs', ...
                'robotics:robotalgs:occgridcommon:CoordinateOutside', ...
                'MATLAB:OccupancyGridBase:incorrectNumcols', ...
                'MATLAB:OccupancyGridBase:expectedReal', ...
                'MATLAB:minrhs'};
            
            % Make expected error message LXE-compliant
            errorMsgs = robotics_tests.LXEConversion.convertErrorID(errorMsgs);
            
            % Invalid inputs
            for k = 1:length(errorMsgs)
                testCase.verifyError(@()world2grid(map, testInputs{k}{:}),...
                    errorMsgs{k});
            end
        end
        
%         %%
%         function inflateWorldTest(testCase, optionalArgW)
%             %inflateWorldTest Test the inflation with world option
%             
%             % Includes a testImage, radius in cell count, expectedImage
%             % More details regarding baseline is provided in
%             % InflateBaseline property description.
%             data = load(fullfile(testCase.DataDir, testCase.InflateBaseline));
%             
%             % Test using world coordinates Test only once that float data
%             % is rounded
%             
%             % Also checks if round-off works by multiplying 0.9
%             radiusInMeters = 0.9*data.baseline(1).radius/testCase.DefaultResolution;
%             
%             % Inflate the map and verify
%             testCase.validateInflatedMap(data.baseline(1).testImage, ...
%                 data.baseline(1).expectedImage, radiusInMeters, optionalArgW);
%         end
%         
%         function inflateTest(testCase, optionalArgG)
%             %inflateTest Test the inflation of the grid
%             
%             % Includes a testImage, radius in cell count, expectedImage
%             data = load(fullfile(testCase.DataDir, testCase.InflateBaseline));
%             
%             % Iterate over multiple data points
%             for i = 1:length(data.baseline)
%                 % Inflate the map and verify
%                 testCase.validateInflatedMap(data.baseline(i).testImage, ...
%                     data.baseline(i).expectedImage, data.baseline(i).radius,...
%                     optionalArgG);
%             end
%         end
%         
%         function inflateDefaultInputTest(testCase)
%             %inflateDefaultInputTest Test inflate method with default frame
%             
%             % Includes a testImage, radius in cell count, expectedImage
%             data = load(fullfile(testCase.DataDir, testCase.InflateBaseline));
%             
%             % Constructor Test Image is a probabilistic matrix
%             map = robotics.OccupancyGrid(data.baseline(1).testImage);
%             
%             % Test using world coordinates Also checks if round-off works
%             % by multiplying 0.9
%             radiusInMeters = 0.9*data.baseline(1).radius/map.Resolution;
%             
%             % Inflate the map
%             map.inflate(radiusInMeters);
%             
%             % Extract matrix from the occupancy grid
%             actualMatUsingWorld = testCase.extractGrid(map);
%             testCase.verifyEqual(actualMatUsingWorld, data.baseline(1).expectedImage);
%         end
%         
%         function diskstrelTest(testCase)
%             %diskstrelTest Test for internal function diskstrel
%             
%             % Following data has multiple data points for radius and
%             % expected output See description of StrelBaseline property for
%             % more details on baseline generation
%             data = load(fullfile(testCase.DataDir, testCase.StrelBaseline));
%             
%             % Iterate over multiple data points
%             for i =1:length(data.baseline)
%                 actualSe = robotics.algs.internal.diskstrel(data.baseline(i).radius);
%                 testCase.verifyEqual(actualSe, data.baseline(i).se);
%             end
%         end
%         
%         function inflateNeg(testCase)
%             %inflateNeg Negative test for inflate method
%             
%             [~, ~, map] = constructTestOGObj(testCase, []);
%             
%             testInputs = {{3,2}, ...
%                 {1+2i}, ...
%                 {3,'g',1}, ...
%                 {}, ...
%                 {10000}, ...
%                 {3.2, 'g'}};
%             
%             errorMsgs = {'robotics:robotalgs:occgridcommon:InvalidCoordType', ...
%                 'MATLAB:inflate:expectedPositive', ...
%                 'MATLAB:maxrhs', ...
%                 'MATLAB:minrhs', ...
%                 'MATLAB:inflate:notLessEqual', ...
%                 'MATLAB:inflate:expectedInteger'};
%             
%             % Make expected error message LXE-compliant
%             errorMsgs = robotics_tests.LXEConversion.convertErrorID(errorMsgs);
%             
%             % Invalid inputs
%             for k = 1:length(errorMsgs)
%                 testCase.verifyError(@()inflate(map, testInputs{k}{:}),...
%                     errorMsgs{k});
%             end
%         end
        
        %%
        function showWorldTest(testCase)
            %showWorldTest Test the default show method using world option
            
            % To ensure correct behavior, test with resolution other than
            % 1.
            [~, ~, map] = constructTestOGObj(testCase, 5);
            
            % Test default plot
            figHandle = figure;
            imgHandle = map.show('world');
            axHandle = imgHandle.Parent;
            
            % Validate if it used existing figure
            testCase.verifyEqual(figHandle, axHandle.Parent);
            
            % Validate figure attributes for world coordinates
            expectedXWLimits = map.XWorldLimits;
            expectedYWLimits = map.YWorldLimits;
            
            % World coordinate axes needs some manipulation
            correction = 1/(2*map.Resolution);
            expectedXWData = [expectedXWLimits(1)+correction, ...
                expectedXWLimits(2)-correction];
            
            % Y-axes starts from bottom, increasing towards top
            expectedYWData = [expectedYWLimits(2)-correction, ...
                expectedYWLimits(1)+correction];
            
            testCase.verifyEqual(imgHandle.XData, expectedXWData, 'RelTol', eps);
            testCase.verifyEqual(imgHandle.YData, expectedYWData, 'RelTol', eps);
            testCase.verifyEqual(axHandle.XLim, expectedXWLimits, 'RelTol', eps);
            testCase.verifyEqual(axHandle.YLim, expectedYWLimits, 'RelTol', eps);
            
            % Verify if axes is visible
            testCase.verifyMatches(axHandle.Visible, 'on');
            
            % Verify that title and axes labels match the string
            testCase.verifyEqual(axHandle.Title.String, ...
                message('robotics:robotalgs:binaryoccgrid:FigureTitle').getString);
            testCase.verifyEqual(axHandle.XLabel.String, ...
                message('robotics:robotalgs:occgridcommon:FigureXLabel').getString);
            testCase.verifyEqual(axHandle.YLabel.String, ...
                message('robotics:robotalgs:occgridcommon:FigureYLabel').getString);
            
            close(figHandle);
            
            % Validate if it plots on given axes
            % Test default plot
            figHandle = figure; %('visible', 'off');
            axHandle = axes('Parent', figHandle);
            imgHandle = map.show('world', 'Parent', axHandle);
            actualAxHandle = imgHandle.Parent;
            
            % Verify both axes are same
            testCase.verifySameHandle(actualAxHandle,axHandle);
            
            % Verify if axes is visible
            testCase.verifyMatches(axHandle.Visible, 'on');
            close(figHandle);
        end
        
        function showGridTest(testCase)
            %showGridTest Test the show method for grid input
            
            % To ensure correct behavior, test with resolution other than
            % 1.
            [~, ~, map] = constructTestOGObj(testCase, 5);
            
            % Validate figure attributes for grid coordinates
            % Test default plot
            figHandle = figure; %('visible', 'off');
            imgHandle = map.show('grid');
            axHandle = imgHandle.Parent;
            
            expectedXGData = [1 map.GridSize(2)];
            expectedYGData = [1 map.GridSize(1)];
            
            expectedXGLimits = [0.5 map.GridSize(2)+0.5];
            expectedYGLimits = [0.5 map.GridSize(1)+0.5];
            
            testCase.verifyEqual(imgHandle.XData, expectedXGData, 'RelTol', eps);
            testCase.verifyEqual(imgHandle.YData, expectedYGData, 'RelTol', eps);
            testCase.verifyEqual(axHandle.XLim, expectedXGLimits, 'RelTol', eps);
            testCase.verifyEqual(axHandle.YLim, expectedYGLimits, 'RelTol', eps);
            
            % Verify if axes is visible
            testCase.verifyMatches(axHandle.Visible, 'on');
            
            % Verify that title and axes labels match the string
            testCase.verifyEqual(axHandle.Title.String, ...
                message('robotics:robotalgs:binaryoccgrid:FigureTitle').getString);
            testCase.verifyEqual(axHandle.XLabel.String, ...
                message('robotics:robotalgs:occgridcommon:FigureColLabel').getString);
            testCase.verifyEqual(axHandle.YLabel.String, ...
                message('robotics:robotalgs:occgridcommon:FigureRowLabel').getString);
            close(figHandle);
            
            % Validate if it plots on given axes
            
            % Test default plot
            figHandle = figure; %('visible', 'off');
            axHandle = axes('Parent', figHandle);
            imgHandle = map.show('grid', 'Parent', axHandle);
            actualAxHandle = imgHandle.Parent;
            
            % Verify both axes are same
            testCase.verifySameHandle(actualAxHandle,axHandle);
            
            close(figHandle);
        end
        
        function showNeg(testCase)
            %showNeg Negative tests for show method
            
            [~, ~, map] = constructTestOGObj(testCase, []);
            
            testFigHandle = figure;
            testAxesHandle = axes('Parent', testFigHandle);
            testPlotHandle = plot(zeros(10,1));
            
            testInputs = {{'Parent', testAxesHandle}, ...
                {'world','Parent', testFigHandle}, ...
                {'world','Parent', testPlotHandle}, ...
                {'Parent', testAxesHandle, 'grid'}, ...
                {'g', 23}};
            
            errorMsgs = {'robotics:robotalgs:occgridcommon:InvalidCoordString', ...
                'robotics:validation:InvalidAxesHandle', ...
                'robotics:validation:InvalidAxesHandle', ...
                'robotics:robotalgs:occgridcommon:InvalidCoordString',...
                'MATLAB:InputParser:ParamMustBeChar'};
            
            % Invalid inputs
            for k = 1:length(errorMsgs)
                testCase.verifyError(@()show(map, testInputs{k}{:}),...
                    errorMsgs{k});
            end
            
            close(testFigHandle);
        end
    end
    
    methods (Test, ParameterCombination='sequential')
        function showTestEdgeCase(testCase, showTestGridLocation, showTestResolution, ...
                showTestSize, showTestExpectedXData, showTestExpectedYData)
            %showTestEdgeCase Test the show method for g1157624
            [~, ~, map] = constructTestOGObj(testCase, showTestResolution, ...
                showTestSize(1), showTestSize(2));
            
            map.GridLocationInWorld = showTestGridLocation;
            
            % Test default plot
            figHandle = figure;
            axHandle = axes('Parent', figHandle);
            imgHandle = map.show('world', 'Parent', axHandle);
            
            
            % Validate figure attributes for world coordinates
            expectedXWLimits = map.XWorldLimits;
            expectedYWLimits = map.YWorldLimits;
            
            testCase.verifyEqual(imgHandle.XData, showTestExpectedXData, 'RelTol', eps);
            testCase.verifyEqual(imgHandle.YData, showTestExpectedYData, 'RelTol', eps);
            testCase.verifyEqual(axHandle.XLim, expectedXWLimits, 'RelTol', eps);
            testCase.verifyEqual(axHandle.YLim, expectedYWLimits, 'RelTol', eps);
            
            close(figHandle);
        end
    end
    
    methods (Access = private)
        function matrix = extractGrid(~, map)
            %extractGrid Extract entire grid using getOccupancy
            gridsize = map.GridSize;
            [x, y] = ndgrid(1:gridsize(1), 1:gridsize(2));
            occgrid = map.getOccupancy([x(:), y(:)], 'grid');
            matrix = reshape(occgrid, gridsize(1), gridsize(2));
        end
        
        function validateInflatedMap(testCase, testImage, expectedImage, radius, optarg)
            %validateInflatedMap Validate inflated map
            
            % Constructor
            % Test Image is a probabilistic matrix
            map = robotics.OccupancyGrid(testImage);
            
            % Inflate the map
            map.inflate(radius, optarg);
            
            % Extract matrix from the occupancy grid
            actualInflatedImage = testCase.extractGrid(map);
            testCase.verifyEqual(actualInflatedImage, expectedImage);
        end
        
        function [tWidth, tHeight, map] = constructTestOGObj(testCase, resolution, tWidth, tHeight)
            %constructTestOGObj Construct an object to test
            
            if nargin == 2
                tWidth = 10;
                tHeight = 20;
            end
            
            if isempty(resolution)
                resolution = testCase.DefaultResolution;
            end
            % Test default constructor
            map = robotics.OccupancyGrid(tWidth, tHeight, resolution);
        end
    end
    
    methods(Access = protected)
        function sObj = getSaveObject(testCase)
            %getSaveObject Test object to save load and copy
            [~, ~, sObj] = constructTestOGObj(testCase, []);
        end
    end
    
end

