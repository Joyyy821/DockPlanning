map = load("exampleMaps.mat").simpleMap;
startPose = [5 5 pi/2];
goalPose = [22 4 0];
% Create a state space object
stateSpace = stateSpaceSE2;

% Create a state validator object
validator = validatorOccupancyMap(stateSpace);

% Create a binary occupancy map and assign the map to the state 
% validator object.
validator.Map = binaryOccupancyMap(map);

% Set the validation distance for the validator
validator.ValidationDistance = 0.01;

% Assign the state validator object to the plannerHybridAStar object
planner = plannerHybridAStar(validator);

% Compute a path for the given start and goal poses
pathObj = plan(planner,startPose,goalPose);

% Extract the path poses from the path object
path = pathObj.States;
path = floor(path);
show(planner,"Tree", "off")
hold on