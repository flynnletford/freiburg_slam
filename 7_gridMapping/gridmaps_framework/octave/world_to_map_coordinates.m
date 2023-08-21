function [pntsMap] = world_to_map_coordinates(pntsWorld, gridSize, offset)
% Convert points from the world coordinates frame to the map frame.
% pntsWorld is a matrix of N points with each column representing a point in world coordinates (meters).
% gridSize is the size of each grid in meters.
% offset = [offsetX; offsetY] is the offset that needs to be subtracted from a point
% when converting to map coordinates.
% pntsMap is a 2xN matrix containing the corresponding points in map coordinates.

% TODO: compute pntsMap

pntsMap = zeros(2,columns(pntsWorld));

offsetX = offset(1);
offsetY = offset(2);

for i = 1:columns(pntsWorld)

    x = (pntsWorld(1,i) - offsetX) / gridSize;
    y = (pntsWorld(2,i) - offsetY) / gridSize;

    pntsMap(1,i) = floor(x);
    pntsMap(2,i) = floor(y);
end

end
