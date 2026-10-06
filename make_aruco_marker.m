function make_aruco_marker(outputFile, markerID, markerFamily, markerPixels)
%MAKE_ARUCO_MARKER Generate a marker image accepted by the camera model.
% Example:
%   make_aruco_marker('markers/my_marker.png', 23, "DICT_6X6_250", 400)

    arguments
        outputFile {mustBeTextScalar}
        markerID (1,1) double {mustBeInteger, mustBeNonnegative}
        markerFamily (1,1) string = "DICT_6X6_250"
        markerPixels (1,1) double {mustBeInteger, mustBePositive} = 400
    end

    outputFile = char(outputFile);
    outputFolder = fileparts(outputFile);
    if ~isempty(outputFolder) && ~isfolder(outputFolder)
        mkdir(outputFolder);
    end
    marker = generateArucoMarker(markerFamily, markerID, markerPixels);
    imwrite(marker, outputFile);
    fprintf('Wrote %s (family %s, ID %d).\n', outputFile, markerFamily, markerID);
end
