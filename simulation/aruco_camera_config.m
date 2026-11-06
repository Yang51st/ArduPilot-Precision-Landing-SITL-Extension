% Configuration for mavlinkMST_ArucoCamera.slx.
% Stop and restart the Simulink model after changing this file.

configFolder = fileparts(mfilename('fullpath'));
allConfig = jsondecode(fileread(fullfile(configFolder, '..', 'landing_config.json')));
landingConfig = allConfig.sim;

% Replace this with any square ArUco PNG/JPG. The image is reloaded each run.
markerImageFile = fullfile(configFolder, landingConfig.marker_image);

% The picture can hold several ArUco markers, each with its own ID; the
% "markers" list in the JSON gives the physical size of each one for the
% Python bridge. MarkerSizeMeters is the rendered width of the whole picture,
% so it must be at least as large as the largest marker in that list.
if isfield(landingConfig, 'marker_image_width_m')
    markerSizeMeters = landingConfig.marker_image_width_m;
else
    markerSizeMeters = max([landingConfig.markers.size_m]);
end

% Marker pose in the same local NED frame reported by LOCAL_POSITION_NED.
% The default places the centre of the picture at SITL home, flat on ground.
markerNorthMeters = landingConfig.marker_north_m;
markerEastMeters = landingConfig.marker_east_m;
markerYawDegrees = landingConfig.marker_yaw_deg;
groundDownMeters = landingConfig.ground_down_m;

% Simple pinhole-camera settings.
cameraHorizontalFovDegrees = landingConfig.camera_hfov_deg;
cameraImageWidth = landingConfig.camera_width;
cameraImageHeight = landingConfig.camera_height;
cameraFramePeriodSeconds = 1 / landingConfig.camera_fps;

% The Python companion-style bridge connects here for raw RGB frames.
[cameraFrameHost, cameraFramePortText] = strtok(landingConfig.frame_endpoint, ':');
cameraFramePort = str2double(cameraFramePortText(2:end));
