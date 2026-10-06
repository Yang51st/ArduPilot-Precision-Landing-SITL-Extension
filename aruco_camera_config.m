% Configuration for mavlinkMST_ArucoCamera.slx.
% Stop and restart the Simulink model after changing this file.

configFolder = fileparts(mfilename('fullpath'));
landingConfig = jsondecode(fileread(fullfile(configFolder, ...
    'aruco_landing_config.json')));

% Replace this with any square ArUco PNG/JPG. The image is reloaded each run.
markerImageFile = fullfile(configFolder, landingConfig.marker_image);

% The picture can contain an embedded ArUco whose physical size differs from
% the full landing picture. MarkerSizeMeters controls the rendered picture;
% marker_size_m is reserved for OpenCV's pose estimate of the inner ArUco.
if isfield(landingConfig, 'marker_image_width_m')
    markerSizeMeters = landingConfig.marker_image_width_m;
else
    markerSizeMeters = landingConfig.marker_size_m;
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
cameraFrameHost = landingConfig.frame_host;
cameraFramePort = landingConfig.frame_port;
