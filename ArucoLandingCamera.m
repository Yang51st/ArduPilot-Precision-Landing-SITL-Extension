classdef ArucoLandingCamera < matlab.System
    %ARUCOLANDINGCAMERA Simple downward-looking camera for SITL landing tests.
    %
    % Inputs use MAVLink's local NED frame. The attitude quaternion rotates
    % body-FRD vectors into NED. The simulated camera is fixed to the body:
    % optical +x is body-right, optical +y is body-back, and optical +z is
    % body-down. A landing-picture image is placed on the horizontal ground
    % plane; the picture may contain a much smaller embedded ArUco marker.

    properties (Nontunable)
        MarkerFile = 'aruco_marker.png'
        MarkerSizeMeters = 0.40 % physical width of the complete picture
        MarkerNorthMeters = 0
        MarkerEastMeters = 0
        MarkerYawDegrees = 0
        GroundDownMeters = 0
        HorizontalFovDegrees = 70
        ImageWidth = 320
        ImageHeight = 240
        FramePeriodSeconds = 0.05
    end

    properties (Access = private)
        MarkerRGB
        RaysBody
    end

    methods (Access = protected)
        function setupImpl(obj)
            [marker, map, alpha] = imread(obj.MarkerFile);
            if ~isempty(map)
                marker = im2uint8(ind2rgb(marker, map));
            elseif ~isa(marker, 'uint8')
                marker = im2uint8(marker);
            end
            if ismatrix(marker)
                marker = repmat(marker, 1, 1, 3);
            end
            if ~isempty(alpha)
                if ~isa(alpha, 'uint8')
                    alpha = im2uint8(alpha);
                end
                a = double(alpha) / 255;
                marker = uint8(double(marker) .* a + 255 .* (1 - a));
            end
            obj.MarkerRGB = marker(:, :, 1:3);

            width = double(obj.ImageWidth);
            height = double(obj.ImageHeight);
            focalLength = (width / 2) / tand(double(obj.HorizontalFovDegrees) / 2);
            [u, v] = meshgrid(1:width, 1:height);
            opticalX = (u - (width + 1) / 2) / focalLength;
            opticalY = (v - (height + 1) / 2) / focalLength;

            % Optical [right; down-image; forward] to body FRD.
            obj.RaysBody = [-opticalY(:)'; opticalX(:)'; ones(1, numel(u))];
        end

        function image = stepImpl(obj, north, east, down, q1, q2, q3, q4)
            q = double([q1 q2 q3 q4]);
            qNorm = norm(q);
            if ~isfinite(qNorm) || qNorm < 1e-8
                q = [1 0 0 0];
            else
                q = q / qNorm;
            end

            w = q(1); x = q(2); y = q(3); z = q(4);
            bodyToNed = [ ...
                1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y); ...
                2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x); ...
                2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)];

            cameraPosition = double([north; east; down]);
            raysNed = bodyToNed * obj.RaysBody;
            scale = (double(obj.GroundDownMeters) - cameraPosition(3)) ./ raysNed(3, :);
            onGround = isfinite(scale) & scale > 0 & raysNed(3, :) > 1e-6;

            pointNorth = cameraPosition(1) + raysNed(1, :) .* scale;
            pointEast = cameraPosition(2) + raysNed(2, :) .* scale;

            pixelCount = double(obj.ImageWidth) * double(obj.ImageHeight);
            pixels = repmat(uint8([135; 195; 235]), 1, pixelCount); % sky

            % A subtle one-metre checkerboard makes translation and attitude
            % visible without requiring a heavyweight 3-D scene engine.
            groundIndex = find(onGround);
            checker = mod(floor(pointNorth(groundIndex)) + ...
                          floor(pointEast(groundIndex)), 2);
            shade = uint8(212 - 12 * checker);
            pixels(:, groundIndex) = repmat(shade, 3, 1);

            deltaNorth = pointNorth - double(obj.MarkerNorthMeters);
            deltaEast = pointEast - double(obj.MarkerEastMeters);
            yaw = deg2rad(double(obj.MarkerYawDegrees));
            markerForward = cos(yaw) .* deltaNorth + sin(yaw) .* deltaEast;
            markerRight = -sin(yaw) .* deltaNorth + cos(yaw) .* deltaEast;

            markerHeight = size(obj.MarkerRGB, 1);
            markerWidth = size(obj.MarkerRGB, 2);
            % MarkerSizeMeters is the physical width of the complete image.
            % Preserve the source aspect ratio so an ArUco embedded in a
            % non-square landing picture remains physically square.
            markerWidthMeters = double(obj.MarkerSizeMeters);
            markerHeightMeters = markerWidthMeters * ...
                double(markerHeight - 1) / double(markerWidth - 1);
            markerRow = round((0.5 - markerForward / markerHeightMeters) * ...
                (markerHeight - 1) + 1);
            markerColumn = round((markerRight / markerWidthMeters + 0.5) * ...
                (markerWidth - 1) + 1);
            onMarker = onGround & markerRow >= 1 & markerRow <= markerHeight & ...
                markerColumn >= 1 & markerColumn <= markerWidth;

            outputIndex = find(onMarker);
            if ~isempty(outputIndex)
                sourceIndex = sub2ind([markerHeight markerWidth], ...
                    markerRow(outputIndex), markerColumn(outputIndex));
                for channel = 1:3
                    plane = obj.MarkerRGB(:, :, channel);
                    pixels(channel, outputIndex) = plane(sourceIndex);
                end
            end

            image = zeros(obj.ImageHeight, obj.ImageWidth, 3, 'uint8');
            for channel = 1:3
                image(:, :, channel) = reshape(pixels(channel, :), ...
                    obj.ImageHeight, obj.ImageWidth);
            end
        end

        function sizeOut = getOutputSizeImpl(obj)
            sizeOut = [obj.ImageHeight obj.ImageWidth 3];
        end

        function typeOut = getOutputDataTypeImpl(~)
            typeOut = 'uint8';
        end

        function fixedOut = isOutputFixedSizeImpl(~)
            fixedOut = true;
        end

        function complexOut = isOutputComplexImpl(~)
            complexOut = false;
        end

        function [north, east, down, q1, q2, q3, q4] = getInputNamesImpl(~)
            north = 'north';
            east = 'east';
            down = 'down';
            q1 = 'q1';
            q2 = 'q2';
            q3 = 'q3';
            q4 = 'q4';
        end

        function image = getOutputNamesImpl(~)
            image = 'RGB image';
        end

        function icon = getIconImpl(~)
            icon = sprintf('ArUco landing\ncamera');
        end

        function sampleTime = getSampleTimeImpl(obj)
            sampleTime = createSampleTime(obj, 'Type', 'Discrete', ...
                'SampleTime', obj.FramePeriodSeconds);
        end
    end

    methods (Static, Access = protected)
        function simMode = getSimulateUsingImpl
            simMode = 'Interpreted execution';
        end

        function show = showSimulateUsingImpl
            show = false;
        end
    end
end
