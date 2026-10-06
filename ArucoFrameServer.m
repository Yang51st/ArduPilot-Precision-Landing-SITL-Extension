classdef ArucoFrameServer < matlab.System
    %ARUCOFRAMESERVER Publish fixed-size RGB frames to the Python bridge.
    %
    % Protocol (little endian): 24-byte header followed by packed RGB bytes.
    % Header fields are magic "APSM", width, height, channels, frame number,
    % and payload byte count, all integer fields being uint32.

    properties (Nontunable)
        Host = '127.0.0.1'
        Port = 5600
    end

    properties (Access = private)
        Server
        FrameNumber = uint32(0)
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.Server = tcpserver(obj.Host, obj.Port, 'Timeout', 1);
        end

        function stepImpl(obj, image)
            if isempty(obj.Server) || ~obj.Server.Connected
                return;
            end

            height = uint32(size(image, 1));
            width = uint32(size(image, 2));
            channels = uint32(size(image, 3));
            raw = reshape(permute(uint8(image), [3 2 1]), [], 1);
            payloadBytes = uint32(numel(raw));
            obj.FrameNumber = obj.FrameNumber + 1;

            header = [uint8('APSM')'; ...
                typecast(width, 'uint8')'; ...
                typecast(height, 'uint8')'; ...
                typecast(channels, 'uint8')'; ...
                typecast(obj.FrameNumber, 'uint8')'; ...
                typecast(payloadBytes, 'uint8')'];
            try
                write(obj.Server, header, 'uint8');
                write(obj.Server, raw, 'uint8');
            catch
                % A disconnected client is expected when the bridge restarts.
                % tcpserver will report Connected=false on a later time step.
            end
        end

        function releaseImpl(obj)
            obj.Server = [];
        end

        function name = getInputNamesImpl(~)
            name = 'RGB image';
        end

        function icon = getIconImpl(~)
            icon = sprintf('RGB frames to\nPython');
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
