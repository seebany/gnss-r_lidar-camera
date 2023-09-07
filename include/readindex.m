function outputFrame = readindex(videoSource,frameNumber)
% Used to extract required frame from video
% Shahrukh Khan
% 2020

info = get(videoSource); % Get video information
videoSource.CurrentTime=(frameNumber-1)/info.FrameRate; % Set timestamp for required frame
outputFrame = readFrame(videoSource); % Read video frame
end
