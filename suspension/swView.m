function swView(mode, ax)
% sw_view(mode, ax)
% SolidWorks-like view controls in MATLAB.
%
% Modes:
%   'front'      - Look along -Z
%   'back'       - Look along +Z
%   'left'       - Look along +X
%   'right'      - Look along -X
%   'top'        - Look along -Y
%   'bottom'     - Look along +Y
%   'iso'        - Standard isometric (X+, Y+, Z+)
%   'iso_flip'   - Flipped isometric (X-, Y+, Z-)
%
% Optional second argument: axis handle (defaults to gca).

    if nargin < 2 || isempty(ax), ax = gca; end
    
    % Disable MATLAB's Z-up snapping
    cameratoolbar('SetCoordSys','none');
    
    % Lock various camera modes
    set(ax, 'CameraTargetMode','manual', ...
            'CameraPositionMode','manual', ...
            'CameraUpVectorMode','manual', ...
            'DataAspectRatioMode','manual');
    
    axis(ax,'equal');
    grid(ax,'on');
    
    switch lower(mode)
        case 'front'
            view(ax, [0 0 -1]);
            camup(ax, [0 1 0]);
    
        case 'back'
            view(ax, [0 0 1]);
            camup(ax, [0 1 0]);
    
        case 'left'
            view(ax, [1 0 0]);
            camup(ax, [0 1 0]);
    
        case 'right'
            view(ax, [-1 0 0]);
            camup(ax, [0 1 0]);
    
        case 'top'
            view(ax, [0 -1 0]);
            camup(ax, [0 0 -1]); % Keep Z pointing down-screen
    
        case 'bottom'
            view(ax, [0 1 0]);
            camup(ax, [0 0 1]);  % Keep Z pointing up-screen
    
        case 'iso'
            view(ax, [1 1 1]);
            camup(ax, [0 1 0]);
    
        case 'iso_flip'
            view(ax, [-1 1 -1]);
            camup(ax, [0 1 0]);
    
        otherwise
            error('Unknown view mode: %s', mode);
    end
    % % Enable free rotation without losing Y-up
    % rotate3d(ax,'on');
end
