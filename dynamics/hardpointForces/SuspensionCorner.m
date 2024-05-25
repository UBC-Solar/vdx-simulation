classdef SuspensionCorner
    properties
        Coords = table('Size',[17 3],'VariableTypes',["double","double","double"],...
            RowNames=["O","MC","LCA_IR","LCA_IF","LCA_O","UCA_IR",...
            "UCA_IF","UCA_O","PR_O","PR_I","RP","RS","SC","TR_O","TR_I",...
            "WC","TP"],VariableNames=["X","Y","Z"]);

        O (3,1) double {mustBeReal, mustBeFinite};          % Coordinate system origin
        MC (3,1) double {mustBeReal, mustBeFinite};         % Moment center (typically same as origin)
        LCA_IR (3,1) double {mustBeReal, mustBeFinite};     % LCA inboard rear rod end
        LCA_IF (3,1) double {mustBeReal, mustBeFinite};     % LCA inboard front rod end
        LCA_O (3,1) double {mustBeReal, mustBeFinite};      % LCA outboard ball joint
        UCA_IR (3,1) double {mustBeReal, mustBeFinite};     % UCA inboard rear rod end
        UCA_IF (3,1) double {mustBeReal, mustBeFinite};     % UCA inboard front rod end
        UCA_O (3,1) double {mustBeReal, mustBeFinite};      % UCA outboard ball joint
        PR_O (3,1) double {mustBeReal, mustBeFinite};       % Pull/push rod outboard rod end
        PR_I (3,1) double {mustBeReal, mustBeFinite};       % Pull/push rod inboard rod end
        RP (3,1) double {mustBeReal, mustBeFinite};         % Rocker pivot
        RS (3,1) double {mustBeReal, mustBeFinite};         % Rocker-shock mount
        SC (3,1) double {mustBeReal, mustBeFinite};         % Shock-chassis mount
        TR_O (3,1) double {mustBeReal, mustBeFinite};       % Steering tie-rod outboard rod end
        TR_I (3,1) double {mustBeReal, mustBeFinite};       % Steering tie-rod inboard rod end
        WC (3,1) double {mustBeReal, mustBeFinite};         % Wheel center
        TP (3,1) double {mustBeReal, mustBeFinite};         % Tire contact patch    
    end
    properties (Dependent)
        LCA (3,3) double {mustBeReal, mustBeFinite};        
        UCA (3,3) double {mustBeReal, mustBeFinite};
        PR (3,2) double {mustBeReal, mustBeFinite};
        R (3,3) double {mustBeReal, mustBeFinite};
        S (3,2) double {mustBeReal, mustBeFinite};
        TR (3,2) double {mustBeReal, mustBeFinite};
        W (3,100) double {mustBeReal, mustBeFinite};

    end
    properties (Constant)
        x = 1; y = 2; z = 3;
    end
    methods
        function obj = SuspensionCorner(coords)
            if nargin > 0
                obj.Coords = coords;
                obj.O = obj.Coords{'O',1:3}';
                obj.MC = obj.Coords{'MC',1:3}';
                obj.LCA_IR = obj.Coords{'LCA_IR',1:3}';     
                obj.LCA_IF= obj.Coords{'LCA_IF',1:3}';
                obj.LCA_O = obj.Coords{'LCA_O',1:3}';
                obj.UCA_IR = obj.Coords{'UCA_IR',1:3}';
                obj.UCA_IF = obj.Coords{'UCA_IF',1:3}';
                obj.UCA_O = obj.Coords{'UCA_O',1:3}';
                obj.PR_O = obj.Coords{'PR_O',1:3}';
                obj.PR_I = obj.Coords{'PR_I',1:3}';
                obj.RP = obj.Coords{'RP',1:3}';
                obj.RS = obj.Coords{'RS',1:3}';
                obj.SC = obj.Coords{'SC',1:3}';
                obj.TR_O = obj.Coords{'TR_O',1:3}';
                obj.TR_I = obj.Coords{'TR_I',1:3}';
                obj.WC = obj.Coords{'WC',1:3}';
                obj.TP = obj.Coords{'TP',1:3}';

                obj.LCA = [obj.LCA_IF obj.LCA_O obj.LCA_IR];
                obj.UCA = [obj.UCA_IF obj.UCA_O obj.UCA_IR];
                obj.PR = [obj.PR_O obj.PR_I];
                obj.R = [obj.PR_I obj.RP obj.RS];
                obj.S = [obj.RS obj.SC];
                obj.TR = [obj.TR_O obj.TR_I];
                t = linspace(0,2*pi);
                obj.W = [norm([0.45-obj.WC(2) obj.WC(3)])*sin(t) + obj.WC(1);
                         0*t + obj.WC(2);
                         norm([0.45-obj.WC(2) obj.WC(3)])*cos(t) + obj.WC(3)];
            end
        end
        
        function corner = plotCorner(corner,fig,lineWidth)
            if isa(fig,"fig"
            figure(fig)
            scatter3(corner.Coords,"X","Y","Z",'filled','SizeData',hardpointsize,'Marker','o','MarkerFaceColor','blue')
            line(corner.LCA(:,corner.x),corner.LCA(:,corner.y),corner.LCA(:,corner.z),'Color','blue','LineWidth',lineWidth);
            line(corner.UCA(:,corner.x),corner.UCA(:,corner.y),corner.UCA(:,corner.z),'Color','blue','LineWidth',lineWidth);
            line(corner.PR(:,corner.x),corner.PR(:,corner.y),corner.PR(:,corner.z),'Color','black','LineWidth',lineWidth);
            line(corner.R(:,corner.x),corner.R(:,corner.y),corner.R(:,corner.z),'Color','black','LineWidth',lineWidth);
            line(corner.S(:,corner.x),corner.S(:,corner.y),corner.S(:,corner.z),'Color','black','LineWidth',lineWidth);
            line(corner.TR(:,corner.x),corner.TR(:,corner.y),corner.TR(:,corner.z),'Color','green','LineWidth',lineWidth);
            plot3(corner.W(corner.x,:),corner.W(corner.y,:),corner.W(corner.z,:),'Color','red','LineWidth',lineWidth)
        end

        function fout = SuspensionCorner.calcForces(loadCase)
            if nargin > 0

            end
        end
    end
end