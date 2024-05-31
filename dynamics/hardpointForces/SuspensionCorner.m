classdef SuspensionCorner
    properties
        Coords = table('Size',[17 3],'VariableTypes',["double","double","double"],...
            RowNames=["O","MC","LCA_IR","LCA_IF","LCA_O","UCA_IR",...
            "UCA_IF","UCA_O","PR_O","PR_I","RP","RS","SC","TR_O","TR_I",...
            "WC","TP"],VariableNames=["X","Y","Z"]);

        % Variables for each coordinate
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

        % Coordinate matrices to represent each component
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
                obj.W = [norm([0.45-abs(obj.WC(2)) obj.WC(3)])*sin(t) + obj.WC(1);
                         0*t + obj.WC(2);
                         norm([0.45-abs(obj.WC(2)) obj.WC(3)])*cos(t) + obj.WC(3)];
            end
        end
        
        function corner = plotCorner(corner,fig,hardpointsize,lineWidth)
            if nargin > 3
                figure(fig)
                if fig.Children ~= 0
                    hold on
                end
                scatter3(corner.Coords,"X","Y","Z",'filled','SizeData',hardpointsize,'Marker','o','MarkerFaceColor','blue')
                hold on
                line(corner.LCA(corner.x,:),corner.LCA(corner.y,:),corner.LCA(corner.z,:),'Color','blue','LineWidth',lineWidth);
                line(corner.UCA(corner.x,:),corner.UCA(corner.y,:),corner.UCA(corner.z,:),'Color','blue','LineWidth',lineWidth);
                line(corner.PR(corner.x,:),corner.PR(corner.y,:),corner.PR(corner.z,:),'Color','black','LineWidth',lineWidth);
                line(corner.R(corner.x,:),corner.R(corner.y,:),corner.R(corner.z,:),'Color','black','LineWidth',lineWidth);
                line(corner.S(corner.x,:),corner.S(corner.y,:),corner.S(corner.z,:),'Color','black','LineWidth',lineWidth);
                line(corner.TR(corner.x,:),corner.TR(corner.y,:),corner.TR(corner.z,:),'Color','green','LineWidth',lineWidth);
                plot3(corner.W(corner.x,:),corner.W(corner.y,:),corner.W(corner.z,:),'Color','red','LineWidth',lineWidth)
                hold off
                xlabel('x (m)');
                ylabel('y (m)');
                zlabel('z (m)');
                axis equal
            end
        end

        function F_Out = calcForces(corner,F_TP)
            if nargin > 1

                % Calculate unit vectors for force direction in each 
                % linkage
                u_LCA_F = (corner.LCA_O-corner.LCA_IF)/(norm(corner.LCA_O-corner.LCA_IF));
                u_LCA_R = (corner.LCA_O-corner.LCA_IR)/(norm(corner.LCA_O-corner.LCA_IR));
                u_UCA_F = (corner.UCA_O-corner.UCA_IF)/(norm(corner.UCA_O-corner.UCA_IF));
                u_UCA_R = (corner.UCA_O-corner.UCA_IR)/(norm(corner.UCA_O-corner.UCA_IR));
                u_PR = (corner.PR_O-corner.PR_I)/(norm(corner.PR_O-corner.PR_I));
                u_S = (corner.SC-corner.RS)/(norm(corner.SC-corner.RS));
                u_TR = (corner.TR_O-corner.TR_I)/(norm(corner.TR_O-corner.TR_I));
                
                % Calculate moment arms for each chassis point
                d_LCA_IR = corner.LCA_IR-corner.MC;
                d_LCA_IF = corner.LCA_IF-corner.MC;
                d_UCA_IR = corner.UCA_IR-corner.MC;
                d_UCA_IF = corner.UCA_IF-corner.MC;
                d_TR_I = corner.TR_I-corner.MC;
                d_PR_I = corner.PR_I-corner.MC;
                d_TP = corner.TP-corner.MC;

                % Calculate unit moment vectors for each chassis point
                % using force unit vectors and moment arms
                uM_UCA_IF = cross(d_UCA_IF, u_UCA_F);
                uM_UCA_IR = cross(d_UCA_IR, u_UCA_R);
                uM_LCA_IF = cross(d_LCA_IF, u_LCA_F);
                uM_LCA_IR = cross(d_LCA_IR, u_LCA_R);
                uM_PR = cross(d_PR_I, u_PR);
                uM_TR = cross(d_TR_I, u_TR);

                % Calculate moments due to input force at tire contact
                % patch
                M_TP = cross(d_TP,F_TP);

                % A*F_mag = B
                % Construct A and B matrices
                A = [u_UCA_F,  u_UCA_R,  u_LCA_F,  u_LCA_R,  u_PR, u_TR;
                     uM_UCA_IF,uM_UCA_IR,uM_LCA_IF,uM_LCA_IR,uM_PR,uM_TR];

                B = [-F_TP';
                     -M_TP'];

                % Find F_mag as A^-1*B or A/B
                F_mag = A\B;

                % Multiply each force magnitude by its unit vector to find
                % force vectors
                F_UCA_F = F_mag(1)*u_UCA_F;
                F_UCA_R = F_mag(2)*u_UCA_R;
                F_LCA_F = F_mag(3)*u_LCA_F;
                F_LCA_R = F_mag(4)*u_LCA_R;
                F_PR = F_mag(5)*u_PR;
                F_TR = F_mag(6)*u_TR;

                % Forces at upright balljoints
                F_UCA_O = F_UCA_F + F_UCA_R + F_PR;
                F_LCA_O = F_LCA_F + F_LCA_R;

                %% Calculate forces in rocker
                % Transformation matrix from base frame to FL rocker frame
                n_R = cross(corner.R(:,1)-corner.R(:,2), ...
                            corner.R(:,3)-corner.R(:,2));   % Find vector normal to rocker plane by tkaing cross product of rocker arms
                u3_R = (n_R/norm(n_R))';                    % Unit normal (u3)
                u1_R = ((corner.R(:,3)-corner.R(:,1))/ ...
                    norm(corner.R(:,3)-corner.R(:,1)))';    % Set unit X vector (u1) as difference between rocker-shock point and rocker-pull-rod point
                u2_R = cross(u3_R,u1_R);                    % Set unit Y vector (u2) as cross product of u1 and u3
                R_Transform = [u1_R;                        % Rocker coordinate transformation matrix
                               u2_R;
                               u3_R];
                % Transform the relevant already existing vectors
                u_RP_S = R_Transform*u_S;
                F_RP_PR = R_Transform*F_PR;

                % Calculate new moment arms as distance from point to 
                % rocker pivot
                d_RP_PR = R_Transform*(corner.PR_I - corner.RP);
                d_RP_RS = R_Transform*(corner.RS - corner.RP);

                % Moment about rocker pivot from pullrod input force
                M_RP_PR = R_Transform*cross(d_RP_PR, F_RP_PR);
                
                % Moment unit vectors about rocker pivot for shock
                uM_RP_RS = R_Transform*cross(d_RP_RS, u_RP_S);
                
                % Setting up matrix and solving
                RP_A = [1 0 0 0 0 u_RP_S(corner.x)
                        0 1 0 0 0 u_RP_S(corner.y)
                        0 0 1 0 0 u_RP_S(corner.z)
                        0 0 0 1 0 uM_RP_RS(corner.x)
                        0 0 0 0 1 uM_RP_RS(corner.y)
                        0 0 0 0 0 uM_RP_RS(corner.z)];
                RP_B = [-F_RP_PR; -M_RP_PR];
                F_RP_mag = RP_A\RP_B;
                
                F_RP_RP = F_RP_mag(1:3);
                M_RP_RP = [F_RP_mag(4:5); 0];
                F_RP_S = F_RP_mag(6)*u_S;

                F_RP = R_Transform\F_RP_RP;
                M_RP = R_Transform\M_RP_RP;
                F_S = R_Transform\F_RP_S;
                

                F_Out = cell2table({"UCA Front" F_UCA_F'; 
                                    "UCA Rear" F_UCA_R';
                                    "UCA Balljoint" F_UCA_O'; 
                                    "LCA Front" F_LCA_F';
                                    "LCA Rear" F_LCA_R';
                                    "LCA Balljoint" F_LCA_O';
                                    "Steering Tie-rod" F_TR';
                                    "Pull-rod" F_PR';
                                    "Rocker pivot" F_RP';
                                    "Rocker pivot moment" M_RP';
                                    "Shock" F_S';
                                    "Rocker Coords - Rocker Pivot" F_RP_RP';
                                    "Rocker Coords - Rocker Pivot Moment" M_RP_RP';
                                    "Rocker Coords - Pull-rod" F_RP_PR';
                                    "Rocker Coords - Shock" F_RP_S'},...
                                    "VariableNames",{'Member/Point','Force/Moment'});
            end
        end
    end
end