classdef Tbx
    % Tbx Check for installation of VDX required toolboxes.
    %   Includes static methods to return boolean flags for verifying dependency installation.
    %
    % See also Tbx.list.

    properties (Constant, Access = private)
        Toolboxes = {
            'Symbolic Math Toolbox', 'hasSymbolic';
            'Optimization Toolbox', 'hasOptim';
            'Curve Fitting Toolbox', 'hasCurvefit';
            'Signal Processing Toolbox', 'hasSignal';
            };
    end

    methods (Static)
        function x = hasSymbolic()
            x = license('test', 'Symbolic_Toolbox');
        end

        function x = hasOptim()
            x = license('test', 'Optimization_Toolbox');
        end

        function x = hasCurvefit()
            x = license('test', 'Curve_Fitting_Toolbox');
        end

        function x = hasSignal()
            x = license('test', 'Signal_Toolbox');
        end

        function x = hasAll()
            x = true;
            for i = 1:length(Tbx.Toolboxes)
                if ~Tbx.(Tbx.Toolboxes{i, 2})()
                    x = false;
                    return;
                end
            end
        end

        function ver()
            % Tbx.ver is a clone of MATLAB ver.
            %
            % See also VER.

            ver;
        end

        function list()
            % Tbx.list prints compatibility list of VDX toolbox requirements.

            allPresent = true;
            for i = 1:length(Tbx.Toolboxes)
                [name, method] = deal(Tbx.Toolboxes{i, :});
                available = Tbx.(method)();

                if available
                    fprintf(' %-32sInstalled\n', name);
                else
                    fprintf(2, ' %-32sMissing\n', name);  % 2 - red text in console
                    allPresent = false;
                end
            end

            if allPresent
                fprintf('\nAll required toolboxes are installed!\n')
            else
                fprintf(2, '\nOne or more required toolboxes are missing.\n');
                fprintf('https://www.mathworks.com/help/matlab/matlab_env/get-add-ons.html\n');
            end
        end
    end
end
