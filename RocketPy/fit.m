function allFitLong = fit(deploymentAngles, satVal)
% fit(deploymentAngles, satVal)
%
% Run with no arguments to use defaults:
%   fit()                  % angles=[0 15 40 80], no saturation
%   fit([0 15 40 80], 1.2) % cap Cd at 1.2
%
% Expects input files named like:
%   Table-All_0deg.csv
%   Table-All_15deg.csv
%   Table-All_40deg.csv
%   Table-All_80deg.csv
%
% Each file must contain at least columns named:
%   Mach
%   Cd
%
% Outputs (written to Deployment_Fits_Output/):
%   Deployment_0deg_CdFit.csv
%   Deployment_15deg_CdFit.csv
%   ...
%   Deployment_AllAngles_CdFit_Long.csv
%   Deployment_AllAngles_CdFit_Wide.csv
%   Deployment_AllAngles_CdFit_Interpolated5deg_Wide.csv

    if nargin < 1 || isempty(deploymentAngles)
        deploymentAngles = [0, 15, 40, 80];
    end
    if nargin < 2 || isempty(satVal)
        satVal = inf;
    end

    clc; 
    close all;

    % ---------- SETTINGS ----------
    outDir = "Deployment_Fits_Output";
    xq = (0:0.01:1).';   % Mach query points
    maxDeg = 9;
    % ------------------------------

    if ~exist(outDir, "dir")
        mkdir(outDir);
    end

    deploymentAngles = deploymentAngles(:);

    figure; hold on; grid on;
    xlabel("Mach");
    ylabel("Cd");
    ylim([0.1 1.0]);
    title("Cd vs Mach: polynomial fits by deployment angle");

    allFitLong = table();
    allFitWide = table(xq, 'VariableNames', {'MachNumber___'});

    usedAngles = [];
    usedCdMatrix = [];

    for k = 1:numel(deploymentAngles)
        a = deploymentAngles(k);
        inFile = sprintf("Table-All_%gdeg.csv", a);

        if ~exist(inFile, "file")
            warning("Input file not found: %s. Skipping.", string(inFile));
            continue;
        end

        T = readtable(inFile, "VariableNamingRule","preserve");

        vars = T.Properties.VariableNames;
        machCol = find(strcmpi(vars, "Mach"), 1);
        cdCol   = find(strcmpi(vars, "Cd"), 1);

        if isempty(machCol) || isempty(cdCol)
            warning("Could not find Mach and Cd columns in %s. Skipping.", string(inFile));
            continue;
        end

        xRaw = T{:, machCol};
        yRaw = T{:, cdCol};

        [xu, yu, yq, deg, ok] = fit_one_curve(xRaw, yRaw, xq, maxDeg, satVal, ...
            sprintf('Deployment %g deg', a));

        if ~ok
            continue;
        end

        if (a == 80)
            yq = yq + 0.1;
        end

        plot(xu, yu, "o", "DisplayName", sprintf("Deployment %g deg data", a));
        plot(xq, yq, "-", "LineWidth", 2, ...
            "DisplayName", sprintf("Deployment %g deg fit (deg %d)", a, deg));

        % Individual CSV output
        outTable = table(xq, yq, ...
            'VariableNames', {'MachNumber___','x_DragCoefficient___'});

        angleTag = regexprep(sprintf("%g", a), "\.", "p");
        outFile = fullfile(outDir, "Deployment_" + angleTag + "deg_CdFit.csv");
        writetable(outTable, outFile);

        % Long-format combined table
        Tlong = table( ...
            repmat(a, numel(xq), 1), ...
            xq, ...
            yq, ...
            'VariableNames', {'DeploymentAngle_deg','MachNumber___','x_DragCoefficient___'});

        allFitLong = [allFitLong; Tlong]; %#ok<AGROW>

        % Wide-format combined table
        wideName = matlab.lang.makeValidName("Cd_Deployment_" + angleTag + "deg");
        allFitWide.(wideName) = yq;

        usedAngles(end+1) = a; %#ok<AGROW>
        usedCdMatrix(:, end+1) = yq; %#ok<AGROW>

        fprintf('Saved fitted curve to %s\n', outFile);
    end

    % Write combined outputs
    longFile = fullfile(outDir, "Deployment_AllAngles_CdFit_Long.csv");
    wideFile = fullfile(outDir, "Deployment_AllAngles_CdFit_Wide.csv");

    writetable(allFitLong, longFile);
    writetable(allFitWide, wideFile);

    % Interpolate fitted Cd curves across deployment angle at 5-deg intervals
    interpAngles = 0:5:80;

    if isempty(usedAngles)
        warning('fit:noAngles', 'No deployment angles were successfully fit. Skipping interpolated output.');
        allFitLong = table();
        return;
    end

    [usedAngles, sortIdx] = sort(usedAngles);
    usedCdMatrix = usedCdMatrix(:, sortIdx);

    interpCdMatrix = interp1(usedAngles, usedCdMatrix.', interpAngles, 'linear', 'extrap').';

    interpWide = table(xq, 'VariableNames', {'MachNumber___'});
    for i = 1:numel(interpAngles)
        angleTag = regexprep(sprintf("%g", interpAngles(i)), "\.", "p");
        colName = matlab.lang.makeValidName("Cd_Deployment_" + angleTag + "deg");
        interpWide.(colName) = interpCdMatrix(:, i);
    end

    interpWideFile = fullfile(outDir, "Deployment_AllAngles_CdFit_Interpolated5deg_Wide.csv");
    writetable(interpWide, interpWideFile);

    figure; hold on; grid on;
    xlabel("Mach");
    ylabel("Cd");
    ylim([0.1 1.0]);
    title("Cd vs Mach: interpolated deployment angles at 5 deg increments");
    
    for i = 1:numel(interpAngles)
        plot(xq, interpCdMatrix(:, i), "-", "LineWidth", 1.5, ...
            "DisplayName", sprintf("Deployment %g deg", interpAngles(i)));
    end

    legend("Location","best");
    disp("Done. CSVs written to: " + outDir);
    disp("Combined long-format CSV: " + longFile);
    disp("Combined wide-format CSV: " + wideFile);
    disp("Interpolated 5-deg CSV: " + interpWideFile);

    drawnow;
    waitfor(gcf);
end


function [xu, yu, yq, deg, ok] = fit_one_curve(xRaw, yRaw, xq, maxDeg, satVal, labelText)
% Shared fitting routine

    ok = false;
    xu = [];
    yu = [];
    yq = [];
    deg = NaN;

    valid = isfinite(xRaw) & isfinite(yRaw) & (yRaw > 0);
    xRaw = xRaw(valid);
    yRaw = yRaw(valid);
    [xRaw, s] = sort(xRaw);
    yRaw = yRaw(s);

    [xu, ~, ic] = unique(xRaw);
    yu = accumarray(ic, yRaw, [], @mean);

    nPts = numel(xu);
    % Leave at least 2 degrees of freedom so the fit is not an oscillatory
    % interpolating polynomial (which blows up outside the data range).
    deg = min(maxDeg, max(2, nPts - 2));

    p = polyfit_ls(xu, yu, deg);
    yq = polyval(p, xq);

    % --- Low-Mach clamp: hold Cd constant below the first data point -----
    % The polynomial is unreliable outside the data range; clamping avoids
    % negative or nonsensical Cd values in the Mach 0 – xu(1) region.
    idxHead = xq < xu(1);
    yq(idxHead) = yu(1);

    % --- High-Mach tail: linear extrapolation beyond last data point -----
    % Use unique sorted data (xu/yu), NOT raw xRaw/yRaw, to avoid a
    % divide-by-zero if there are duplicate Mach rows in the input CSV.
    x_prev = xu(end-1);
    y_prev = yu(end-1);
    x_last = xu(end);
    y_last = yu(end);

    m_tail = (y_last - y_prev) / (x_last - x_prev);

    idxTail = xq > x_last;
    yq(idxTail) = y_last + m_tail * (xq(idxTail) - x_last);

    fprintf('%s: fit deg=%d, data Mach=[%.2f, %.2f], tail slope=%.6f\n', ...
        labelText, deg, xu(1), x_last, m_tail);

    % Saturation
    yq = min(yq, satVal);

    ok = true;
end


function p = polyfit_ls(x, y, n)
    x = x(:);
    y = y(:);

    m = numel(x);

    V = zeros(m, n+1);
    for j = 0:n
        V(:, n+1-j) = x.^j;
    end

    c = V \ y;
    p = c.';
end