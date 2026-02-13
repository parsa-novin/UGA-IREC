function polyfit(inFile)
% polyfit(inFile)
% Run:
%   matlab -batch "polyfit('Table-All.csv')"

    if nargin < 1 || strlength(string(inFile)) == 0
        error("Usage: polyfit(""your_data.csv"")");
    end

    clc; close all;

    % ---------- SETTINGS ----------
    outDir = "AoA_Fits_Output";
    xq = (0:0.01:1).';   % Mach query points (column)
    maxDeg = 9;
    % -----------------------------

    if ~exist(inFile, "file")
        error("Input file not found: %s", string(inFile));
    end
    if ~exist(outDir, "dir")
        mkdir(outDir);
    end

    % Preserve original CSV headers (prevents that warning)
    T = readtable(inFile, "VariableNamingRule","preserve");

    % Column detection
    vars = T.Properties.VariableNames;
    machCol = find(strcmpi(vars, "Mach"), 1);
    cdCol   = find(strcmpi(vars, "Cd"), 1);
    aoaCol  = find(contains(lower(vars), "aoa"), 1);

    if isempty(machCol) || isempty(cdCol) || isempty(aoaCol)
        error("Could not find required columns. Need Mach, Cd, and AoA.");
    end

    Mach = T{:, machCol};
    Cd   = T{:, cdCol};
    AoA  = T{:, aoaCol};

    % Clean
    mask = isfinite(Mach) & isfinite(Cd) & isfinite(AoA);
    Mach = Mach(mask);
    Cd   = Cd(mask);
    AoA  = AoA(mask);

    aoaVals = unique(AoA, "stable");

    % Plot
    figure; hold on; grid on;
    xlabel("Mach"); ylabel("Cd");
    ylim([0.1 0.6]);
    title("Cd vs Mach: polynomial fits by AoA");

    for k = 1:numel(aoaVals)
        a = aoaVals(k);
        idx = (AoA == a);

        x = Mach(idx);
        y = Cd(idx);

        % Sort by Mach
        [x, s] = sort(x);
        y = y(s);

        % Average duplicate Mach values (helps stability)
        [xu, ~, ic] = unique(x);
        yu = accumarray(ic, y, [], @mean);

        nPts = numel(xu);
        deg = min(maxDeg, nPts - 1);
        if deg < 1
            warning("AoA = %g deg has <2 points; skipping.", a);
            continue;
        end

        % Local polynomial fit (least squares)
        p  = polyfit_ls(xu, yu, deg);   % row vector high->low power
        yq = polyval(p, xq);

        % Plot data and fit
        plot(xu, yu, "o", "DisplayName", sprintf("AoA %g data", a));
        plot(xq, yq, "-", "LineWidth", 2, "DisplayName", sprintf("AoA %g fit (deg %d)", a, deg));

        % Export CSV with required headers
        outTable = table(xq, yq, 'VariableNames', {'MachNumber___','x_DragCoefficient___'});
        aoaTag = regexprep(sprintf("%g", a), "\.", "p");  % 2.5 -> 2p5
        outFile = fullfile(outDir, "AoA_" + aoaTag + "deg_CdFit.csv");
        writetable(outTable, outFile);
    end

    legend("Location","best");
    disp("Done. CSVs written to: " + outDir);
    drawnow;
    disp("Close the figure window or press Ctrl+C in this terminal to end.");
    waitfor(gcf);   % waits until the current figure is closed
end

function p = polyfit_ls(x, y, n)
% Least-squares polynomial fit like MATLAB polyfit (returns coeffs high->low)
    x = x(:);
    y = y(:);

    m = numel(x);
    if n >= m
        error("Polynomial degree n must be < number of points.");
    end

    % Vandermonde matrix with descending powers: [x^n ... x^1 x^0]
    V = zeros(m, n+1);
    for j = 0:n
        V(:, n+1-j) = x.^j;
    end

    c = V \ y;       % least-squares solution
    p = c.';         % row vector for polyval
end
