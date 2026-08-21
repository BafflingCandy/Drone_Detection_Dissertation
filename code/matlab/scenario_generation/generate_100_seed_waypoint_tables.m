% generate_100_seed_waypoint_tables.m
%
% Creates appendix/corpus CSV tables for the 100 deterministic random
% waypoint drone runs used in the prison RF sensor dissertation experiment.
%
% The random number generator and waypoint logic intentionally mirror the
% ns-3 implementation in uav-net-sim-prison-sensor.cc. This means seed N in
% this MATLAB table corresponds to --randomSeed=N in the FlyNetSim/ns-3 run.

clear; clc;

matlabRoot = fileparts(fileparts(mfilename('fullpath')));
addpath(matlabRoot);
paths = corpus_paths();

%% Output location
outputDir = paths.scenarios;
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

%% Prison/fence scenario constants copied from the ns-3 prison sensor model
simMinX = 0.0;
simMaxX = 1000.0;
simMinY = 0.0;
simMaxY = 1000.0;

prisonMinX = 300.0;
prisonMaxX = 700.0;
prisonMinY = 350.0;
prisonMaxY = 650.0;
clearZoneM = 50.0;

fenceMinX = prisonMinX - clearZoneM;
fenceMaxX = prisonMaxX + clearZoneM;
fenceMinY = prisonMinY - clearZoneM;
fenceMaxY = prisonMaxY + clearZoneM;

droneAltitudeM = 20.0;
droneSpeedMps = 10.0;
numSeeds = 100;
numIntermediateWaypoints = 3;

% Same vulnerable target used in the ns-3 random runs unless changed by
% --randomTargetX/--randomTargetY.
target = [560.0, 520.0, droneAltitudeM];

% Distance resolution for the coverage sample table. This is not used by
% ns-3 movement directly; it is only for appendix coverage visualization.
sampleSpacingM = 10.0;

entryNames = [
    "west"
    "east"
    "south"
    "north"
    "south_west"
    "north_west"
    "south_east"
    "north_east"
];

wideRows = table();
longRows = table();
sampleRows = table();
summaryRows = table();

for seed = 1:numSeeds
    [waypoints, entryCase, entryRegion] = generateWaypointsForSeed(seed, target, ...
        simMinX, simMaxX, simMinY, simMaxY, fenceMinX, fenceMaxX, fenceMinY, fenceMaxY, ...
        numIntermediateWaypoints, droneAltitudeM, entryNames);

    cumulativeDistances = cumulativePathDistances(waypoints);
    totalPathDistanceM = cumulativeDistances(end);

    [boundaryFound, boundaryPoint, boundaryDistanceM, boundarySegmentIndex] = findPrisonEntry( ...
        waypoints, cumulativeDistances, prisonMinX, prisonMaxX, prisonMinY, prisonMaxY);

    if ~boundaryFound
        boundaryPoint = waypoints(end, :);
        boundaryDistanceM = totalPathDistanceM;
        boundarySegmentIndex = height(array2table(waypoints)) - 1;
    end

    startPoint = waypoints(1, :);
    targetPoint = waypoints(end, :);

    wideRows = [wideRows; table(seed, entryCase, string(entryRegion), ...
        startPoint(1), startPoint(2), startPoint(3), ...
        waypoints(2,1), waypoints(2,2), waypoints(2,3), ...
        waypoints(3,1), waypoints(3,2), waypoints(3,3), ...
        waypoints(4,1), waypoints(4,2), waypoints(4,3), ...
        targetPoint(1), targetPoint(2), targetPoint(3), ...
        boundaryFound, boundaryPoint(1), boundaryPoint(2), boundaryPoint(3), ...
        boundaryDistanceM, totalPathDistanceM, boundarySegmentIndex, ...
        boundaryDistanceM / droneSpeedMps, totalPathDistanceM / droneSpeedMps, ...
        'VariableNames', {'seed','entry_case','entry_region', ...
        'start_x_m','start_y_m','start_z_m', ...
        'waypoint1_x_m','waypoint1_y_m','waypoint1_z_m', ...
        'waypoint2_x_m','waypoint2_y_m','waypoint2_z_m', ...
        'waypoint3_x_m','waypoint3_y_m','waypoint3_z_m', ...
        'target_x_m','target_y_m','target_z_m', ...
        'crosses_secure_wall','boundary_x_m','boundary_y_m','boundary_z_m', ...
        'boundary_distance_m','total_path_distance_m','boundary_segment_index', ...
        'time_to_boundary_s','total_flight_time_s'})];

    waypointTypes = ["start"; "intermediate"; "intermediate"; "intermediate"; "target"];
    for waypointIndex = 1:size(waypoints, 1)
        point = waypoints(waypointIndex, :);
        longRows = [longRows; table(seed, waypointIndex - 1, waypointTypes(waypointIndex), ...
            point(1), point(2), point(3), cumulativeDistances(waypointIndex), ...
            'VariableNames', {'seed','waypoint_index','waypoint_type','x_m','y_m','z_m','distance_along_path_m'})];
    end

    sampleDistances = 0:sampleSpacingM:totalPathDistanceM;
    if sampleDistances(end) < totalPathDistanceM
        sampleDistances = [sampleDistances, totalPathDistanceM];
    end

    zoneCounts = struct('outside_region', 0, 'early_warning', 0, 'approach_clear_zone', 0, ...
        'perimeter_crossing', 0, 'inside_secure_wall', 0);

    for sampleIndex = 1:numel(sampleDistances)
        d = sampleDistances(sampleIndex);
        point = interpolatePathAtDistance(waypoints, cumulativeDistances, d);
        zone = classifyZone(point, fenceMinX, fenceMaxX, fenceMinY, fenceMaxY, ...
            prisonMinX, prisonMaxX, prisonMinY, prisonMaxY);
        zoneCounts.(char(zone)) = zoneCounts.(char(zone)) + 1;

        sampleRows = [sampleRows; table(seed, sampleIndex - 1, d, ...
            point(1), point(2), point(3), string(zone), ...
            'VariableNames', {'seed','sample_index','distance_along_path_m','x_m','y_m','z_m','zone'})];
    end

    summaryRows = [summaryRows; table(seed, entryCase, string(entryRegion), boundaryFound, ...
        min(waypoints(:,1)), max(waypoints(:,1)), min(waypoints(:,2)), max(waypoints(:,2)), ...
        boundaryPoint(1), boundaryPoint(2), boundaryDistanceM, totalPathDistanceM, ...
        zoneCounts.outside_region, zoneCounts.early_warning, zoneCounts.approach_clear_zone, ...
        zoneCounts.perimeter_crossing, zoneCounts.inside_secure_wall, ...
        'VariableNames', {'seed','entry_case','entry_region','crosses_secure_wall', ...
        'min_x_m','max_x_m','min_y_m','max_y_m','boundary_x_m','boundary_y_m', ...
        'boundary_distance_m','total_path_distance_m','outside_region_samples', ...
        'early_warning_samples','approach_clear_zone_samples','perimeter_crossing_samples', ...
        'inside_secure_wall_samples'})];
end

%% Write professor/corpus CSV files
wideFile = fullfile(outputDir, 'random_waypoint_100_seed_wide.csv');
longFile = fullfile(outputDir, 'random_waypoint_100_seed_long.csv');
sampleFile = fullfile(outputDir, 'random_waypoint_100_seed_path_samples.csv');
summaryFile = fullfile(outputDir, 'random_waypoint_100_seed_coverage_summary.csv');

writetable(wideRows, wideFile);
writetable(longRows, longFile);
writetable(sampleRows, sampleFile);
writetable(summaryRows, summaryFile);

%% Create coverage plots for quick visual checking
plotAllPaths(wideRows, longRows, prisonMinX, prisonMaxX, prisonMinY, prisonMaxY, ...
    fenceMinX, fenceMaxX, fenceMinY, fenceMaxY, fullfile(outputDir, 'random_waypoint_100_seed_paths.png'));
plotPointScatter(wideRows.start_x_m, wideRows.start_y_m, wideRows.entry_region, ...
    prisonMinX, prisonMaxX, prisonMinY, prisonMaxY, fenceMinX, fenceMaxX, fenceMinY, fenceMaxY, ...
    '100 Seed Random Start Points', fullfile(outputDir, 'random_waypoint_100_seed_start_points.png'));
plotPointScatter(wideRows.boundary_x_m, wideRows.boundary_y_m, wideRows.entry_region, ...
    prisonMinX, prisonMaxX, prisonMinY, prisonMaxY, fenceMinX, fenceMaxX, fenceMinY, fenceMaxY, ...
    '100 Seed Secure-Wall Crossing Points', fullfile(outputDir, 'random_waypoint_100_seed_boundary_points.png'));

figure('Color', 'w');
entrySummary = groupsummary(wideRows, 'entry_region');
bar(categorical(entrySummary.entry_region), entrySummary.GroupCount);
ylabel('Number of seeds');
title('Entry Region Counts for 100 Random Waypoint Seeds');
grid on;
exportgraphics(gcf, fullfile(outputDir, 'random_waypoint_100_seed_entry_region_counts.png'), 'Resolution', 200);
close(gcf);

fprintf('Created 100-seed waypoint coordinate tables:\n');
fprintf('  %s\n', wideFile);
fprintf('  %s\n', longFile);
fprintf('  %s\n', sampleFile);
fprintf('  %s\n', summaryFile);

%% Local functions
function [waypoints, entryCase, entryRegion] = generateWaypointsForSeed(seed, target, ...
    simMinX, simMaxX, simMinY, simMaxY, fenceMinX, fenceMaxX, fenceMinY, fenceMaxY, ...
    numIntermediateWaypoints, droneAltitudeM, entryNames)

    state = double(seed);
    [entryCaseRand, state] = randomBetween(state, 0.0, 8.0);
    entryCase = floor(entryCaseRand);
    entryRegion = entryNames(entryCase + 1);

    [offset, state] = randomBetween(state, 90.0, 180.0);

    if entryCase == 0
        [startY, state] = randomBetween(state, fenceMinY, fenceMaxY);
        start = [clampDouble(fenceMinX - offset, simMinX, simMaxX), startY, droneAltitudeM];
    elseif entryCase == 1
        [startY, state] = randomBetween(state, fenceMinY, fenceMaxY);
        start = [clampDouble(fenceMaxX + offset, simMinX, simMaxX), startY, droneAltitudeM];
    elseif entryCase == 2
        [startX, state] = randomBetween(state, fenceMinX, fenceMaxX);
        start = [startX, clampDouble(fenceMinY - offset, simMinY, simMaxY), droneAltitudeM];
    elseif entryCase == 3
        [startX, state] = randomBetween(state, fenceMinX, fenceMaxX);
        start = [startX, clampDouble(fenceMaxY + offset, simMinY, simMaxY), droneAltitudeM];
    elseif entryCase == 4
        start = [clampDouble(fenceMinX - offset, simMinX, simMaxX), ...
            clampDouble(fenceMinY - offset, simMinY, simMaxY), droneAltitudeM];
    elseif entryCase == 5
        start = [clampDouble(fenceMinX - offset, simMinX, simMaxX), ...
            clampDouble(fenceMaxY + offset, simMinY, simMaxY), droneAltitudeM];
    elseif entryCase == 6
        start = [clampDouble(fenceMaxX + offset, simMinX, simMaxX), ...
            clampDouble(fenceMinY - offset, simMinY, simMaxY), droneAltitudeM];
    else
        start = [clampDouble(fenceMaxX + offset, simMinX, simMaxX), ...
            clampDouble(fenceMaxY + offset, simMinY, simMaxY), droneAltitudeM];
    end

    waypoints = start;
    dx = target(1) - start(1);
    dy = target(2) - start(2);
    length2d = sqrt(dx * dx + dy * dy);
    if length2d > 1e-9
        perpX = -dy / length2d;
        perpY = dx / length2d;
    else
        perpX = 0.0;
        perpY = 0.0;
    end

    for i = 1:numIntermediateWaypoints
        progress = i / (numIntermediateWaypoints + 1);
        [latRand, state] = randomBetween(state, -80.0, 80.0);
        [forwardJitter, state] = randomBetween(state, -25.0, 25.0);
        lateralOffset = latRand * sin(pi * progress);

        waypointX = clampDouble(start(1) + dx * progress + perpX * lateralOffset + ...
            (dx / max(length2d, 1.0)) * forwardJitter, simMinX, simMaxX);
        waypointY = clampDouble(start(2) + dy * progress + perpY * lateralOffset + ...
            (dy / max(length2d, 1.0)) * forwardJitter, simMinY, simMaxY);
        waypoints = [waypoints; waypointX, waypointY, droneAltitudeM];
    end

    waypoints = [waypoints; target];
end

function [value, state] = randomBetween(state, minValue, maxValue)
    [r, state] = nextRandom01(state);
    value = minValue + (maxValue - minValue) * r;
end

function [r, state] = nextRandom01(state)
    % Mirrors C++ uint32_t LCG wrap-around:
    % state = state * 1664525u + 1013904223u;
    state = mod(state * 1664525.0 + 1013904223.0, 2^32);
    r = mod(state, 2^24) / 2^24;
end

function value = clampDouble(value, minValue, maxValue)
    value = max(minValue, min(value, maxValue));
end

function cumulativeDistances = cumulativePathDistances(waypoints)
    cumulativeDistances = zeros(size(waypoints, 1), 1);
    for i = 2:size(waypoints, 1)
        cumulativeDistances(i) = cumulativeDistances(i-1) + norm(waypoints(i,:) - waypoints(i-1,:));
    end
end

function [found, boundaryPoint, boundaryDistanceM, boundarySegmentIndex] = findPrisonEntry( ...
    waypoints, cumulativeDistances, prisonMinX, prisonMaxX, prisonMinY, prisonMaxY)

    found = false;
    boundaryPoint = [NaN, NaN, NaN];
    boundaryDistanceM = NaN;
    boundarySegmentIndex = NaN;

    for i = 2:size(waypoints, 1)
        a = waypoints(i-1, :);
        b = waypoints(i, :);
        [segmentFound, t] = findSegmentPrisonEntryFraction(a, b, prisonMinX, prisonMaxX, prisonMinY, prisonMaxY);
        if segmentFound
            boundaryPoint = a + (b - a) * t;
            segmentDistance = norm(b - a);
            boundaryDistanceM = cumulativeDistances(i-1) + segmentDistance * t;
            boundarySegmentIndex = i - 2;
            found = true;
            return;
        end
    end
end

function [found, bestT] = findSegmentPrisonEntryFraction(a, b, prisonMinX, prisonMaxX, prisonMinY, prisonMaxY)
    found = false;
    bestT = 2.0;
    dx = b(1) - a(1);
    dy = b(2) - a(2);

    if abs(dx) > 1e-9
        txWest = (prisonMinX - a(1)) / dx;
        yWest = a(2) + dy * txWest;
        if txWest >= 0.0 && txWest <= 1.0 && yWest >= prisonMinY && yWest <= prisonMaxY
            found = true;
            bestT = min(bestT, txWest);
        end

        txEast = (prisonMaxX - a(1)) / dx;
        yEast = a(2) + dy * txEast;
        if txEast >= 0.0 && txEast <= 1.0 && yEast >= prisonMinY && yEast <= prisonMaxY
            found = true;
            bestT = min(bestT, txEast);
        end
    end

    if abs(dy) > 1e-9
        tySouth = (prisonMinY - a(2)) / dy;
        xSouth = a(1) + dx * tySouth;
        if tySouth >= 0.0 && tySouth <= 1.0 && xSouth >= prisonMinX && xSouth <= prisonMaxX
            found = true;
            bestT = min(bestT, tySouth);
        end

        tyNorth = (prisonMaxY - a(2)) / dy;
        xNorth = a(1) + dx * tyNorth;
        if tyNorth >= 0.0 && tyNorth <= 1.0 && xNorth >= prisonMinX && xNorth <= prisonMaxX
            found = true;
            bestT = min(bestT, tyNorth);
        end
    end
end

function point = interpolatePathAtDistance(waypoints, cumulativeDistances, distanceAlongPath)
    if distanceAlongPath <= 0
        point = waypoints(1, :);
        return;
    end

    if distanceAlongPath >= cumulativeDistances(end)
        point = waypoints(end, :);
        return;
    end

    for i = 2:size(waypoints, 1)
        if distanceAlongPath <= cumulativeDistances(i)
            a = waypoints(i-1, :);
            b = waypoints(i, :);
            segmentStart = cumulativeDistances(i-1);
            segmentDistance = cumulativeDistances(i) - segmentStart;
            fraction = (distanceAlongPath - segmentStart) / segmentDistance;
            point = a + (b - a) * fraction;
            return;
        end
    end

    point = waypoints(end, :);
end

function zone = classifyZone(point, fenceMinX, fenceMaxX, fenceMinY, fenceMaxY, ...
    prisonMinX, prisonMaxX, prisonMinY, prisonMaxY)

    x = point(1);
    y = point(2);
    toleranceM = 5.0;

    insideSecureWall = x >= prisonMinX && x <= prisonMaxX && y >= prisonMinY && y <= prisonMaxY;
    insideFence = x >= fenceMinX && x <= fenceMaxX && y >= fenceMinY && y <= fenceMaxY;
    nearSecureWall = (abs(x - prisonMinX) <= toleranceM || abs(x - prisonMaxX) <= toleranceM || ...
        abs(y - prisonMinY) <= toleranceM || abs(y - prisonMaxY) <= toleranceM) && insideFence;

    if insideSecureWall
        zone = "inside_secure_wall";
    elseif nearSecureWall
        zone = "perimeter_crossing";
    elseif insideFence
        zone = "approach_clear_zone";
    elseif x >= 0 && x <= 1000 && y >= 0 && y <= 1000
        zone = "early_warning";
    else
        zone = "outside_region";
    end
end

function plotAllPaths(wideRows, longRows, prisonMinX, prisonMaxX, prisonMinY, prisonMaxY, ...
    fenceMinX, fenceMaxX, fenceMinY, fenceMaxY, outputFile)

    figure('Color', 'w');
    hold on;
    axis equal;
    grid on;
    xlim([0 1000]);
    ylim([0 1000]);
    drawFacility(prisonMinX, prisonMaxX, prisonMinY, prisonMaxY, fenceMinX, fenceMaxX, fenceMinY, fenceMaxY);

    for seed = unique(longRows.seed)'
        pathRows = longRows(longRows.seed == seed, :);
        plot(pathRows.x_m, pathRows.y_m, '-', 'Color', [0.1 0.4 0.8 0.22], 'LineWidth', 0.8);
    end

    scatter(wideRows.start_x_m, wideRows.start_y_m, 20, 'filled', 'MarkerFaceColor', [0.85 0.25 0.1], ...
        'MarkerFaceAlpha', 0.65);
    scatter(wideRows.boundary_x_m, wideRows.boundary_y_m, 20, 'filled', 'MarkerFaceColor', [0.1 0.6 0.25], ...
        'MarkerFaceAlpha', 0.65);
    scatter(wideRows.target_x_m, wideRows.target_y_m, 45, 'kx', 'LineWidth', 1.5);
    title('100 Random Waypoint Drone Paths');
    xlabel('x coordinate (m)');
    ylabel('y coordinate (m)');
    legend({'Outer fence', 'Secure wall', 'Path samples', 'Start points', 'Secure-wall crossing', 'Target'}, ...
        'Location', 'eastoutside');
    exportgraphics(gcf, outputFile, 'Resolution', 200);
    close(gcf);
end

function plotPointScatter(x, y, groups, prisonMinX, prisonMaxX, prisonMinY, prisonMaxY, ...
    fenceMinX, fenceMaxX, fenceMinY, fenceMaxY, plotTitle, outputFile)

    figure('Color', 'w');
    hold on;
    axis equal;
    grid on;
    xlim([0 1000]);
    ylim([0 1000]);
    drawFacility(prisonMinX, prisonMaxX, prisonMinY, prisonMaxY, fenceMinX, fenceMaxX, fenceMinY, fenceMaxY);

    % Keep plotting toolbox-free: draw one scatter series per entry region
    % instead of using gscatter from the Statistics and Machine Learning Toolbox.
    uniqueGroups = unique(groups);
    colors = lines(numel(uniqueGroups));
    for i = 1:numel(uniqueGroups)
        mask = groups == uniqueGroups(i);
        scatter(x(mask), y(mask), 32, colors(i,:), 'filled', 'MarkerFaceAlpha', 0.75, ...
            'DisplayName', char(uniqueGroups(i)));
    end

    title(plotTitle);
    xlabel('x coordinate (m)');
    ylabel('y coordinate (m)');
    legend('Location', 'eastoutside');
    exportgraphics(gcf, outputFile, 'Resolution', 200);
    close(gcf);
end

function drawFacility(prisonMinX, prisonMaxX, prisonMinY, prisonMaxY, fenceMinX, fenceMaxX, fenceMinY, fenceMaxY)
    rectangle('Position', [fenceMinX, fenceMinY, fenceMaxX - fenceMinX, fenceMaxY - fenceMinY], ...
        'EdgeColor', [0.15 0.15 0.15], 'LineWidth', 1.5, 'LineStyle', '--');
    rectangle('Position', [prisonMinX, prisonMinY, prisonMaxX - prisonMinX, prisonMaxY - prisonMinY], ...
        'EdgeColor', [0.05 0.05 0.05], 'LineWidth', 2.5);
end
