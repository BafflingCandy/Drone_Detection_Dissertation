function [x, y, totalIterations, converged, usedMask, rejectedCount, maxNormResidual] = ...
    robustWeightedLeastSquares2d(sensorPositions, ranges, rangeStd, initialPosition)
%ROBUSTWEIGHTEDLEASTSQUARES2D WLS localisation with residual outlier rejection.
%
% The first WLS pass estimates position using all detecting sensors. The
% residual for each sensor is then checked:
%     residual = predicted_range - measured_range
% Large normalised residuals indicate that one sensor's range estimate is not
% consistent with the others. When possible, the worst sensor is removed and
% WLS is repeated.

    residualThreshold = 4.0;
    maxReject = 2;

    n = size(sensorPositions, 1);
    usedMask = true(n, 1);
    rejectedCount = 0;
    totalIterations = 0;
    converged = false;
    maxNormResidual = NaN;
    currentInitial = initialPosition;

    x = NaN;
    y = NaN;

    for pass = 1:(maxReject + 1)
        activePositions = sensorPositions(usedMask, :);
        activeRanges = ranges(usedMask);
        activeStd = rangeStd(usedMask);

        weights = 1 ./ (activeStd .^ 2);
        weights = weights / max(weights);

        [xCandidate, yCandidate, iterations, passConverged] = weightedLeastSquares2d( ...
            activePositions, activeRanges, weights, currentInitial);
        totalIterations = totalIterations + iterations;

        if ~isfinite(xCandidate) || ~isfinite(yCandidate)
            return;
        end

        x = xCandidate;
        y = yCandidate;
        converged = passConverged;

        predicted = sqrt((x - activePositions(:, 1)).^2 + ...
                         (y - activePositions(:, 2)).^2);
        normResiduals = abs(predicted - activeRanges) ./ activeStd;
        [maxNormResidual, worstActiveIndex] = max(normResiduals);

        if maxNormResidual <= residualThreshold || sum(usedMask) <= 4
            return;
        end

        activeIndices = find(usedMask);
        candidateMask = usedMask;
        candidateMask(activeIndices(worstActiveIndex)) = false;

        if sum(candidateMask) < 4 || ~hasNonCollinearPoints(sensorPositions(candidateMask, :))
            return;
        end

        usedMask = candidateMask;
        rejectedCount = rejectedCount + 1;
        currentInitial = [x, y];
    end
end

function ok = hasNonCollinearPoints(points)
    if size(points, 1) < 3
        ok = false;
        return;
    end

    centred = points - mean(points, 1);
    ok = rank(centred) >= 2;
end
