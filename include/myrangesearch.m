function [Idx, D] = myrangesearch(X, Y, r)
% function [Idx, D] = myrangesearch(X, Y, r) is a homemade version of the
% Matlab Statistics&Machine Learning Toolbox function rangesearch.m a
% https://www.mathworks.com/help/stats/rangesearch.html
% It finds all the X points that are within distance r of the Y points.
% Rows of X and Y correspond to observations, and columns correspond to
% variables.
% Unlike rangesearch, the indices are sorted by index number, not by
% distance. And it only does Euclidean distance.
%
% Seebany Datta-Barua
% 9 Dec 2022

% Compute dimensions of X and Y needed later.
numYrows = size(Y,1);
numXrows = size(X,1);

% Initialize output as empty cell array.
Idx = cell(numYrows,1);

% Loop through each row of Y
for i = 1:numYrows
    % Repeat that row to match the number of rows of X.
    Yrep = repmat(Y(i,:), numXrows,1);

    % Euclidean distance is sqrt of sum of squares.
    D(:,i) = sqrt( sum( (X - Yrep).^2, 2))';
    Idx{i} = find(D(:,i) < r);
end