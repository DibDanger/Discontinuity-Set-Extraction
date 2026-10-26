clc 
clear all

% Save outputs alongside this MATLAB script
outputFolder = fileparts(mfilename('fullpath'));
%%  Reading point cloud file: LAS or PCD format 
[filename, pathname] = uigetfile('*.*','MultiSelect','on');
    if isequal(filename,0) || isequal(pathname,0)
       disp('User pressed cancel')
        
    else
       disp(['User selected ', fullfile(pathname, filename)])
    end
 name1=fullfile(pathname, filename);
 i=1;
%  name = name1{1,i};
name = name1;
%  PC = pcread(name)
 lasReader = lasFileReader(name);
 PC =readPointCloud(lasReader);
 PCL = PC.Location;
 PCDS_xyz = PCL;
 OPCL=PC.Location;
 pcshow(PC);
 title("Original Point Cloud");
 
 %%  Structure Mapping

PCN = pcdownsample(PC,"gridAverage",0.05); % Set based on level of resolution needed
PCL = PCN.Location;
pcshow(PCL);
title("Downsampled Point Cloud");


xyz = PCL;
pc  = pointCloud(PCL);

% ---- Compute normals ----
k = 50;                        % neighborhood size
normals = pcnormals(pc, k);

% ---- Orient normals outward from centroid ----
centroid = mean(xyz, 1);
p = xyz - centroid;                        % vector from centroid to each point
dotp = sum(normals .* p, 2);
normals(dotp < 0, :) = -normals(dotp < 0, :);

% Ensure unit length 
normals = normals ./ vecnorm(normals, 2, 2);

% ---- Dip Angle & Dip Direction ----
nx = normals(:,1);
ny = normals(:,2);
nz = normals(:,3);

% Dip Angle: angle between normal and vertical (Z), range 0-90
DA = acosd(abs(nz));  

% Dip Direction: compass bearing (0-360, clockwise from North)
DD = mod(atan2d(-nx, -ny), 360);

% Handle near-horizontal-normal (i.e., near-vertical/steep surfaces) 
tol = 1e-3;
flatIdx = (nx.^2 + ny.^2) < tol; 
DD(flatIdx) = 0; 

DAN = DA;
DDN = DD;

figure
pcshow(PCL,DDN);
colorbar
title('Dip Direction')
colormap(jet)
set(gcf,'Color','w');
set(gca, 'Color', 'w');  
figure
pcshow(PCL,DAN)
colorbar;
title('Dip Angle')
set(gcf,'Color','w');
set(gca, 'Color', 'w');  

DIP = [DAN DDN];
nradius = DAN/90;
txp = nradius .* sind(DDN);
typ = nradius .* cosd(DDN);
DIP_S = [txp, typ];

Stereonet_OwnFigureCode_FascillaAlternative;
scatter(DIP_S(:,1), DIP_S(:,2), 1, [0.5 0.5 0.5], 'filled');
title('Point-wise Orientation Plot')

%% Check with density graph

tempDA = DIP_S(:,1);
tempDD = DIP_S(:,2);
% Stereonet properties
%falsilla_riquelme; 
Stereonet_OwnFigureCode_FascillaAlternative;
hold on;

% Initialize variables for KDE grid
n_points = 400; % Number of grid points default = 400
theta = linspace(0, 2*pi, n_points); % Angular coordinates
radius = linspace(0, 1, n_points); % Radial coordinates
[Theta, Radius] = meshgrid(theta, radius);

% Convert grid to Cartesian coordinates
X_grid = Radius .* cos(Theta);
Y_grid = Radius .* sin(Theta);

% KDE bandwidth parameter
bandwidth = 0.05; % Adjust for smoothing

% Initialize density array
density = zeros(size(X_grid));

% Perform KDE for each data point
for i = 1:length(tempDA)

    txp = tempDA(i);
    typ = tempDD(i);

    % Compute Gaussian kernel contribution to the density
    dist = sqrt((X_grid - txp).^2 + (Y_grid - typ).^2);
    kernel = exp(-dist.^2 / (2 * bandwidth^2)); % Gaussian kernel
    density = density + kernel;
end

% Normalize density for consistent visualization
density = density / max(density(:));

% Create 3D contour plot with colored lines
[~, h] = contour3(X_grid, Y_grid, density, 100, 'LineWidth', 0.5);

% Apply colormap to contour lines
colormap(jet); % Use jet colormap (adjust as needed)
colorbar; % Add color bar for reference

% Adjust stereonet appearance
axis equal;
view(3); % Set to 3D view
zlabel('Kernel Density Estimation');
title('Kernel Density Estimation Plot');
grid on;

hold off;

%% Plotting

minPeakDensity = 0.30;
minSep_xy      = 1/3;
assignRadius   = 1/3;

insideCircle = (X_grid.^2 + Y_grid.^2) <= 1.0;

isMax = insideCircle & (density >= minPeakDensity);

nbrShifts = [ -1 -1; -1 0; -1 1;
               0 -1;        0 1;
               1 -1;  1 0;  1 1 ];

for s = 1:size(nbrShifts,1)
    dShift = circshift(density, nbrShifts(s,:));
    isMax = isMax & (density >= dShift);
end

strictlyGreaterSomeNeighbor = false(size(density));
for s = 1:size(nbrShifts,1)
    dShift = circshift(density, nbrShifts(s,:));
    strictlyGreaterSomeNeighbor = strictlyGreaterSomeNeighbor | (density > dShift);
end
isMax = isMax & strictlyGreaterSomeNeighbor;

candIdx = find(isMax);
candX   = X_grid(candIdx);
candY   = Y_grid(candIdx);
candV   = density(candIdx);

peakXY = zeros(0,2);
peakV  = zeros(0,1);

if ~isempty(candIdx)
    [candV_sorted, order] = sort(candV, 'descend');
    candX_sorted = candX(order);
    candY_sorted = candY(order);

    for k = 1:numel(candV_sorted)
        xk = candX_sorted(k);
        yk = candY_sorted(k);
        vk = candV_sorted(k);

        if isempty(peakXY)
            peakXY(end+1,:) = [xk yk];
            peakV(end+1,1)  = vk;
        else
            d = hypot(peakXY(:,1) - xk, peakXY(:,2) - yk);
            if all(d >= minSep_xy)
                peakXY(end+1,:) = [xk yk];
                peakV(end+1,1)  = vk;
            end
        end
    end
end

nPeaks = size(peakXY,1);

N = size(DIP_S,1);
pointLabel = zeros(N,1);
pointDist  = inf(N,1);

for p = 1:nPeaks
    d = hypot(DIP_S(:,1) - peakXY(p,1), DIP_S(:,2) - peakXY(p,2));
    within = (d <= assignRadius);
    better = within & (d < pointDist);
    pointDist(better)  = d(better);
    pointLabel(better) = p;
end

Stereonet_OwnFigureCode_FascillaAlternative;
hold on;

cmap = jet(nPeaks);
nCircle = 720;
t = linspace(0,2*pi,nCircle).';
unitPoly = polyshape(cos(t), sin(t));

for i = 1:nPeaks
    ci = peakXY(i,:);
    P = [ci(1) + assignRadius*cos(t), ci(2) + assignRadius*sin(t)];

    for j = 1:nPeaks
        if j == i, continue; end
        cj = peakXY(j,:);

        a = cj(1) - ci(1);
        b = cj(2) - ci(2);
        c = (cj(1)^2 + cj(2)^2 - ci(1)^2 - ci(2)^2)/2;

        P = clipPolyHalfPlane(P,a,b,c);
        if size(P,1) < 3, break; end
    end

    if size(P,1) < 3, continue; end
    R = intersect(polyshape(P(:,1),P(:,2)), unitPoly);
    if isempty(R.Vertices), continue; end

    plot(R,'FaceColor',cmap(i,:),'EdgeColor','none','FaceAlpha',1.0);
end

colormap(cmap);
caxis([1 nPeaks]);
cb = colorbar;
cb.Label.String = 'Discontinuity Set ID';
cb.Ticks = 1:nPeaks;
cb.TickLabels = string(1:nPeaks);

title('Detected Discontinuity Sets');
axis equal;
xlim([-1.2 1.2]); ylim([-1.2 1.2]);
set(gcf,'Color','w'); set(gca,'Color','w');
hold off;

% Save detected discontinuity sets figure
exportgraphics(gcf, fullfile(outputFolder, 'Detected Discontinuity Sets.jpg'), 'Resolution', 300);

assigned = pointLabel > 0;

if any(assigned)
    cmap = jet(nPeaks);
    P_assigned = PCL(assigned,:);
    C_assigned = cmap(pointLabel(assigned),:);

    % figure;
    % pcshow(pointCloud(P_assigned,'Color',C_assigned));
    % title('Point Cloud – Discontinuity Sets Only - Before Filtering');
    % colormap(cmap);
    % caxis([1 nPeaks]);
    % cb = colorbar;
    % cb.Label.String = 'Discontinuity Set ID';
    % cb.Ticks = 1:nPeaks;
    % cb.TickLabels = string(1:nPeaks);
    % set(gcf,'Color','w'); set(gca,'Color','w');
end

% Connected Components

% Parameters
gridStep  = 0.10;     % Connected-component voxel size (m)
minPoints = 50;       % Minimum actual cloud points per component


% Output variables

DIP_S_Planes = [];       % Stereonet coordinates of each plane
DIP_Planes   = [];       % [DA DD] of each plane
kmap_Planes  = [];       % Original discontinuity set ID

Centroid_Planes = [];    % XYZ representative centroid point
ComponentSize   = [];    % Number of points in each plane


% Variables for plotting identified plane points

% TRUE = point belongs to a retained plane
planePointMask = false(size(PCL,1),1);

% Original discontinuity set ID of retained plane points
% 0 = not part of a retained plane
planePointKmap = zeros(size(PCL,1),1);

for clusterID = 1:nPeaks

    % Points belonging to current discontinuity set
    clusterIdx = find(pointLabel == clusterID);

    if isempty(clusterIdx)
        continue
    end

    xyzCluster = PCL(clusterIdx,:);


    % Voxelise cluster points

    voxelXYZ = floor(xyzCluster ./ gridStep);

    % Shift voxel indices to start at 1
    minVoxel = min(voxelXYZ,[],1);
    voxelXYZ = voxelXYZ - minVoxel + 1;

    % Size of voxel grid
    gridSize = max(voxelXYZ,[],1);


    % Create binary voxel grid

    BW = false(gridSize);

    linearIdx = sub2ind(gridSize, ...
        voxelXYZ(:,1), ...
        voxelXYZ(:,2), ...
        voxelXYZ(:,3));

    BW(linearIdx) = true;


    % Connected components using 26-connectivity

    CC = bwconncomp(BW,26);


    % Process each connected component

    for componentID = 1:CC.NumObjects

        % Voxels belonging to current component
        componentVoxels = CC.PixelIdxList{componentID};

        % Find actual cloud points occupying those voxels
        pointMask = ismember(linearIdx,componentVoxels);

        localPointIdx = find(pointMask);


        % Minimum point criterion

        if numel(localPointIdx) < minPoints
            continue
        end


        % Convert back to original point-cloud indices

        originalIdx = clusterIdx(localPointIdx);

        xyzComponent = PCL(originalIdx,:);


        % Mark all points belonging to retained plane

        planePointMask(originalIdx) = true;

        % Preserve original discontinuity set ID
        planePointKmap(originalIdx) = clusterID;


        % Calculate geometric centroid

        geometricCentroid = mean(xyzComponent,1);


        % Find actual cloud point nearest to centroid

        distanceToCentroid = vecnorm( ...
            xyzComponent - geometricCentroid,2,2);

        [~,nearestLocal] = min(distanceToCentroid);

        centroidOriginalIdx = originalIdx(nearestLocal);


        % Store plane information

        Centroid_Planes(end+1,:) = ...
            PCL(centroidOriginalIdx,:);

        DIP_Planes(end+1,:) = ...
            DIP(centroidOriginalIdx,:);

        DIP_S_Planes(end+1,:) = ...
            DIP_S(centroidOriginalIdx,:);

        kmap_Planes(end+1,1) = ...
            clusterID;

        ComponentSize(end+1,1) = ...
            numel(originalIdx);

    end

end


% Original point cloud
PC_original = PCN.Location;

% Retain only points belonging to identified planes
P_planes = PCL(planePointMask,:);

% Original discontinuity set IDs of retained points
setIDs = planePointKmap(planePointMask);

% Same colours as original discontinuity sets
clusterColors = jet(nPeaks);

% Assign RGB colours according to set ID
pointColors = clusterColors(setIDs,:);

% Create point cloud
PC_planes = pointCloud(P_planes, ...
    'Color', uint8(pointColors*255));

% Visualise
figure;
pcshow(PC_planes);

axis equal;

xlabel('X');
ylabel('Y');
zlabel('Z');

title('Identified Discontinuities Coloured by Set');

colormap(clusterColors);
clim([0.5 nPeaks+0.5]);

cb = colorbar;
cb.Label.String = 'Discontinuity Set ID';
cb.Ticks = 1:nPeaks;
cb.TickLabels = string(1:nPeaks);

set(gcf,'Color','w');
set(gca,'Color','w');

view(3);

% Save final filtered planes figure
exportgraphics(gcf, fullfile(outputFolder, ...
    'Identified Discontinuities Coloured by Set.jpg'), ...
    'Resolution', 300);


function Pout = clipPolyHalfPlane(P,a,b,c)
    inside = @(x,y) (a*x + b*y <= c + 1e-12);
    Pout = [];
    for k = 1:size(P,1)
        A = P(k,:);
        B = P(mod(k,size(P,1))+1,:);
        Ain = inside(A(1),A(2));
        Bin = inside(B(1),B(2));
        if Ain && Bin
            Pout(end+1,:) = B;
        elseif Ain && ~Bin
            I = intersectLine(A,B,a,b,c);
            Pout(end+1,:) = I;
        elseif ~Ain && Bin
            I = intersectLine(A,B,a,b,c);
            Pout(end+1,:) = I;
            Pout(end+1,:) = B;
        end
    end
end

function I = intersectLine(A,B,a,b,c)
    fA = a*A(1)+b*A(2)-c;
    fB = a*B(1)+b*B(2)-c;
    t = fA/(fA-fB);
    t = max(0,min(1,t));
    I = A + t*(B-A);
end
