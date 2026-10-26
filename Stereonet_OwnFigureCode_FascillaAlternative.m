figure; hold on; 
axis equal off;

theta = linspace(0, 2*pi, 400);

% Original radii: 1/6, 2/6, 3/6, 4/6, 5/6
radii_all = linspace(1/6, 1, 6);
r_outer = 1.0;

% Keep only the radii for 30°, 60°, and 90°
r_30 = radii_all(2);   % 2/6
r_60 = radii_all(4);   % 4/6
r_90 = radii_all(6);   % 6/6 (outer)

% Plot only these circles

% 30° circle (grey dotted)
plot(r_30*cos(theta), r_30*sin(theta), ...
     'Color', [0.6 0.6 0.6], 'LineStyle', ':', 'LineWidth', 1.4);

% 60° circle (grey dotted)
plot(r_60*cos(theta), r_60*sin(theta), ...
     'Color', [0.6 0.6 0.6], 'LineStyle', ':', 'LineWidth', 1.4);

% 90° outer circle (black dotted, bold)
plot(r_90*cos(theta), r_90*sin(theta), ...
     'k:', 'LineWidth', 2);

% Limits
xlim([-1.2 1.2]);
ylim([-1.2 1.2]);

% Dip angle labels on +x-axis (only 30, 60, 90)
dip_angles = [30 60 90];

% Matching radii for these angles
radii_keep = [r_30, r_60, r_90];  % from previous circle code

for i = 1:numel(dip_angles)
    x_pos = radii_keep(i) * 1.03;  % slight outward offset (keeps ° visible)
    text(x_pos, 0, sprintf('%d°', dip_angles(i)), ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom', ...
        'FontSize', 10);
end

% --- Radial dashed lines (grey, same thickness) & dip direction labels ---

radial_dd_deg = 0:30:330;                % dip direction labels: 0..330
radial_angles = deg2rad(90 - radial_dd_deg);  % rotate so 0° at +y axis

% Radial lines from origin to outer circle
for k = 1:length(radial_angles)
    ang = radial_angles(k);
    xx = [0, r_outer * cos(ang)];
    yy = [0, r_outer * sin(ang)];
    plot(xx, yy, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.4);
end

% Azimuth / dip direction labels just outside outer circle
label_radius = 1.13;   % already bumped out a bit

for k = 1:length(radial_angles)
    ang = radial_angles(k);
    xL = label_radius * cos(ang);
    yL = label_radius * sin(ang);

    text(xL, yL, sprintf('%d°', radial_dd_deg(k)), ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', ...
        'FontSize', 10);
end
set(gcf, 'Color', 'w');  % Figure background
set(gca, 'Color', 'w');  % Axes background
hold on;
