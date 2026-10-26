function rov_sim(varargin)
%ROV_SIM  3D simulation of the 3D-printed mini ROV with battery model and settings.
%
%   rov_sim            run the simulation and play the animation
%   rov_sim('video')   also record the first run to rov_sim.mp4
%
%   Needs base MATLAB only (R2016b or newer). No toolboxes.
%
%   Settings panel (bottom left). Change values, then press "Apply and re-run":
%     Environment    Shallow (seabed 6 m) / Medium (seabed 15 m)
%     Water quality  Clear / Moderate / Turbid / Very turbid
%                    -> camera visibility = AI detection range, more light power
%     Power mode     Eco / Normal / Performance (cruise speed, lights, video, AI)
%     4G uplink      off / only at the surface / always on
%     Controller gains  depth PID, heading PD, speed gain
%     Battery        capacity (mAh), voltage (V), usable fraction (%)
%
%   Model: 2 rear horizontal thrusters + 1 vertical thruster. Electrical power is
%   thrusters (~thrust^1.5) + Raspberry Pi + AI + lights + 4G modem + ESC, divided
%   by converter efficiency. Battery state of charge and runtime come from that.
%
%   Mission: 1 DIVE -> 2 SEARCH -> 3 AI TRACKING -> 4 RETURN -> 5 ASCEND -> 6 SURFACED
%            (below 20% battery the vehicle returns early; at 0% it floats up)
%
%   All numbers (mass, drag, thrust, power) are assumptions. Replace them with
%   values measured in your own tests.

saveVideo = any(strcmpi(varargin, 'video'));
cfg = defaultCfg();
sz = get(0, 'ScreenSize');
figPos = [sz(1) + 15, sz(2) + 45, sz(3) - 30, sz(4) - 110];
while true
    [again, cfg, figPos] = runOnce(cfg, saveVideo, figPos);
    saveVideo = false;
    if ~again, break; end
end
end

%% ================================================================ main run
function [again, cfg, figPos] = runOnce(cfg, saveVideo, figPos)

P = params(cfg);
L = simulate(P);
S = summarize(L, P);
N = numel(L.t);

modeNames = {'1  DIVE', '2  SEARCH (waypoints)', '3  AI TRACKING target', ...
             '4  RETURN to base', '5  ASCEND', '6  SURFACED - 4G uplink', ...
             '7  BATTERY EMPTY - floating up'};
modeCols  = [0.35 0.55 0.90; 0.40 0.80 0.50; 1.00 0.60 0.20; ...
             0.70 0.50 0.90; 0.30 0.80 0.85; 0.90 0.85 0.30; 0.90 0.30 0.30];
pnames = {'thrust', 'Pi', 'AI', 'lights', '4G', 'ESC', 'loss'};
pcols  = [1.00 0.60 0.30; 0.40 0.80 0.50; 0.70 0.50 0.90; 1.00 0.90 0.30; ...
          0.40 0.80 0.90; 0.60 0.60 0.65; 0.80 0.35 0.35];

bgc    = [0.07 0.09 0.12];
txtCol = [0.92 0.94 0.96];
axCol  = [0.10 0.13 0.18];
dimCol = [0.70 0.75 0.80];
sceneCols = [0.04 0.17 0.28; 0.06 0.20 0.26; 0.12 0.20 0.17; 0.17 0.19 0.12];
hazeCols  = [0.30 0.50 0.60; 0.35 0.50 0.45; 0.40 0.45 0.30; 0.42 0.38 0.26];
hazeAlpha = [0.0 0.15 0.35 0.55];

%% ------------------------------------------------------------ figure
fig = figure('Name', 'Mini-ROV simulation', 'NumberTitle', 'off', ...
             'Color', bgc, 'Position', figPos);

% ---- main 3D axes (follows the vehicle)
p3d = uipanel(fig, 'Units', 'normalized', 'Position', [0.005 0.43 0.45 0.56], ...
              'BackgroundColor', bgc, 'BorderType', 'line', 'HighlightColor', [0.25 0.30 0.38]);
ax3 = axes('Parent', p3d, 'Position', [0 0 1 1]);
hold(ax3, 'on');
set(ax3, 'Color', sceneCols(cfg.wq, :), 'XColor', 'none', 'YColor', 'none', ...
         'ZColor', 'none', 'DataAspectRatio', [1 1 1], ...
         'XLim', [-7 22], 'YLim', [-11 12], 'ZLim', [P.seabedZ - 0.7, 1.2], ...
         'Projection', 'perspective', 'CameraViewAngle', 40, ...
         'CameraViewAngleMode', 'manual');
buildScene(ax3, P);
fish = buildFish(ax3);
V = buildVehicle(ax3, min(P.detR, 4));
hTrail  = plot3(ax3, nan, nan, nan, '-', 'Color', [0.55 1.00 0.80], 'LineWidth', 1.3);
hTether = plot3(ax3, nan, nan, nan, '-', 'Color', [1.00 0.85 0.20], 'LineWidth', 1.8);
hFloats = plot3(ax3, nan, nan, nan, 'o', 'LineStyle', 'none', 'MarkerSize', 5, ...
                'MarkerFaceColor', [1.00 0.40 0.10], 'MarkerEdgeColor', 'none');
light(ax3, 'Position', [6 -6 12], 'Style', 'infinite');
light(ax3, 'Position', [-8 8 6], 'Style', 'infinite', 'Color', [0.35 0.4 0.5]);
lighting(ax3, 'gouraud');

% ---- mission map (top view)
axM = axes('Parent', fig, 'Position', [0.505 0.665 0.200 0.265]);
hold(axM, 'on');
styleAx(axM, axCol, dimCol);
axis(axM, 'equal');
set(axM, 'XLim', [-3 15], 'YLim', [-6 8]);
title(axM, 'Mission map (top view)', 'Color', txtCol, 'FontSize', 10, 'FontWeight', 'normal');
xlabel(axM, 'East (m)', 'Color', dimCol);
ylabel(axM, 'North (m)', 'Color', dimCol);
tt = linspace(0, 2*pi, 80);
plot(axM, P.tgtC(1) + P.tgtR*cos(tt), P.tgtC(2) + P.tgtR*sin(tt), '--', 'Color', [1 0.6 0.2]);
plot(axM, P.WP(:,1), P.WP(:,2), 'd', 'MarkerSize', 8, 'MarkerFaceColor', [0.3 1 0.5], ...
     'MarkerEdgeColor', 'w');
plot(axM, P.anchor(1), P.anchor(2), 's', 'MarkerSize', 9, 'MarkerFaceColor', [1 0.4 0.1], ...
     'MarkerEdgeColor', 'w');
text(axM, P.anchor(1) + 0.4, P.anchor(2) + 0.9, 'base buoy', 'Color', dimCol, 'FontSize', 9);
hMTrail = plot(axM, nan, nan, '-', 'Color', [0.55 1 0.8], 'LineWidth', 1.3);
hMFov   = patch(axM, nan, nan, [1 0.9 0.3], 'FaceAlpha', 0.18, 'EdgeColor', 'none');
hMDet   = plot(axM, nan, nan, '-', 'Color', [0.3 1 0.4], 'LineWidth', 1.5);
hMVeh   = patch(axM, nan, nan, [0.9 0.95 1], 'EdgeColor', 'k');
hMTgt   = plot(axM, nan, nan, 'o', 'MarkerSize', 9, 'MarkerFaceColor', [1 0.55 0.1], ...
               'MarkerEdgeColor', 'w');

% ---- onboard camera view with AI overlay
axC = axes('Parent', fig, 'Position', [0.765 0.665 0.225 0.265]);
hold(axC, 'on');
set(axC, 'XLim', [-1 1], 'YLim', [-1 1], 'XTick', [], 'YTick', [], 'Box', 'on', ...
         'XColor', dimCol, 'YColor', dimCol, 'Color', axCol);
title(axC, 'Onboard camera + AI (simulated detection)', 'Color', txtCol, ...
      'FontSize', 10, 'FontWeight', 'normal');
hCBg   = patch(axC, [-1 1 1 -1], [-1 -1 1 1], [0.1 0.4 0.58], 'EdgeColor', 'none');
hCSand = patch(axC, [-1 1 1 -1], [-1 -1 -1 -1], [0.72 0.64 0.45], 'EdgeColor', 'none');
patch(axC, [-1 1 1 -1], [-1 -1 1 1], hazeCols(cfg.wq, :), 'FaceAlpha', hazeAlpha(cfg.wq), ...
      'EdgeColor', 'none');
plot(axC, [-0.06 0.06], [0 0], '-', 'Color', [1 1 1], 'LineWidth', 0.8);
plot(axC, [0 0], [-0.08 0.08], '-', 'Color', [1 1 1], 'LineWidth', 0.8);
hCFish = patch(axC, nan, nan, [1 0.5 0.1], 'EdgeColor', 'none');
hCBox  = plot(axC, nan, nan, '-', 'Color', [0.3 1 0.4], 'LineWidth', 2);
hCLbl  = text(axC, 0, 0, '', 'Color', [0.3 1 0.4], 'FontSize', 10, ...
              'FontWeight', 'bold', 'VerticalAlignment', 'bottom');
hCStat = text(axC, -0.97, 0.90, '', 'Color', txtCol, 'FontSize', 10, ...
              'FontWeight', 'bold', 'VerticalAlignment', 'top');
hCVis  = text(axC, 0.97, 0.90, sprintf('visibility %.1f m', P.vis), 'Color', txtCol, ...
              'FontSize', 9, 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top');
hCFlash = text(axC, 0.97, -0.90, '', 'Color', [1 0.85 0.2], 'FontSize', 10, ...
               'FontWeight', 'bold', 'HorizontalAlignment', 'right', ...
               'VerticalAlignment', 'bottom');

% ---- analysis plot (selectable)
uicontrol(fig, 'Style', 'text', 'String', 'Plot:', 'Units', 'normalized', ...
          'Position', [0.475 0.555 0.035 0.028], 'BackgroundColor', bgc, ...
          'ForegroundColor', txtCol, 'HorizontalAlignment', 'right');
pAn = uicontrol(fig, 'Style', 'popupmenu', 'Units', 'normalized', ...
                'String', {'Depth', 'Thrusters', 'Power (stacked)', 'Battery endurance'}, ...
                'Value', 3, 'Position', [0.515 0.552 0.17 0.035], 'Callback', @onAn);
axA = axes('Parent', fig, 'Position', [0.52 0.405 0.465 0.125]);
styleAx(axA, axCol, dimCol);
hCurA = [];
hDotA = [];

% ---- HUD overlay and results text (transparent axes on top)
axH = axes('Parent', p3d, 'Position', [0 0 1 1], 'Visible', 'off', ...
           'XLim', [0 1], 'YLim', [0 1], 'HitTest', 'off');
hHud1 = text(axH, 0.015, 0.980, '', 'Color', txtCol, 'FontSize', 12, ...
             'FontWeight', 'bold', 'VerticalAlignment', 'top');
hHud2 = text(axH, 0.015, 0.925, '', 'Color', dimCol, 'FontSize', 9.5, 'VerticalAlignment', 'top');
hHud3 = text(axH, 0.015, 0.882, '', 'Color', dimCol, 'FontSize', 9.5, 'VerticalAlignment', 'top');
hHud4 = text(axH, 0.015, 0.839, '', 'Color', dimCol, 'FontSize', 9.5, 'VerticalAlignment', 'top');
rp = uipanel(fig, 'Title', 'Results', 'Units', 'normalized', 'Position', [0.47 0.065 0.525 0.27], ...
             'BackgroundColor', bgc, 'ForegroundColor', txtCol, 'FontSize', 9);
uicontrol(rp, 'Style', 'text', 'String', resultLines(P, S, cfg), 'Units', 'normalized', ...
          'Position', [0.01 0.02 0.98 0.94], 'BackgroundColor', bgc, 'ForegroundColor', txtCol, ...
          'HorizontalAlignment', 'left', 'FontName', get(0, 'FixedWidthFontName'), 'FontSize', 9);

% ---- settings panel
pnl = uipanel(fig, 'Title', 'Settings  (change values, then press  Apply and re-run)', ...
              'Units', 'normalized', 'Position', [0.005 0.065 0.45 0.355], ...
              'BackgroundColor', bgc, 'ForegroundColor', txtCol, 'FontSize', 9);
rowsY = [0.72 0.58 0.44 0.30 0.16 0.02];
rh = 0.12;
ui(pnl, 'text', 'ENVIRONMENT', [0.01 0.86 0.33 0.11], 'bold');
ui(pnl, 'text', 'Environment', [0.01 rowsY(1) 0.125 rh]);
ui(pnl, 'text', 'Water quality', [0.01 rowsY(2) 0.125 rh]);
ui(pnl, 'text', 'Power mode', [0.01 rowsY(3) 0.125 rh]);
ui(pnl, 'text', '4G uplink', [0.01 rowsY(4) 0.125 rh]);
pEnv = ui(pnl, 'popup', {'Shallow (seabed 6 m)', 'Medium (seabed 15 m)'}, [0.14 rowsY(1) 0.20 rh]);
pWq  = ui(pnl, 'popup', {'Clear (8 m visibility)', 'Moderate (5 m)', 'Turbid (3 m)', ...
                         'Very turbid (1.5 m)'}, [0.14 rowsY(2) 0.20 rh]);
pPm  = ui(pnl, 'popup', {'Eco (0.4 m/s)', 'Normal (0.6 m/s)', 'Performance (0.9 m/s)'}, ...
          [0.14 rowsY(3) 0.20 rh]);
pCm  = ui(pnl, 'popup', {'4G off', '4G on at surface only', '4G always on'}, ...
          [0.14 rowsY(4) 0.20 rh]);
set(pEnv, 'Value', cfg.env);  set(pWq, 'Value', cfg.wq);
set(pPm, 'Value', cfg.pm);    set(pCm, 'Value', cfg.comm);
ui(pnl, 'text', 'Turbid water cuts the AI range and needs brighter lights. Always-on 4G wastes power.', ...
   [0.01 0.02 0.33 0.22], 'dim');

ui(pnl, 'text', 'CONTROLLER GAINS', [0.36 0.86 0.30 0.11], 'bold');
ui(pnl, 'text', 'Depth Kp', [0.36 rowsY(1) 0.15 rh]);
ui(pnl, 'text', 'Depth Ki', [0.36 rowsY(2) 0.15 rh]);
ui(pnl, 'text', 'Depth Kd', [0.36 rowsY(3) 0.15 rh]);
ui(pnl, 'text', 'Heading Kp', [0.36 rowsY(4) 0.15 rh]);
ui(pnl, 'text', 'Heading Kd', [0.36 rowsY(5) 0.15 rh]);
ui(pnl, 'text', 'Speed gain', [0.36 rowsY(6) 0.15 rh]);
eKz1 = ui(pnl, 'edit', sprintf('%g', cfg.kz(1)),   [0.52 rowsY(1) 0.13 rh]);
eKz2 = ui(pnl, 'edit', sprintf('%g', cfg.kz(2)),   [0.52 rowsY(2) 0.13 rh]);
eKz3 = ui(pnl, 'edit', sprintf('%g', cfg.kz(3)),   [0.52 rowsY(3) 0.13 rh]);
eKp1 = ui(pnl, 'edit', sprintf('%g', cfg.kpsi(1)), [0.52 rowsY(4) 0.13 rh]);
eKp2 = ui(pnl, 'edit', sprintf('%g', cfg.kpsi(2)), [0.52 rowsY(5) 0.13 rh]);
eKu  = ui(pnl, 'edit', sprintf('%g', cfg.ku),      [0.52 rowsY(6) 0.13 rh]);

ui(pnl, 'text', 'BATTERY', [0.68 0.86 0.30 0.11], 'bold');
ui(pnl, 'text', 'Capacity mAh', [0.68 rowsY(1) 0.17 rh]);
ui(pnl, 'text', 'Voltage V', [0.68 rowsY(2) 0.17 rh]);
ui(pnl, 'text', 'Usable %', [0.68 rowsY(3) 0.17 rh]);
eCap  = ui(pnl, 'edit', sprintf('%g', cfg.cap),    [0.86 rowsY(1) 0.13 rh]);
eVolt = ui(pnl, 'edit', sprintf('%g', cfg.volt),   [0.86 rowsY(2) 0.13 rh]);
eUse  = ui(pnl, 'edit', sprintf('%g', cfg.usable), [0.86 rowsY(3) 0.13 rh]);
bApply = ui(pnl, 'push', 'Apply and re-run', [0.68 0.20 0.31 0.15]);
bReset = ui(pnl, 'push', 'Reset defaults', [0.68 0.02 0.31 0.14]);
set(bApply, 'Callback', @onApply);
set(bReset, 'Callback', @onReset);
hMsg = ui(pnl, 'text', '', [0.68 0.37 0.31 0.06], 'warn');

% ---- playback controls
btn = uicontrol(fig, 'Style', 'pushbutton', 'String', 'Pause', 'Units', 'normalized', ...
                'Position', [0.008 0.012 0.06 0.045], 'Callback', @onPlay);
sld = uicontrol(fig, 'Style', 'slider', 'Min', 1, 'Max', N, 'Value', 1, ...
                'SliderStep', [1/(N-1) 0.05], 'Units', 'normalized', ...
                'Position', [0.075 0.022 0.24 0.028], 'Callback', @onSlide);
uicontrol(fig, 'Style', 'text', 'String', 'Speed', 'Units', 'normalized', ...
          'Position', [0.320 0.018 0.035 0.030], 'BackgroundColor', bgc, ...
          'ForegroundColor', txtCol, 'HorizontalAlignment', 'right');
pop = uicontrol(fig, 'Style', 'popupmenu', 'String', {'0.5x', '1x', '2x', '4x'}, ...
                'Value', 2, 'Units', 'normalized', 'Position', [0.358 0.016 0.05 0.040], ...
                'Callback', @onSpeed);
uicontrol(fig, 'Style', 'text', 'String', 'View', 'Units', 'normalized', ...
          'Position', [0.415 0.018 0.035 0.030], 'BackgroundColor', bgc, ...
          'ForegroundColor', txtCol, 'HorizontalAlignment', 'right');
vpop = uicontrol(fig, 'Style', 'popupmenu', 'String', {'Chase', 'Side', 'Front', 'Top'}, ...
                 'Value', 1, 'Units', 'normalized', 'Position', [0.452 0.016 0.06 0.040], ...
                 'Callback', @onView);
bRep = uicontrol(fig, 'Style', 'pushbutton', 'String', 'Save report', 'Units', 'normalized', ...
                 'Position', [0.52 0.012 0.085 0.045], 'Callback', @onReport);

%% ------------------------------------------------------------ state
playing = true;
restart = false;
speedFactor = 1;
tcur = 0;
k = 1;
lastK = 1;
camYaw = L.psi(1);
viewIdx = 1;
camOff = [-3.2 -2.2 1.0; 0.0 -4.0 0.3; 3.6 -1.8 0.5; 0.0 0.0 6.0];

drawAnalysis();
render(1);

if saveVideo
    vw = VideoWriter('rov_sim.mp4', 'MPEG-4');
    vw.FrameRate = round(1/P.dt);
    open(vw);
    for kk = 1:N
        render(kk);
        drawnow;
        writeVideo(vw, getframe(fig));
    end
    close(vw);
    fprintf('Saved rov_sim.mp4\n');
    render(1);
end

lastTic = tic;
while ishandle(fig) && ~restart
    dtw = toc(lastTic);
    lastTic = tic;
    if playing
        tcur = tcur + dtw*speedFactor;
        k = min(N, 1 + floor(tcur/P.dt));
        if k >= N
            playing = false;
            set(btn, 'String', 'Replay');
        end
        if ~ishandle(fig), break; end
        set(sld, 'Value', k);
        render(k);
    end
    drawnow limitrate;
    if ~ishandle(fig), break; end
    pause(0.004);
end

again = false;
if restart && ishandle(fig)
    figPos = get(fig, 'Position');
    delete(fig);
    again = true;
elseif ishandle(fig)
    figPos = get(fig, 'Position');
end

%% ============================================================ nested
    function onPlay(~, ~)
        if k >= N
            tcur = 0;
            k = 1;
            playing = true;
            set(btn, 'String', 'Pause');
        elseif playing
            playing = false;
            set(btn, 'String', 'Play');
        else
            playing = true;
            set(btn, 'String', 'Pause');
        end
    end

    function onSlide(~, ~)
        playing = false;
        set(btn, 'String', 'Play');
        k = max(1, min(N, round(get(sld, 'Value'))));
        tcur = (k - 1)*P.dt;
        render(k);
    end

    function onSpeed(~, ~)
        sp = [0.5 1 2 4];
        speedFactor = sp(get(pop, 'Value'));
    end

    function onView(~, ~)
        viewIdx = get(vpop, 'Value');
        render(k);
    end

    function onAn(~, ~)
        drawAnalysis();
        render(k);
    end

    function onApply(~, ~)
        c = cfg;
        c.env = get(pEnv, 'Value');
        c.wq = get(pWq, 'Value');
        c.pm = get(pPm, 'Value');
        c.comm = get(pCm, 'Value');
        v = [str2double(get(eKz1, 'String')), str2double(get(eKz2, 'String')), ...
             str2double(get(eKz3, 'String')), str2double(get(eKp1, 'String')), ...
             str2double(get(eKp2, 'String')), str2double(get(eKu, 'String')), ...
             str2double(get(eCap, 'String')), str2double(get(eVolt, 'String')), ...
             str2double(get(eUse, 'String'))];
        if any(~isfinite(v)) || any(v < 0) || v(7) <= 0 || v(8) <= 0 || v(9) <= 0 || v(9) > 100
            set(hMsg, 'String', 'Invalid value: use numbers >= 0 (usable 1-100).');
            return;
        end
        c.kz = v(1:3);
        c.kpsi = v(4:5);
        c.ku = v(6);
        c.cap = v(7);
        c.volt = v(8);
        c.usable = v(9);
        cfg = c;
        restart = true;
    end

    function onReset(~, ~)
        cfg = defaultCfg();
        restart = true;
    end

    function onReport(~, ~)
        set(bRep, 'String', 'Saving...');
        drawnow;
        try
            folder = makeReport(P, L, S, cfg, fig);
            set(bRep, 'String', 'Save report');
            msgbox(['Report saved in:  ' folder char(10) 'Open report.html (it also opens in your browser). ' ...
                    'Use Print > Save as PDF in the browser to get a PDF.'], 'Report');
        catch ME
            set(bRep, 'String', 'Save report');
            errordlg(['Could not save the report: ' ME.message], 'Report');
        end
    end

    function drawAnalysis()
        legend(axA, 'off');
        cla(axA);
        hold(axA, 'on');
        hCurA = [];
        hDotA = [];
        tEnd = L.t(end);
        switch get(pAn, 'Value')
            case 1
                yl = [floor(min(L.z)) - 0.5, 0.4];
                drawBands(axA, L, modeCols, yl(1), yl(2));
                plot(axA, L.t, L.zref, '--', 'Color', [0.75 0.75 0.75], 'LineWidth', 1);
                plot(axA, L.t, L.z, '-', 'Color', [0.5 0.9 1], 'LineWidth', 1.6);
                hCurA = plot(axA, [0 0], yl, '-', 'Color', 'w', 'LineWidth', 1.2);
                set(axA, 'XLim', [0 tEnd], 'YLim', yl);
                ylabel(axA, 'Depth z (m)', 'Color', dimCol);
                xlabel(axA, 'Time (s)   dashed = governed reference', 'Color', dimCol);
            case 2
                yl = [-110 110];
                drawBands(axA, L, modeCols, yl(1), yl(2));
                plot(axA, L.t, 100*L.TL/P.Tthr, '-', 'Color', [1.0 0.6 0.3], 'LineWidth', 1.2);
                plot(axA, L.t, 100*L.TR/P.Tthr, '-', 'Color', [0.4 1.0 0.5], 'LineWidth', 1.2);
                plot(axA, L.t, 100*L.TV/P.Tthr, '-', 'Color', [0.5 0.7 1.0], 'LineWidth', 1.2);
                hCurA = plot(axA, [0 0], yl, '-', 'Color', 'w', 'LineWidth', 1.2);
                set(axA, 'XLim', [0 tEnd], 'YLim', yl);
                ylabel(axA, 'Thrust (%)', 'Color', dimCol);
                xlabel(axA, 'Time (s)   orange = left, green = right, blue = vertical', 'Color', dimCol);
            case 3
                yl = [0 1.4*max(L.Pbat)];
                hA = area(axA, L.t, L.P');
                for i = 1:7
                    set(hA(i), 'FaceColor', pcols(i, :), 'EdgeColor', 'none');
                end
                for i = 1:7
                    text(axA, 0.01 + 0.135*(i - 1), 0.97, pnames{i}, 'Units', 'normalized', ...
                         'Color', pcols(i, :), 'FontSize', 8, 'FontWeight', 'bold', ...
                         'VerticalAlignment', 'top');
                end
                hCurA = plot(axA, [0 0], yl, '-', 'Color', 'w', 'LineWidth', 1.2);
                set(axA, 'XLim', [0 tEnd], 'YLim', yl);
                ylabel(axA, 'Battery power (W)', 'Color', dimCol);
                xlabel(axA, 'Time (s)', 'Color', dimCol);
            otherwise
                Tm = L.t(end);
                tr = linspace(0, max(S.runMin*60, Tm), 600);
                nC = floor(tr/Tm);
                Ec = nC*L.E(end) + interp1(L.t, L.E, mod(tr, Tm), 'linear', 'extrap');
                sc = max(0, 100*(1 - Ec/P.EcapWh));
                plot(axA, tr/60, sc, '-', 'Color', [0.5 1 0.6], 'LineWidth', 1.8);
                plot(axA, [0 max(tr)/60], [20 20], '--', 'Color', [1 0.6 0.3]);
                hDotA = plot(axA, 0, 100, 'o', 'MarkerFaceColor', 'w', ...
                             'MarkerEdgeColor', 'none', 'MarkerSize', 7);
                set(axA, 'XLim', [0 max(tr)/60], 'YLim', [0 100]);
                text(axA, 0.98, 0.95, sprintf('empty after about %.0f min', S.runMin), ...
                     'Units', 'normalized', 'HorizontalAlignment', 'right', ...
                     'VerticalAlignment', 'top', 'Color', txtCol, 'FontSize', 9);
                ylabel(axA, 'Battery (%)', 'Color', dimCol);
                xlabel(axA, 'Time (min), mission repeated back-to-back   dashed = 20% reserve', ...
                       'Color', dimCol);
        end
        grid(axA, 'on');
    end

    function render(kr)
        t = L.t(kr);
        pos = [L.x(kr) L.y(kr) L.z(kr)];
        psi = L.psi(kr);

        % ---------- vehicle pose
        M = makehgtform('translate', pos) * makehgtform('zrotate', psi) * ...
            makehgtform('yrotate', L.theta(kr)) * makehgtform('xrotate', L.phi(kr));
        set(V.h, 'Matrix', M);

        % propellers
        set(V.spin(1), 'Matrix', makehgtform('xrotate', L.angL(kr)));
        set(V.spin(2), 'Matrix', makehgtform('xrotate', L.angR(kr)));
        set(V.spin(3), 'Matrix', makehgtform('xrotate', L.angV(kr)));

        % water jets (length ~ thrust)
        TLn = L.TL(kr)/P.Tthr;  TRn = L.TR(kr)/P.Tthr;  TVn = L.TV(kr)/P.Tthr;
        onf = {'off', 'on'};
        set(V.jetL, 'XData', [-0.31, -0.31 - 0.35*TLn], 'Visible', onf{(abs(TLn) > 0.08) + 1});
        set(V.jetR, 'XData', [-0.31, -0.31 - 0.35*TRn], 'Visible', onf{(abs(TRn) > 0.08) + 1});
        set(V.jetV, 'ZData', [0.15, 0.15 - 0.30*TVn], 'Visible', onf{(abs(TVn) > 0.08) + 1});

        % camera FOV pyramid colour
        if L.det(kr)
            set(V.fov, 'FaceColor', [0.3 1 0.4], 'EdgeColor', [0.3 1 0.4]);
        else
            set(V.fov, 'FaceColor', [1 0.9 0.3], 'EdgeColor', [1 0.9 0.3]);
        end

        % ---------- target fish
        tpos = [L.tx(kr) L.ty(kr) L.tz(kr)];
        ta = P.tgtW*t;
        set(fish, 'Matrix', makehgtform('translate', tpos) * ...
                            makehgtform('zrotate', ta + pi/2));

        % ---------- trail and tether
        set(hTrail, 'XData', L.x(1:kr), 'YData', L.y(1:kr), 'ZData', L.z(1:kr));
        p0 = M(1:3, 4)' + (M(1:3, 1:3)*[-0.215; 0; 0])';
        [tx_, ty_, tz_] = tetherCurve(p0, P.anchor);
        set(hTether, 'XData', tx_, 'YData', ty_, 'ZData', tz_);
        fi = 4:5:numel(tx_);
        set(hFloats, 'XData', tx_(fi), 'YData', ty_(fi), 'ZData', tz_(fi));

        % ---------- follow camera
        if abs(kr - lastK) > 5
            camYaw = psi;
        else
            camYaw = camYaw + 0.10*wrap(psi - camYaw);
        end
        lastK = kr;
        Rc = [cos(camYaw) -sin(camYaw); sin(camYaw) cos(camYaw)];
        o = camOff(viewIdx, :);
        xy = Rc*[o(1); o(2)];
        cp = [pos(1) + xy(1), pos(2) + xy(2), pos(3) + o(3)];
        cp(3) = max(cp(3), P.seabedZ + 0.4);
        campos(ax3, cp);
        camtarget(ax3, pos);
        if viewIdx == 4
            camup(ax3, [cos(camYaw) sin(camYaw) 0]);
        else
            camup(ax3, [0 0 1]);
        end

        % ---------- map
        set(hMTrail, 'XData', L.x(1:kr), 'YData', L.y(1:kr));
        s = 0.55;
        tri = [1.0 0; -0.7 0.55; -0.7 -0.55]*s;
        Rm = [cos(psi) -sin(psi); sin(psi) cos(psi)];
        tri = (Rm*tri')';
        set(hMVeh, 'XData', pos(1) + tri(:,1), 'YData', pos(2) + tri(:,2));
        set(hMTgt, 'XData', tpos(1), 'YData', tpos(2));
        aa = linspace(psi - P.fovH, psi + P.fovH, 14);
        Rf = min(P.detR, 4);
        set(hMFov, 'XData', [pos(1), pos(1) + Rf*cos(aa)], ...
                   'YData', [pos(2), pos(2) + Rf*sin(aa)]);
        if L.det(kr)
            set(hMFov, 'FaceColor', [0.3 1 0.4]);
            set(hMDet, 'XData', [pos(1) tpos(1)], 'YData', [pos(2) tpos(2)]);
        else
            set(hMFov, 'FaceColor', [1 0.9 0.3]);
            set(hMDet, 'XData', nan, 'YData', nan);
        end

        % ---------- onboard camera + AI overlay
        bgk = 0.45 + 0.55*exp(0.35*pos(3));
        set(hCBg, 'FaceColor', [0.10 0.40 0.58]*bgk);
        ynSea = (P.seabedZ - pos(3))/P.detR/tan(P.fovV);
        ynSea = max(-1, min(1, ynSea));
        set(hCSand, 'YData', [-1 -1 ynSea ynSea], 'FaceColor', [0.72 0.64 0.45]*bgk);

        dxw = tpos(1) - pos(1);  dyw = tpos(2) - pos(2);  dzw = tpos(3) - pos(3);
        xb = cos(psi)*dxw + sin(psi)*dyw;
        yb = -sin(psi)*dxw + cos(psi)*dyw;
        if L.det(kr)
            xn = -yb/xb/tan(P.fovH);
            yn = dzw/xb/tan(P.fovV);
            wn = 0.30/xb/tan(P.fovH);
            hn = 0.14/xb/tan(P.fovV);
            th = linspace(0, 2*pi, 30);
            set(hCFish, 'XData', xn + wn*cos(th), 'YData', yn + hn*sin(th), ...
                        'FaceAlpha', max(0.25, exp(-1.2*L.rg(kr)/P.vis)));
            set(hCBox, 'XData', xn + [-1 1 1 -1 -1]*wn*1.25, ...
                       'YData', yn + [-1 -1 1 1 -1]*hn*1.6);
            conf = max(0.35, min(0.99, 1.05 - L.rg(kr)/(1.4*P.detR)));
            set(hCLbl, 'Position', [xn - wn*1.25, yn + hn*1.6 + 0.02], ...
                       'String', sprintf('fish %.2f  %.1f m', conf, L.rg(kr)));
            set(hCStat, 'String', 'AI: OBJECT DETECTED', 'Color', [0.3 1 0.4]);
        else
            set(hCFish, 'XData', nan, 'YData', nan);
            set(hCBox, 'XData', nan, 'YData', nan);
            set(hCLbl, 'String', '');
            set(hCStat, 'String', 'AI: searching...', 'Color', txtCol);
        end
        if ~isnan(L.snapT) && t >= L.snapT && t < L.snapT + 2.0
            set(hCFlash, 'String', 'SNAPSHOT SAVED (queued for uplink)');
        else
            set(hCFlash, 'String', '');
        end

        % ---------- time cursors
        if ~isempty(hCurA), set(hCurA, 'XData', [t t]); end
        if ~isempty(hDotA), set(hDotA, 'XData', t/60, 'YData', L.soc(kr)); end

        % ---------- HUD
        md = L.mode(kr);
        set(hHud1, 'String', ['MODE ' modeNames{md}], 'Color', modeCols(md, :));
        set(hHud2, 'String', sprintf(['t = %5.1f s    depth = %4.2f m    speed = %4.2f m/s' ...
                  '    heading = %3.0f deg'], t, -pos(3), L.u(kr), mod(rad2deg_(psi), 360)));
        up = pos(3) > -0.45;
        if P.comm == 1
            st = '4G UPLINK: DISABLED  -  data stays on the SD card';
            sc3 = dimCol;
        elseif up
            if isnan(L.snapT)
                st = '4G UPLINK: ON  -  antenna above water, sending telemetry';
            elseif ~isnan(L.surfT) && t >= L.surfT + 2
                st = '4G UPLINK: ON  -  snapshot + detections uploaded';
            else
                st = '4G UPLINK: ON  -  antenna above water, uploading snapshot...';
            end
            sc3 = [0.3 1 0.4];
        elseif P.comm == 3
            st = '4G UPLINK: modem searching for signal under water (wasting 2.5 W)';
            sc3 = [1 0.75 0.3];
        else
            st = '4G UPLINK: OFF (submerged)  -  recording locally';
            sc3 = [1 0.55 0.45];
        end
        set(hHud3, 'String', st, 'Color', sc3);
        soc = L.soc(kr);
        if soc > 40
            bc = [0.3 1 0.4];
        elseif soc > 20
            bc = [1 0.8 0.3];
        else
            bc = [1 0.4 0.35];
        end
        bs = sprintf('BATTERY %3.0f%%    load %5.1f W    used %.2f Wh', soc, L.Pbat(kr), L.E(kr));
        if ~isnan(L.lowT) && t >= L.lowT
            bs = [bs '    LOW BATTERY: returning'];
        end
        set(hHud4, 'String', bs, 'Color', bc);
    end

end

%% ================================================================ local
function h = ui(parent, style, str, pos, opt)
bg = [0.07 0.09 0.12];
fg = [0.92 0.94 0.96];
if nargin < 5, opt = ''; end
switch style
    case 'text'
        h = uicontrol(parent, 'Style', 'text', 'String', str, 'Units', 'normalized', ...
                      'Position', pos, 'BackgroundColor', bg, 'ForegroundColor', fg, ...
                      'HorizontalAlignment', 'left', 'FontSize', 9);
        if strcmp(opt, 'bold')
            set(h, 'FontWeight', 'bold');
        elseif strcmp(opt, 'dim')
            set(h, 'ForegroundColor', [0.65 0.7 0.75], 'FontSize', 8);
        elseif strcmp(opt, 'warn')
            set(h, 'ForegroundColor', [1 0.5 0.4], 'FontSize', 8);
        end
    case 'edit'
        h = uicontrol(parent, 'Style', 'edit', 'String', str, 'Units', 'normalized', ...
                      'Position', pos, 'FontSize', 9);
    case 'popup'
        h = uicontrol(parent, 'Style', 'popupmenu', 'String', str, 'Units', 'normalized', ...
                      'Position', pos, 'FontSize', 9);
    otherwise
        h = uicontrol(parent, 'Style', 'pushbutton', 'String', str, 'Units', 'normalized', ...
                      'Position', pos, 'FontSize', 9);
end
end

function lines = resultLines(P, S, cfg)
lines = {};
lines{end+1} = sprintf('BATTERY  %g mAh x %.1f V = %.1f Wh   (usable %.1f Wh)', ...
                       cfg.cap, cfg.volt, P.EnomWh, P.EcapWh);
lines{end+1} = sprintf('MISSION  %.0f s, %.2f Wh used (%.1f%% of usable)', ...
                       S.T, S.Em, 100*S.Em/P.EcapWh);
lines{end+1} = sprintf('LOAD     average %.1f W    peak %.0f W', S.Pavg, S.Pmax);
lines{end+1} = sprintf('RUNTIME  about %.0f min   (%.0f missions back-to-back)', ...
                       S.runMin, S.nMis);
lines{end+1} = sprintf('SHARE    thrust %.0f%% | Pi+AI %.0f%% | lights %.0f%% | 4G %.0f%% | other %.0f%%', ...
                       100*S.frac(1), 100*(S.frac(2) + S.frac(3)), 100*S.frac(4), ...
                       100*S.frac(5), 100*(S.frac(6) + S.frac(7)));
lines{end+1} = sprintf('DEPTH    overshoot %.2f m    rms error %.2f m', S.ovs, S.rmsz);
if isnan(S.psiRms)
    lines{end+1} = 'HEADING  no tracking phase (target not detected in this run)';
else
    lines{end+1} = sprintf('HEADING  tracking error rms %.1f deg   max %.1f deg', S.psiRms, S.psiMax);
end
lines{end+1} = sprintf('WATER    visibility %.1f m   lights %.0f%%   cruise %.1f m/s', ...
                       P.vis, 100*P.lightLvl, P.cruise);
if ~isnan(S.lowT)
    lines{end+1} = sprintf('EVENT    low battery at %.0f s, vehicle returned early', S.lowT);
end
if ~isnan(S.deadT)
    lines{end+1} = sprintf('EVENT    battery empty at %.0f s, vehicle floated up', S.deadT);
end
end

function a = rad2deg_(r)
a = r*180/pi;
end

function a = wrap(a)
a = mod(a + pi, 2*pi) - pi;
end

function styleAx(ax, axCol, dimCol)
set(ax, 'Color', axCol, 'XColor', dimCol, 'YColor', dimCol, 'GridColor', [0.5 0.55 0.6], ...
        'GridAlpha', 0.25, 'Box', 'on', 'FontSize', 9);
grid(ax, 'on');
end

function drawBands(ax, L, cols, ylo, yhi)
idx = [1, find(diff(L.mode) ~= 0) + 1, numel(L.t) + 1];
for i = 1:numel(idx) - 1
    a = idx(i);
    b = min(idx(i+1), numel(L.t));
    patch(ax, [L.t(a) L.t(b) L.t(b) L.t(a)], [ylo ylo yhi yhi], cols(L.mode(a), :), ...
          'FaceAlpha', 0.20, 'EdgeColor', 'none');
end
end

function cfg = defaultCfg()
cfg.env = 1;               % 1 shallow, 2 medium
cfg.wq = 1;                % 1 clear, 2 moderate, 3 turbid, 4 very turbid
cfg.pm = 2;                % 1 eco, 2 normal, 3 performance
cfg.comm = 2;              % 1 off, 2 surface only, 3 always on
cfg.kz = [25 4 30];        % depth PID: Kp Ki Kd
cfg.kpsi = [3.0 2.2];      % heading PD: Kp Kd
cfg.ku = 12;               % speed gain
cfg.cap = 5000;            % battery capacity (mAh)
cfg.volt = 14.8;           % battery voltage (V), 4S LiPo
cfg.usable = 85;           % usable fraction of capacity (%)
end

function P = params(cfg)
P.dt = 0.05;  P.Tmax = 300;
% vehicle (effective mass includes added mass) -- assumed values
P.mu = 8.0;   P.mw = 12.0;   P.Iz = 0.35;
P.Xu = 6.0;   P.Xuu = 18.0;          % surge drag  (linear, quadratic)
P.Zw = 10.0;  P.Zww = 30.0;          % heave drag
P.Nr = 0.8;   P.Nrr = 1.5;           % yaw drag
P.buoy = 1.0;                        % residual buoyancy (N, slightly positive)
P.d = 0.13;                          % rear thruster lateral offset (m)
P.Tthr = 15;                         % max thrust per thruster (N)
P.tau = 0.15;                        % thruster lag (s)
P.zRate = 0.35;                      % max depth reference rate (m/s)
% controller gains (from the settings panel)
P.kz = cfg.kz;  P.kpsi = cfg.kpsi;  P.ku = cfg.ku;
% environment
envSeabed = [-6 -15];
envDepth  = [-3 -8];
envWp     = [-2 -3; -4 -8];
P.seabedZ = envSeabed(cfg.env);
P.depthOp = envDepth(cfg.env);
P.WP = [4 -3 envWp(cfg.env, 1); 8 -3 envWp(cfg.env, 2); 8 4 envWp(cfg.env, 2)];
P.anchor = [-1.5 0 0];               % base buoy (tether end)
P.tgtC = [10 2];  P.tgtR = 2;  P.tgtW = 0.15;  P.tgtZ = P.depthOp;   % swimming fish
P.home = [0 0];
% water quality -> visibility -> AI detection range
visTab = [8 5 3 1.5];
P.wq = cfg.wq;
P.vis = visTab(cfg.wq);
P.detR = P.vis;
P.fovH = pi/4;  P.fovV = 35*pi/180;  % camera model
P.standoff = 2.5;  P.hold = 12;
% power mode
cruiseTab = [0.4 0.6 0.9];
piTab = [4.5 5.5 7.0];
aiTab = [1.5 3.0 4.0];
lightTab = [0.3 0.6 1.0];
boostTab = [1.0 1.25 1.6 2.0];
P.cruise = cruiseTab(cfg.pm);
P.piW = piTab(cfg.pm);
P.aiW = aiTab(cfg.pm);
P.lightLvl = min(1.0, lightTab(cfg.pm)*boostTab(cfg.wq));
P.lightMax = 14;                     % W at 100% light
P.PthrMax = 70;                      % W per thruster at full thrust
P.escW = 1.5;                        % ESC + sensors (W)
P.eta = 0.90;                        % DC-DC converter efficiency
P.comm = cfg.comm;
% battery
P.EnomWh = cfg.cap/1000*cfg.volt;
P.EcapWh = P.EnomWh*cfg.usable/100;
end

function L = simulate(P)
dt = P.dt;
Nmax = round(P.Tmax/dt) + 1;
names = {'t','x','y','z','psi','u','w','r','TL','TR','TV','mode','det','rg', ...
         'tx','ty','tz','zref','psiref','phi','theta','angL','angR','angV','Pbat'};
for i = 1:numel(names)
    L.(names{i}) = zeros(1, Nmax);
end
L.P = zeros(7, Nmax);

x = 0; y = 0; z = -0.2; psi = 0; u = 0; w = 0; r = 0;
TL = 0; TR = 0; TV = 0;
mode = 1; wpi = 1; intz = 0; trackT = 0; zr = -0.2;
snapT = NaN; surfT = NaN; lowT = NaN; deadT = NaN;
dead = false; Eacc = 0; soc = 1;
phi = 0; theta = 0; aL = 0; aR = 0; aV = 0;
nEnd = Nmax;
kz = P.kz;  kp = P.kpsi;  ku = P.ku;

for k = 1:Nmax
    t = (k - 1)*dt;

    % ---- target (fish swimming on a circle)
    ta = P.tgtW*t;
    tx = P.tgtC(1) + P.tgtR*cos(ta);
    ty = P.tgtC(2) + P.tgtR*sin(ta);
    tz = P.tgtZ + 0.25*sin(0.5*t);

    % ---- "AI camera": is the target inside the field of view and visible?
    dx = tx - x;  dy = ty - y;  dz = tz - z;
    xb = cos(psi)*dx + sin(psi)*dy;
    yb = -sin(psi)*dx + cos(psi)*dy;
    rg = sqrt(dx^2 + dy^2 + dz^2);
    brg = atan2(yb, xb);
    elev = atan2(dz, max(hypot(xb, yb), 1e-6));
    hit = (xb > 0) && (rg < P.detR) && (abs(brg) < P.fovH) && (abs(elev) < P.fovV);

    % ---- mission state machine
    zref = P.depthOp;  psiref = psi;  uref = 0;

    if mode == 1                                   % dive
        psiref = 0;
        if z < P.depthOp + 0.2, mode = 2; end
    end
    if mode == 2                                   % search along waypoints
        wp = P.WP(wpi, :);
        zref = wp(3);
        dxw = wp(1) - x;  dyw = wp(2) - y;
        psiref = atan2(dyw, dxw);
        if abs(wrap(psiref - psi)) < 0.5, uref = P.cruise; end
        if hypot(dxw, dyw) < 0.8
            wpi = wpi + 1;
            if wpi > size(P.WP, 1)
                mode = 4;  wpi = size(P.WP, 1);
            end
        end
        if hit, mode = 3; trackT = 0; end
    elseif mode == 3                               % AI tracking
        psiref = atan2(dy, dx);
        zref = tz;
        uref = max(-0.3, min(P.cruise, 0.5*(hypot(dx, dy) - P.standoff)));
        if abs(hypot(dx, dy) - P.standoff) < 0.8, trackT = trackT + dt; end
        if trackT >= P.hold
            mode = 4;  snapT = t;
        end
    end
    if ~dead && soc < 0.20 && mode < 4             % low battery: go home early
        mode = 4;
        if isnan(lowT), lowT = t; end
    end
    if mode == 4                                   % return to base
        zref = P.depthOp;
        dxh = P.home(1) - x;  dyh = P.home(2) - y;
        psiref = atan2(dyh, dxh);
        if abs(wrap(psiref - psi)) < 0.5, uref = P.cruise; end
        dh = hypot(dxh, dyh);
        if dh < 3, uref = min(uref, 0.25 + 0.1*dh); end
        if dh < 0.6, mode = 5; end
    end
    if mode == 5                                   % ascend
        zref = -0.2;  uref = 0;
        if z > -0.35, mode = 6; surfT = t; end
    end
    if mode == 6                                   % surfaced
        zref = -0.2;  uref = 0;
    end

    % ---- autopilot (depth reference is rate limited)
    zr = zr + max(-P.zRate*dt, min(P.zRate*dt, zref - zr));
    ez = zr - z;
    cVraw = kz(1)*ez + kz(2)*intz - kz(3)*w - P.buoy;
    if abs(cVraw) < P.Tthr
        intz = max(-3, min(3, intz + ez*dt));
    end
    cV = max(-P.Tthr, min(P.Tthr, cVraw));

    Mz = kp(1)*wrap(psiref - psi) - kp(2)*r;
    Mz = max(-2*P.d*P.Tthr, min(2*P.d*P.Tthr, Mz));
    Ff = P.Xu*uref + P.Xuu*abs(uref)*uref + ku*(uref - u);
    cR = max(-P.Tthr, min(P.Tthr, Ff/2 + Mz/(2*P.d)));
    cL = max(-P.Tthr, min(P.Tthr, Ff/2 - Mz/(2*P.d)));
    if dead
        cV = 0;  cR = 0;  cL = 0;
    end

    % ---- electrical power (W) and battery
    if dead
        parts = zeros(7, 1);
    else
        Pthr = P.PthrMax*((abs(TL)/P.Tthr)^1.5 + (abs(TR)/P.Tthr)^1.5 + ...
                          (abs(TV)/P.Tthr)^1.5) + 1.2;
        if z < -0.5, Plt = P.lightLvl*P.lightMax; else, Plt = 0; end
        surfd = z > -0.45;
        if isnan(snapT), Pup = 1.2; else, Pup = 3.5; end
        if P.comm == 1
            Pc = 0;
        elseif surfd
            Pc = Pup;
        elseif P.comm == 3
            Pc = 2.5;
        else
            Pc = 0;
        end
        Pload = Pthr + P.piW + P.aiW + Plt + Pc + P.escW;
        Pb = Pload/P.eta;
        parts = [Pthr; P.piW; P.aiW; Plt; Pc; P.escW; Pb - Pload];
    end
    Pbat = sum(parts);
    Eacc = Eacc + Pbat*dt/3600;
    soc = 1 - Eacc/P.EcapWh;

    % ---- log (state at the start of this step)
    L.t(k) = t;  L.x(k) = x;  L.y(k) = y;  L.z(k) = z;  L.psi(k) = psi;
    L.u(k) = u;  L.w(k) = w;  L.r(k) = r;
    L.TL(k) = TL;  L.TR(k) = TR;  L.TV(k) = TV;
    L.mode(k) = mode;  L.det(k) = hit;  L.rg(k) = rg;
    L.tx(k) = tx;  L.ty(k) = ty;  L.tz(k) = tz;
    L.zref(k) = zr;  L.psiref(k) = psiref;
    L.phi(k) = phi;  L.theta(k) = theta;
    L.angL(k) = aL;  L.angR(k) = aR;  L.angV(k) = aV;
    L.Pbat(k) = Pbat;  L.P(:, k) = parts;

    if ~dead && soc <= 0
        dead = true;  deadT = t;  mode = 7;
    end
    if mode == 6 && t >= surfT + 8
        nEnd = k;
        break;
    end
    if dead && (z > -0.1 || t >= deadT + 60)
        nEnd = k;
        break;
    end

    % ---- thruster lag
    a = dt/P.tau;
    TL = TL + a*(cL - TL);
    TR = TR + a*(cR - TR);
    TV = TV + a*(cV - TV);

    % ---- dynamics
    du = (TL + TR - P.Xu*u - P.Xuu*abs(u)*u)/P.mu;
    dw = (TV + P.buoy - P.Zw*w - P.Zww*abs(w)*w)/P.mw;
    dr = (P.d*(TR - TL) - P.Nr*r - P.Nrr*abs(r)*r)/P.Iz;
    u = u + du*dt;  w = w + dw*dt;  r = r + dr*dt;
    x = x + u*cos(psi)*dt;
    y = y + u*sin(psi)*dt;
    z = z + w*dt;
    psi = psi + r*dt;
    if z > -0.05, z = -0.05; w = min(w, 0); end

    % ---- visual-only attitude and propeller angles
    thT = max(-0.25, min(0.25, -0.8*w));
    phT = max(-0.25, min(0.25, -0.5*r*min(abs(u), 0.8)));
    theta = theta + 0.15*(thT - theta);
    phi = phi + 0.15*(phT - phi);
    aL = aL + 14*TL/P.Tthr*dt;
    aR = aR + 14*TR/P.Tthr*dt;
    aV = aV + 14*TV/P.Tthr*dt;
end

fn = fieldnames(L);
for i = 1:numel(fn)
    v = L.(fn{i});
    L.(fn{i}) = v(:, 1:nEnd);
end
L.E = cumsum(L.Pbat)*dt/3600;                       % Wh used so far
L.soc = max(0, 100*(1 - L.E/P.EcapWh));             % battery %
L.snapT = snapT;  L.surfT = surfT;  L.lowT = lowT;  L.deadT = deadT;
end

function S = summarize(L, P)
dt = P.dt;
S.T = L.t(end);
S.Em = L.E(end);
S.Pavg = S.Em*3600/max(S.T, dt);
S.Pmax = max(L.Pbat);
S.runMin = P.EcapWh/max(S.Pavg, 1e-9)*60;
S.nMis = P.EcapWh/max(S.Em, 1e-9);
sel = (L.mode >= 2) & (L.mode <= 4);
S.ovs = max(0, min(L.zref) - min(L.z));
if any(sel)
    S.rmsz = sqrt(mean((L.zref(sel) - L.z(sel)).^2));
else
    S.rmsz = NaN;
end
idx = find(L.mode == 3);
if numel(idx) > 80
    idx = idx(61:end);
    e = abs(wrap(L.psiref(idx) - L.psi(idx)));
    S.psiRms = sqrt(mean(e.^2))*180/pi;
    S.psiMax = max(e)*180/pi;
else
    S.psiRms = NaN;
    S.psiMax = NaN;
end
parts = sum(L.P, 2)*dt/3600;
S.frac = parts/max(sum(parts), 1e-12);
S.lowT = L.lowT;
S.deadT = L.deadT;
end

function buildScene(ax, P)
[X, Y] = meshgrid(-7:0.5:22, -11:0.5:12);
Z = seabed(X, Y, P);
surf(ax, X, Y, Z, Z, 'FaceColor', 'interp', 'EdgeColor', 'none', ...
     'AmbientStrength', 0.6, 'SpecularStrength', 0.05);
colormap(ax, [linspace(0.50, 0.90, 64)', linspace(0.42, 0.82, 64)', linspace(0.26, 0.58, 64)']);
caxis(ax, [min(Z(:)) max(Z(:))]);

% rocks
[sx, sy, sz] = sphere(10);
rocks = [3 5 0.45; 6 -6 0.55; 12 -3 0.6; 14 6 0.5; -2 -5 0.4; 9 8 0.5; 1 7 0.35];
for i = 1:size(rocks, 1)
    rx = rocks(i, 1);  ry = rocks(i, 2);  rr = rocks(i, 3);
    surf(ax, rr*sx + rx, rr*sy + ry, 0.7*rr*sz + seabed(rx, ry, P) + 0.1, ...
         'FaceColor', [0.38 0.38 0.40], 'EdgeColor', 'none');
end

% suspended particles (more of them in turbid water)
counts = [0 150 450 900];
n = counts(P.wq);
if n > 0
    rs = RandStream('mt19937ar', 'Seed', 7);
    px = -7 + 29*rand(rs, n, 1);
    py = -11 + 23*rand(rs, n, 1);
    pz = P.seabedZ*rand(rs, n, 1);
    plot3(ax, px, py, pz, '.', 'Color', [0.85 0.85 0.72], 'MarkerSize', 4);
end

% water surface
patch(ax, [-7 22 22 -7], [-11 -11 12 12], [0 0 0 0], [0.35 0.75 0.95], ...
      'FaceAlpha', 0.16, 'EdgeColor', 'none');

% base buoy (tether end) at the surface
surf(ax, 0.18*sx + P.anchor(1), 0.18*sy + P.anchor(2), 0.18*sz + 0.02, ...
     'FaceColor', [1 0.4 0.1], 'EdgeColor', 'none');
plot3(ax, P.anchor(1)*[1 1], P.anchor(2)*[1 1], [0.1 0.6], '-', 'Color', [1 0.9 0.2], 'LineWidth', 2);

% waypoints
plot3(ax, P.WP(:,1), P.WP(:,2), P.WP(:,3), 'd', 'MarkerSize', 9, ...
      'MarkerFaceColor', [0.3 1 0.5], 'MarkerEdgeColor', 'w');
end

function Z = seabed(X, Y, P)
Z = P.seabedZ + 0.25*sin(0.6*X).*cos(0.5*Y) + 0.15*sin(1.7*X + 0.9*Y);
end

function h = buildFish(ax)
h = hgtransform('Parent', ax);
[ex, ey, ez] = ellipsoid(0, 0, 0, 0.30, 0.09, 0.13, 16);
surf(ex, ey, ez, 'Parent', h, 'FaceColor', [1.0 0.50 0.10], 'EdgeColor', 'none');
patch('Parent', h, 'XData', [-0.25 -0.48 -0.48], 'YData', [0 0 0], ...
      'ZData', [0 0.14 -0.14], 'FaceColor', [1.0 0.40 0.05], 'EdgeColor', 'none');
patch('Parent', h, 'XData', [0.0 -0.12 -0.18], 'YData', [0 0 0], ...
      'ZData', [0.12 0.24 0.10], 'FaceColor', [1.0 0.40 0.05], 'EdgeColor', 'none');
[sx, sy, sz] = sphere(6);
surf(0.025*sx + 0.20, 0.025*sy + 0.075, 0.025*sz + 0.03, 'Parent', h, ...
     'FaceColor', 'k', 'EdgeColor', 'none');
surf(0.025*sx + 0.20, 0.025*sy - 0.075, 0.025*sz + 0.03, 'Parent', h, ...
     'FaceColor', 'k', 'EdgeColor', 'none');
end

function V = buildVehicle(ax, Rv)
% Body frame: x forward, y left, z up.  Origin = centre of the sealed tube.
V.h = hgtransform('Parent', ax);
h = V.h;
frameCol = [0.90 0.90 0.92];      % white PETG
foamCol  = [1.00 0.65 0.15];
darkCol  = [0.18 0.20 0.24];

% ---- sealed tube (clear acrylic) + dome + rear cap
tubeR = 0.075;  tubeL = 0.40;
[Yc, Zc, Xc] = cylinder(tubeR, 32);
Xc = (Xc - 0.5)*tubeL;
surf(Xc, Yc, Zc, 'Parent', h, 'FaceColor', [0.75 0.90 0.98], 'FaceAlpha', 0.25, ...
     'EdgeColor', 'none');
[thd, phd] = meshgrid(linspace(0, pi/2, 12), linspace(0, 2*pi, 32));
surf(tubeL/2 + tubeR*cos(thd), tubeR*sin(thd).*cos(phd), tubeR*sin(thd).*sin(phd), ...
     'Parent', h, 'FaceColor', [0.70 0.90 1.00], 'FaceAlpha', 0.35, 'EdgeColor', 'none');
diskx(h, -tubeL/2, tubeR, frameCol);

% ---- electronics inside the tube (Pi, ESC board, LiPo) and camera
boxp(h, [0.10 0 -0.005], [0.085 0.056 0.020], [0.10 0.60 0.30]);
boxp(h, [0.00 0 -0.010], [0.070 0.045 0.015], [0.10 0.70 0.70]);
boxp(h, [-0.11 0 -0.010], [0.070 0.035 0.030], [0.20 0.25 0.65]);
boxp(h, [0.185 0 0], [0.020 0.035 0.035], [0.05 0.05 0.06]);
[sx, sy, sz] = sphere(8);
surf(0.016*sx + 0.200, 0.016*sy, 0.016*sz, 'Parent', h, 'FaceColor', [0.1 0.2 0.5], ...
     'EdgeColor', 'none');

% ---- printed frame: two rings + 4 rails + rear arms
for xr = [-0.15 0.15]
    boxp(h, [xr 0 0.10],  [0.015 0.30 0.015], frameCol);
    boxp(h, [xr 0 -0.10], [0.015 0.30 0.015], frameCol);
    boxp(h, [xr 0.14 0],  [0.015 0.015 0.20], frameCol);
    boxp(h, [xr -0.14 0], [0.015 0.015 0.20], frameCol);
end
for ys = [-0.14 0.14]
    for zs = [-0.10 0.10]
        boxp(h, [0 ys zs], [0.30 0.015 0.015], frameCol);
    end
    boxp(h, [-0.225 ys 0], [0.15 0.015 0.015], frameCol);
end

% ---- buoyancy foam (top), ballast (bottom), cable gland (rear)
boxp(h, [0.10 0 0.125],  [0.12 0.14 0.04], foamCol);
boxp(h, [-0.10 0 0.125], [0.12 0.14 0.04], foamCol);
boxp(h, [-0.03 0 -0.125], [0.12 0.05 0.025], darkCol);
boxp(h, [-0.215 0 0], [0.03 0.025 0.025], [0.9 0.7 0.1]);

% ---- LED lights on the front ring
for ys = [-0.12 0.12]
    surf(0.018*sx + 0.16, 0.018*sy + ys, 0.018*sz + 0.0, 'Parent', h, ...
         'FaceColor', [1 1 0.65], 'EdgeColor', 'none');
end

% ---- thrusters: duct + spinning propeller (local x = thrust axis)
base(1) = hgtransform('Parent', h, 'Matrix', makehgtform('translate', [-0.27 0.14 0]));
base(2) = hgtransform('Parent', h, 'Matrix', makehgtform('translate', [-0.27 -0.14 0]));
base(3) = hgtransform('Parent', h, 'Matrix', ...
          makehgtform('translate', [0 0 0.115]) * makehgtform('yrotate', -pi/2));
for i = 1:3
    [Yd, Zd, Xd] = cylinder(0.034, 20);
    Xd = (Xd - 0.5)*0.08;
    surf(Xd, Yd, Zd, 'Parent', base(i), 'FaceColor', [0.22 0.27 0.33], 'EdgeColor', 'none');
    V.spin(i) = hgtransform('Parent', base(i));
    patch('Parent', V.spin(i), 'XData', [0 0 0 0], 'YData', [-0.030 0.030 0.030 -0.030], ...
          'ZData', [-0.007 -0.007 0.007 0.007], 'FaceColor', [0.95 0.35 0.15], 'EdgeColor', 'none');
    patch('Parent', V.spin(i), 'XData', [0 0 0 0], 'YData', [-0.007 -0.007 0.007 0.007], ...
          'ZData', [-0.030 0.030 0.030 -0.030], 'FaceColor', [0.95 0.35 0.15], 'EdgeColor', 'none');
end

% ---- water jets (updated every frame)
jc = [0.65 0.95 1.00];
V.jetL = line('Parent', h, 'XData', [-0.31 -0.31], 'YData', [0.14 0.14], 'ZData', [0 0], ...
              'Color', jc, 'LineWidth', 2, 'Visible', 'off');
V.jetR = line('Parent', h, 'XData', [-0.31 -0.31], 'YData', [-0.14 -0.14], 'ZData', [0 0], ...
              'Color', jc, 'LineWidth', 2, 'Visible', 'off');
V.jetV = line('Parent', h, 'XData', [0 0], 'YData', [0 0], 'ZData', [0.15 0.15], ...
              'Color', jc, 'LineWidth', 2, 'Visible', 'off');

% ---- camera field-of-view pyramid (drawn range Rv metres)
A = [0.205 0 0];
fovH = pi/4;  fovV = 35*pi/180;
C = [A(1) + Rv,  Rv*tan(fovH),  Rv*tan(fovV); ...
     A(1) + Rv, -Rv*tan(fovH),  Rv*tan(fovV); ...
     A(1) + Rv, -Rv*tan(fovH), -Rv*tan(fovV); ...
     A(1) + Rv,  Rv*tan(fovH), -Rv*tan(fovV)];
Vv = [A; C];
V.fov = patch('Parent', h, 'Vertices', Vv, ...
              'Faces', [1 2 3 NaN; 1 3 4 NaN; 1 4 5 NaN; 1 5 2 NaN; 2 3 4 5], ...
              'FaceColor', [1 0.9 0.3], 'FaceAlpha', 0.05, 'EdgeColor', [1 0.9 0.3], ...
              'EdgeAlpha', 0.15);
end

function p = boxp(h, c, s, col)
hx = s(1)/2;  hy = s(2)/2;  hz = s(3)/2;
Vb = [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1];
Vb = Vb.*repmat([hx hy hz], 8, 1) + repmat(c, 8, 1);
Fb = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
p = patch('Parent', h, 'Vertices', Vb, 'Faces', Fb, 'FaceColor', col, 'EdgeColor', 'none');
end

function diskx(h, cx, r, col)
th = linspace(0, 2*pi, 32);
patch('Parent', h, 'XData', cx*ones(size(th)), 'YData', r*cos(th), 'ZData', r*sin(th), ...
      'FaceColor', col, 'EdgeColor', 'none');
end

function [X, Y, Z] = tetherCurve(p0, p1)
n = 40;
s = linspace(0, 1, n);
D = norm(p1 - p0);
sag = min(2.5, 0.35 + 0.05*D + 0.08*abs(p0(3)));
X = p0(1) + (p1(1) - p0(1))*s;
Y = p0(2) + (p1(2) - p0(2))*s;
Z = p0(3) + (p1(3) - p0(3))*s - sag*4*s.*(1 - s);
Z = min(Z, -0.02);
end

function folder = makeReport(P, L, S, cfg, fig)
% Writes a folder with report.html, screenshot.png, 4 plots and log.csv
stamp = datestr(now, 'yyyymmdd_HHMMSS');
folder = fullfile(pwd, ['rov_report_' stamp]);
mkdir(folder);
dt = P.dt;
modeNames = {'Dive', 'Search (waypoints)', 'AI tracking', 'Return to base', 'Ascend', ...
             'Surfaced / 4G uplink', 'Battery empty'};
pnames = {'Thrusters', 'Raspberry Pi', 'AI', 'Lights', '4G modem', 'ESC + sensors', ...
          'Converter loss'};
envN = {'Shallow (seabed 6 m, operating depth 3 m)', 'Medium (seabed 15 m, operating depth 8 m)'};
wqN  = {'Clear (8 m)', 'Moderate (5 m)', 'Turbid (3 m)', 'Very turbid (1.5 m)'};
pmN  = {'Eco (0.4 m/s)', 'Normal (0.6 m/s)', 'Performance (0.9 m/s)'};
cmN  = {'4G off', '4G on at surface only', '4G always on'};

% ---- screenshot of the application window
drawnow;
fr = getframe(fig);
imwrite(fr.cdata, fullfile(folder, 'screenshot.png'));

% ---- plots (white background for printing)
newf = @() figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 900 360]);
f = newf();  ax = axes(f);
plot(ax, L.t, L.zref, '--k', L.t, L.z, 'b', 'LineWidth', 1.4);
grid(ax, 'on');  xlabel(ax, 'Time (s)');  ylabel(ax, 'Depth z (m)');
title(ax, 'Depth: reference and actual');  legend(ax, 'reference', 'actual', 'Location', 'best');
saveas(f, fullfile(folder, 'depth.png'));  close(f);

f = newf();  ax = axes(f);
plot(ax, L.t, 100*L.TL/P.Tthr, L.t, 100*L.TR/P.Tthr, L.t, 100*L.TV/P.Tthr, 'LineWidth', 1.2);
grid(ax, 'on');  xlabel(ax, 'Time (s)');  ylabel(ax, 'Thrust (% of max)');
title(ax, 'Thruster commands');  legend(ax, 'left', 'right', 'vertical', 'Location', 'best');
saveas(f, fullfile(folder, 'thrusters.png'));  close(f);

f = newf();  ax = axes(f);
area(ax, L.t, L.P');
grid(ax, 'on');  xlabel(ax, 'Time (s)');  ylabel(ax, 'Battery power (W)');
title(ax, 'Power draw by component');  legend(ax, pnames, 'Location', 'northeast');
saveas(f, fullfile(folder, 'power.png'));  close(f);

Tm = L.t(end);
tr = linspace(0, max(S.runMin*60, Tm), 600);
Ec = floor(tr/Tm)*L.E(end) + interp1(L.t, L.E, mod(tr, Tm), 'linear', 'extrap');
f = newf();  ax = axes(f);
plot(ax, tr/60, max(0, 100*(1 - Ec/P.EcapWh)), 'g', 'LineWidth', 1.6);  hold(ax, 'on');
plot(ax, [0 max(tr)/60], [20 20], '--r');
grid(ax, 'on');  xlabel(ax, 'Time (min), mission repeated back-to-back');
ylabel(ax, 'Battery (%)');  title(ax, 'Battery endurance estimate');
legend(ax, 'battery', '20% reserve', 'Location', 'northeast');
saveas(f, fullfile(folder, 'battery.png'));  close(f);

% ---- raw log as CSV
fid = fopen(fullfile(folder, 'log.csv'), 'w');
fprintf(fid, 't_s,x_m,y_m,depth_m,heading_deg,speed_mps,mode,TL_N,TR_N,TV_N,power_W,energy_Wh,battery_pct\n');
for k = 1:numel(L.t)
    fprintf(fid, '%.2f,%.3f,%.3f,%.3f,%.1f,%.3f,%d,%.2f,%.2f,%.2f,%.2f,%.4f,%.2f\n', ...
            L.t(k), L.x(k), L.y(k), -L.z(k), mod(L.psi(k)*180/pi, 360), L.u(k), L.mode(k), ...
            L.TL(k), L.TR(k), L.TV(k), L.Pbat(k), L.E(k), L.soc(k));
end
fclose(fid);

% ---- HTML report
row = @(a, b) sprintf('<tr><td>%s</td><td>%s</td></tr>\n', a, b);
h = {};
h{end+1} = '<!doctype html><html><head><meta charset="utf-8"><title>Mini-ROV simulation report</title>';
h{end+1} = ['<style>body{font-family:Segoe UI,Arial,sans-serif;max-width:900px;margin:30px auto;color:#1a1f2b;padding:0 16px}' ...
            'h1{font-size:26px;margin-bottom:2px}h2{font-size:18px;border-bottom:2px solid #2b6cb0;padding-bottom:4px;margin-top:30px}' ...
            'table{border-collapse:collapse;width:100%}td,th{border:1px solid #ccd3dd;padding:6px 10px;text-align:left;font-size:14px}' ...
            'th{background:#eef2f7}img{width:100%;margin:8px 0;border:1px solid #ccd3dd}.sub{color:#5a6475}.note{background:#fff7e0;border:1px solid #f0d98a;padding:10px;font-size:13px}' ...
            '@media print{h2{page-break-after:avoid}img{page-break-inside:avoid}}</style></head><body>'];
h{end+1} = '<h1>Mini-ROV simulation report</h1>';
h{end+1} = ['<div class="sub">Generated ' datestr(now, 'yyyy-mm-dd HH:MM') '</div>'];

h{end+1} = '<h2>1. Settings used</h2><table><tr><th>Parameter</th><th>Value</th></tr>';
h{end+1} = row('Environment', envN{cfg.env});
h{end+1} = row('Water quality', wqN{cfg.wq});
h{end+1} = row('Power mode', pmN{cfg.pm});
h{end+1} = row('4G uplink', cmN{cfg.comm});
h{end+1} = row('Depth controller (Kp / Ki / Kd)', sprintf('%g / %g / %g', cfg.kz));
h{end+1} = row('Heading controller (Kp / Kd)', sprintf('%g / %g', cfg.kpsi));
h{end+1} = row('Speed gain', sprintf('%g', cfg.ku));
h{end+1} = row('Battery', sprintf('%g mAh at %.1f V = %.1f Wh nominal, %.0f%% usable = %.1f Wh', ...
                                  cfg.cap, cfg.volt, P.EnomWh, cfg.usable, P.EcapWh));
h{end+1} = '</table>';

h{end+1} = '<h2>2. Results</h2><table><tr><th>Quantity</th><th>Value</th></tr>';
h{end+1} = row('Mission duration', sprintf('%.0f s', S.T));
h{end+1} = row('Energy used in one mission', sprintf('%.2f Wh (%.1f%% of usable battery)', S.Em, 100*S.Em/P.EcapWh));
h{end+1} = row('Average / peak electrical load', sprintf('%.1f W / %.0f W', S.Pavg, S.Pmax));
h{end+1} = row('Estimated runtime (mission repeated)', sprintf('about %.0f min, %.0f missions back-to-back', S.runMin, S.nMis));
h{end+1} = row('Runtime until 20% reserve', sprintf('about %.0f min', 0.8*S.runMin));
h{end+1} = row('Energy share', sprintf('thrusters %.0f%%, Pi %.0f%%, AI %.0f%%, lights %.0f%%, 4G %.0f%%, ESC %.0f%%, converter loss %.0f%%', ...
                                       100*S.frac(1), 100*S.frac(2), 100*S.frac(3), 100*S.frac(4), ...
                                       100*S.frac(5), 100*S.frac(6), 100*S.frac(7)));
h{end+1} = row('Depth overshoot / RMS error', sprintf('%.2f m / %.2f m', S.ovs, S.rmsz));
if isnan(S.psiRms)
    h{end+1} = row('Heading error while tracking', 'no tracking phase (target not detected)');
else
    h{end+1} = row('Heading error while tracking', sprintf('RMS %.1f deg, max %.1f deg', S.psiRms, S.psiMax));
end
h{end+1} = row('Camera visibility / AI range', sprintf('%.1f m', P.vis));
h{end+1} = row('Lights level', sprintf('%.0f%%', 100*P.lightLvl));
if ~isnan(S.lowT), h{end+1} = row('Event', sprintf('low battery at %.0f s, vehicle returned early', S.lowT)); end
if ~isnan(S.deadT), h{end+1} = row('Event', sprintf('battery empty at %.0f s, vehicle floated up', S.deadT)); end
h{end+1} = '</table>';

h{end+1} = '<h2>3. Time in each mission phase</h2><table><tr><th>Phase</th><th>Time (s)</th></tr>';
for m = 1:7
    tm = sum(L.mode == m)*dt;
    if tm > 0, h{end+1} = row(modeNames{m}, sprintf('%.1f', tm)); end
end
h{end+1} = '</table>';

h{end+1} = '<h2>4. Plots</h2>';
h{end+1} = '<img src="depth.png"><img src="thrusters.png"><img src="power.png"><img src="battery.png">';
h{end+1} = '<h2>5. Application screenshot</h2><img src="screenshot.png">';
h{end+1} = ['<h2>6. Assumptions</h2><div class="note">Vehicle mass, drag, thrust and all power figures are assumed values, ' ...
            'not measurements. The AI detection is simulated from camera field of view and water visibility. ' ...
            'Replace the assumptions with values measured on the real vehicle before drawing conclusions. ' ...
            'The raw simulation log is in log.csv.</div></body></html>'];
fid = fopen(fullfile(folder, 'report.html'), 'w');
for i = 1:numel(h)
    fprintf(fid, '%s\n', h{i});
end
fclose(fid);
try
    web(fullfile(folder, 'report.html'), '-browser');
catch
end
end