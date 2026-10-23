function balance_lab(cmd, outDir)
%BALANCE_LAB  Live simulation dashboard for a two-wheel self-balancing robot.
%
%   Simulates the ESP32 + MPU + L298N + BO-motor robot from
%   github.com/Alpha107/Self-Balancing-Robot-ESP32-Firmware
%
%   MODEL
%     - Wheeled inverted pendulum (Lagrangian), RK4 at ~1 ms
%     - DC motors with back-EMF, L298N voltage drop, PWM deadband
%     - Rolling resistance per floor type, pushes, ground roughness
%     - IMU: gyro noise + bias, accelerometer incl. motion error, mount offset
%     - Complementary filter + discrete PID exactly like a firmware loop
%     - Optional speed/position outer loop (what wheel encoders would add)
%
%   DASHBOARD
%     Live 3-D view + live plots, live tuning, push buttons, run history
%     with overlay compare, 7 tests (stability maps, push sweep, loop rate,
%     filter vs noise, Monte-Carlo, CEM auto-tuner = RL), CSV/PNG export,
%     serial link to the real robot for sim-vs-real comparison.
%
%   USAGE   >> balance_lab                       % open the dashboard
%           >> balance_lab('showcase')           % regenerate results/ (CSV + PNG)
%           >> balance_lab('showcase', folder)   % ... into another folder
%   KEYS    Esc = stop run,  Left/Right arrow = push the robot (live run)
%   NEEDS   MATLAB R2021a or newer. No toolboxes.

    if nargin >= 1 && strcmpi(cmd, 'showcase')
        if nargin < 2, outDir = fullfile(pwd, 'results'); end
        makeShowcase(outDir);
        return
    end

    % ================= shared state (used by nested functions) ==========
    P0        = defaultParams();
    fig       = [];
    C         = struct();          % input widgets
    H         = struct();          % output handles
    L         = [];                % left control grid
    row       = 0;                 % row counter for left grid
    rowH      = 22*ones(1,120);    % row heights for left grid
    runs      = struct('id',{},'P',{},'log',{},'met',{},'show',{},'status',{});
    runCount  = 0;
    isBusy    = false;
    stopReq   = false;
    pushReq   = 0;
    bestGains = [];
    realLog   = [];
    scene     = struct('parts',[],'wheels',[],'spokes',[],'com',[],'arrow',[], ...
                       'floor',[],'trail',[],'G',[],'q',[0;0;0;0],'surface',0);
    hPrev     = P0.h;              % last body height (for CoM linking)
    TH = struct('bg',[0.09 0.10 0.12], 'panel',[0.13 0.14 0.17], 'field',[0.20 0.21 0.25], ...
                'text',[0.91 0.92 0.95], 'muted',[0.62 0.64 0.70], 'accent',[0.42 0.72 1.00], ...
                'mono',[0.55 0.95 0.65], 'monoBg',[0.07 0.08 0.10]);
    PHYS = {'Mb','pay','l','h','r','Vbat','db','Ts','w0'};
    CL = struct('th',[0.30 0.75 1.00], 'est',[1.00 0.62 0.20], 'ref',[0.75 0.75 0.78], ...
                'u',[0.80 0.45 1.00], 'x',[0.40 0.90 0.50], 'v',[1.00 0.85 0.30], ...
                'P',[0.30 0.75 1.00], 'I',[1.00 0.62 0.20], 'D',[0.40 0.90 0.50], ...
                'acc',[0.95 0.40 0.55], 'real',[1.00 0.45 0.35], 'sim',[0.30 0.75 1.00], ...
                'pal',[0.30 0.75 1.00; 1.00 0.62 0.20; 0.40 0.90 0.50; 0.95 0.40 0.60; ...
                       0.75 0.55 1.00; 1.00 0.90 0.35; 0.50 0.95 0.95; 0.90 0.90 0.90]);

    buildUI();
    resetScene(P0);
    updateScene([0; 0; deg2rad(P0.th0); 0], 0, [], P0);
    setBusyUI(false, '');
    updateApply();

    % =====================================================================
    %                              UI LAYOUT
    % =====================================================================
    function buildUI()
        fig = uifigure('Name','BALANCE-LAB  |  two-wheel self-balancing robot simulator', ...
            'Position',[30 40 1580 900], 'Color',TH.bg, ...
            'WindowKeyPressFcn',@onKey, 'CloseRequestFcn',@onClose);
        try, theme(fig, 'dark'); catch, end          % R2025a+ dark theme
        root = uigridlayout(fig,[1 3]);
        root.ColumnWidth = {360,'1x',620};
        root.Padding = [6 6 6 6]; root.ColumnSpacing = 6;
        root.BackgroundColor = TH.bg;

        % ---------------- LEFT: controls ----------------
        L = uigridlayout(root,[120 3]);
        L.Layout.Column = 1;
        L.ColumnWidth = {118,'1x',64};
        L.RowSpacing = 3; L.ColumnSpacing = 4; L.Padding = [10 8 10 8];
        L.Scrollable = 'on'; L.BackgroundColor = TH.panel;

        r = nextRow(30);
        lb = uilabel(L,'Text','BALANCE-LAB','FontSize',20,'FontWeight','bold', ...
            'FontColor',TH.accent); put(lb,r,[1 3]);
        r = nextRow(18);
        lb = uilabel(L,'Text','ESP32 balancing robot  ·  physics + PID + RL', ...
            'FontColor',TH.muted); put(lb,r,[1 3]);

        hdr('1  ROBOT  (measure yours)');
        addNum('Mb','Body mass (kg)',[0.15 2],P0.Mb,'%.2f','Chassis + electronics + battery, without wheels');
        addNum('pay','Payload on top (kg)',[0 1],P0.pay,'%.2f','Extra mass on the top plate (raises the CoM)');
        addNum('l','CoM height (m)',[0.02 0.25],P0.l,'%.3f','Axle to centre of mass of the body');
        addNum('h','Body height (m)',[0.08 0.35],P0.h,'%.3f','Axle to top plate');
        addNum('r','Wheel radius (m)',[0.02 0.06],P0.r,'%.4f','BO wheel 65 mm diameter = 0.0325');
        addNum('Vbat','Battery (V)',[5 12.6],P0.Vbat,'%.1f','2 x 18650 = 7.4 V nominal, 8.4 V full');
        addNum('db','PWM deadband',[0 120],P0.db,'%.0f','PWM value below which the motors do not turn');
        addNum('Ts','Stall torque (N·m)',[0.02 0.4],P0.Ts,'%.3f','Per motor at 6 V (BO 1:48 is about 0.08)');
        addNum('w0','No-load speed (rad/s)',[8 60],P0.w0,'%.1f','Per motor at 6 V (200 rpm = 21 rad/s)');
        addDrop('surface','Floor',{'Tile (smooth)','Wood','Carpet'},1:3,P0.surface);
        C.surface.ValueChangedFcn = @(~,~) afterChange('surface');
        r = nextRow(22);
        C.linkCoM = uicheckbox(L,'Text','CoM moves with body height','Value',true, ...
            'FontColor',TH.text,'Tooltip','Changing body height scales the CoM height by the same ratio');
        put(C.linkCoM,r,[1 3]);

        hdr('2  CONTROLLER');
        addDrop('preset','Preset',{'Custom','Tuned: angle + speed loop', ...
            'Angle PID only (tuned gains)','Firmware Config.h defaults', ...
            'Too soft (Kp 40)','Twitchy (Kd 5, 50 Hz)','No integral, no filter'},1:7,1);
        C.preset.ValueChangedFcn = @(~,~) onPreset();
        addDrop('mode','Mode',{'Angle PID only (firmware)','Angle PID + speed/position loop'},1:2,P0.mode);
        addNum('Kp','Kp  (PWM/°)',[0 400],P0.Kp,'%.1f','Proportional gain on tilt');
        addNum('Ki','Ki  (PWM/°·s)',[0 200],P0.Ki,'%.2f','Integral gain on tilt (firmware default 140)');
        addNum('Kd','Kd  (PWM·s/°)',[0 8],P0.Kd,'%.2f','Derivative gain on tilt rate');
        addNum('Kv','Kv speed (°/(m/s))',[0 40],P0.Kv,'%.1f','Outer loop: lean back when moving (needs wheel encoders)');
        addNum('Kx','Kx position (°/m)',[0 20],P0.Kx,'%.1f','Outer loop: return to start position (needs wheel encoders)');
        addNum('f','Loop rate (Hz)',[20 1000],P0.f,'%.0f','Control loop frequency on the ESP32');
        addNum('alpha','Filter alpha',[0 0.999],P0.alpha,'%.3f','Complementary filter: 1 = gyro only, 0 = accelerometer only');
        addNum('trim','Setpoint trim (°)',[-5 5],P0.trim,'%.2f','Balance setpoint offset');
        addDrop('dsrc','D-term from',{'Gyro rate','Error difference'},1:2,P0.dsrc);
        r = nextRow(22);
        C.comp = uicheckbox(L,'Text','Deadband compensation (add minimum PWM)','Value',P0.comp, ...
            'FontColor',TH.text);
        put(C.comp,r,[1 3]);

        hdr('3  SCENARIO');
        addNum('th0','Start tilt (°)',[-30 30],P0.th0,'%.1f');
        addNum('Fpush','Push force (N)',[-8 8],P0.Fpush,'%.2f','Horizontal push at the CoM, + = forward, 0 = none');
        addNum('tPush','Push time (s)',[0 60],P0.tPush,'%.2f');
        addNum('pushDur','Push duration (s)',[0.02 0.5],P0.pushDur,'%.2f');
        addNum('gyroNoise','Gyro noise (°/s)',[0 5],P0.gyroNoise,'%.2f','Standard deviation per sample');
        addNum('accNoise','Accel noise (°)',[0 10],P0.accNoise,'%.2f','Standard deviation per sample');
        addNum('gyroBias','Gyro bias (°/s)',[-3 3],P0.gyroBias,'%.2f');
        addNum('mountOff','IMU mount offset (°)',[-5 5],P0.mountOff,'%.2f','IMU not perfectly level on the chassis');
        addNum('rough','Ground roughness (N)',[0 2],P0.rough,'%.2f','Random horizontal force (bumpy floor)');
        addNum('seed','Random seed',[0 9999],P0.seed,'%.0f','Same seed = same noise = repeatable run');

        hdr('4  LIVE RUN');
        addDrop('dur','Duration',{'5 s','10 s','15 s','30 s','1 min','2 min','5 min','10 min', ...
            'Until stopped (Esc)'},[5 10 15 30 60 120 300 600 Inf],P0.T);
        addDrop('speed','Playback',{'0.25x','0.5x','1x (real time)','2x','4x','Max'}, ...
            [0.25 0.5 1 2 4 Inf],1);
        addDrop('cam','Camera',{'3-D follow','Side view','Front view','Top view'},1:4,1);
        C.cam.ValueChangedFcn = @(~,~) applyCamera();
        r = nextRow(22);
        C.liveTune = uicheckbox(L,'Text','Live edits (any change applies during the run)', ...
            'Value',true,'FontColor',TH.text);
        put(C.liveTune,r,[1 3]);
        g = btnRow(2, 34);
        H.btnRun  = mkBtn(g,1,'RUN LIVE',[0.30 0.69 0.31],@(~,~) onRunLive());
        H.btnStop = mkBtn(g,2,'STOP (Esc)',[0.90 0.45 0.42],@(~,~) onStop());
        g = btnRow(3, 30);
        H.btnFast  = mkBtn(g,1,'FAST RUN',[0.15 0.16 0.20],@(~,~) onFastRun());
        H.btnPushB = mkBtn(g,2,'◀ PUSH',[0.55 0.35 0.15],@(~,~) setPush(-1));
        H.btnPushF = mkBtn(g,3,'PUSH ▶',[0.55 0.35 0.15],@(~,~) setPush(1));

        hdr('5  TESTS  (graphs + data)');
        addDrop('test','Test',{'Stability map: Kp × Kd','Stability map: Kv × Kx (speed loop)', ...
            'Push recovery sweep (both modes)','Loop rate sweep','Filter alpha vs sensor noise', ...
            'Monte-Carlo robustness (40 robots)','CEM auto-tune (RL)'},1:7,1);
        g = btnRow(2, 32);
        H.btnTest  = mkBtn(g,1,'RUN TEST',[0.25 0.47 0.85],@(~,~) onRunTest());
        H.btnApply = mkBtn(g,2,'APPLY BEST GAINS',[0.80 0.62 0.10],@(~,~) onApplyBest());

        hdr('6  OTHER');
        g = btnRow(1, 30);
        H.btnExport = mkBtn(g,1,'EXPORT DATA + GRAPHS',[0.15 0.16 0.20],@(~,~) onExport());
        g = btnRow(3, 30);
        H.btnReset = mkBtn(g,1,'DEFAULTS',[0.15 0.16 0.20],@(~,~) onResetDefaults());
        H.btnClear = mkBtn(g,2,'CLEAR RUNS',[0.15 0.16 0.20],@(~,~) onClearRuns());
        H.btnHelp  = mkBtn(g,3,'HELP',[0.15 0.16 0.20],@(~,~) onHelp());
        L.RowHeight = num2cell(rowH(1:row));

        % ---------------- CENTRE: 3-D view + summary + history ----------------
        Cg = uigridlayout(root,[3 1]);
        Cg.Layout.Column = 2;
        Cg.RowHeight = {'1x',26,240}; Cg.Padding = [0 0 0 0]; Cg.RowSpacing = 4;
        Cg.BackgroundColor = TH.bg;
        H.ax3 = uiaxes(Cg); H.ax3.Layout.Row = 1;
        setup3D(H.ax3);
        H.status = uilabel(Cg,'Text',' READY  |  Esc = stop  |  Left/Right arrow = push the robot during a live run', ...
            'FontName','Courier New','FontWeight','bold','FontSize',12, ...
            'BackgroundColor',[0.12 0.13 0.16],'FontColor',[0.55 0.95 0.65]);
        H.status.Layout.Row = 2;
        B = uigridlayout(Cg,[1 2]); B.Layout.Row = 3;
        B.ColumnWidth = {'1x','1x'}; B.Padding = [0 0 0 0]; B.ColumnSpacing = 4;
        B.BackgroundColor = TH.bg;
        H.summary = uitextarea(B,'Editable','off','FontName','Courier New','FontSize',11, ...
            'BackgroundColor',TH.monoBg,'FontColor',TH.mono, ...
            'Value',{'Press RUN LIVE to simulate with the settings on the left.', ...
                     'Every run is saved in the history table on the right.', ...
                     'Tick "Show" to overlay runs in the Compare tab;', ...
                     'click a row to reload that run''s graphs.'});
        H.table = uitable(B,'ColumnName',{'Show','#','Mode','Time','Kp','Ki','Kd','Push','Max°','Drift','Result'}, ...
            'ColumnEditable',[true false false false false false false false false false false], ...
            'ColumnWidth',{40,28,46,46,40,36,38,38,40,44,'auto'}, 'RowName',{}, ...
            'FontSize',11, 'Data',cell(0,11), ...
            'BackgroundColor',[0.14 0.15 0.18; 0.17 0.18 0.22],'ForegroundColor',TH.text, ...
            'CellEditCallback',@onTableEdit, 'CellSelectionCallback',@onTableSelect);

        % ---------------- RIGHT: tabs with graphs ----------------
        H.TG = uitabgroup(root); H.TG.Layout.Column = 3;
        H.tabLive = uitab(H.TG,'Title','Live');
        H.tabRes  = uitab(H.TG,'Title','Results');
        H.tabCmp  = uitab(H.TG,'Title','Compare runs');
        H.tabTest = uitab(H.TG,'Title','Tests');
        H.tabHw   = uitab(H.TG,'Title','Real robot');
        dark = [0.16 0.17 0.20];

        g = darkGrid(H.tabLive,[3 1],dark);
        H.axL = gobjects(1,3);
        for i = 1:3, H.axL(i) = uiaxes(g); end
        styleAx(H.axL(1),'Pitch (°)','','deg');
        styleAx(H.axL(2),'Motor command (PWM)','','PWM');
        styleAx(H.axL(3),'Position (cm) and speed (cm/s)','time (s)','');
        H.lnRef = line(H.axL(1),nan,nan,'Color',CL.ref,'LineStyle','--');
        H.lnEst = line(H.axL(1),nan,nan,'Color',CL.est,'LineWidth',1);
        H.lnTh  = line(H.axL(1),nan,nan,'Color',CL.th,'LineWidth',1.5);
        legendDark(H.axL(1),[H.lnTh H.lnEst H.lnRef],{'true','estimate','reference'});
        H.lnU   = line(H.axL(2),nan,nan,'Color',CL.u,'LineWidth',1.2);
        H.lnX   = line(H.axL(3),nan,nan,'Color',CL.x,'LineWidth',1.5);
        H.lnV   = line(H.axL(3),nan,nan,'Color',CL.v,'LineWidth',1);
        legendDark(H.axL(3),[H.lnX H.lnV],{'position cm','speed cm/s'});
        for i = 1:3, H.axL(i).XLim = [0 8]; end
        H.axL(2).YLim = [-270 270];

        g = darkGrid(H.tabRes,[3 2],dark);
        H.axR = gobjects(1,6);
        ttls = {'Pitch: true vs estimate (°)','PID terms (PWM)','Position (cm) & speed (cm/s)', ...
                'Sensor fusion (°)','Phase portrait','Motor voltage (V) & wheel torque (N·cm)'};
        for i = 1:6
            H.axR(i) = uiaxes(g);
            styleAx(H.axR(i),ttls{i},'time (s)','');
        end
        H.axR(5).XLabel.String = 'pitch (°)'; H.axR(5).YLabel.String = 'pitch rate (°/s)';

        g = darkGrid(H.tabCmp,[3 1],dark);
        H.axC = gobjects(1,3);
        ct = {'Pitch (°) - ticked runs','Position (cm)','Motor command (PWM)'};
        for i = 1:3
            H.axC(i) = uiaxes(g);
            styleAx(H.axC(i),ct{i},'time (s)','');
        end

        g = darkGrid(H.tabTest,[3 1],dark);
        g.RowHeight = {'1.6x','1x',120};
        H.axT1 = uiaxes(g); styleAx(H.axT1,'Choose a test on the left and press RUN TEST','','');
        H.axT2 = uiaxes(g); styleAx(H.axT2,'','','');
        H.testTxt = uitextarea(g,'Editable','off','FontName','Courier New','FontSize',11, ...
            'BackgroundColor',TH.monoBg,'FontColor',TH.mono,'Value',{'Test results appear here.'});

        buildHardwareTab(dark);
    end

    function buildHardwareTab(dark)
        g = darkGrid(H.tabHw,[6 1],dark);
        g.RowHeight = {28,30,36,'1x','1x',96};
        r1 = uigridlayout(g,[1 5]); r1.Padding = [0 0 0 0]; r1.BackgroundColor = dark;
        r1.ColumnWidth = {'1x',70,90,80,60};
        r1.Layout.Row = 1;
        C.port = uidropdown(r1,'Items',{'(press Refresh)'},'BackgroundColor',TH.field,'FontColor',TH.text);
        C.port.Layout.Column = 1;
        mkBtn(r1,2,'Refresh',[0.3 0.32 0.36],@(~,~) onRefreshPorts());
        C.baud = uidropdown(r1,'Items',{'115200','230400','460800','921600'},'Value','115200', ...
            'BackgroundColor',TH.field,'FontColor',TH.text);
        C.baud.Layout.Column = 3;
        lb = uilabel(r1,'Text','Capture (s)','FontColor',[0.85 0.86 0.9],'HorizontalAlignment','right');
        lb.Layout.Column = 4;
        C.capDur = uieditfield(r1,'numeric','Limits',[1 120],'Value',10, ...
            'BackgroundColor',TH.field,'FontColor',TH.text);
        C.capDur.Layout.Column = 5;
        r2 = uigridlayout(g,[1 4]); r2.Padding = [0 0 0 0]; r2.BackgroundColor = dark;
        r2.Layout.Row = 2;
        H.btnCapture = mkBtn(r2,1,'CAPTURE SERIAL',[0.25 0.47 0.85],@(~,~) onCapture());
        H.btnImport  = mkBtn(r2,2,'IMPORT CSV',[0.3 0.32 0.36],@(~,~) onImportCSV());
        H.btnSimHw   = mkBtn(r2,3,'SIM SAME START',[0.30 0.69 0.31],@(~,~) onSimVsReal());
        H.btnSend    = mkBtn(r2,4,'SEND GAINS',[0.80 0.62 0.10],@(~,~) onSendGains());
        lb = uilabel(g,'WordWrap','on','FontColor',[0.85 0.86 0.9],'FontSize',11, 'Text', ...
            ['Serial format, one line per control step:  t_ms,pitch_deg,pwm   (e.g. 15320,0.84,-37). ', ...
             'SEND GAINS writes  PID,Kp,Ki,Kd  - add a small parser in the firmware to accept it.']);
        lb.Layout.Row = 3;
        H.axH = gobjects(1,2);
        H.axH(1) = uiaxes(g); H.axH(1).Layout.Row = 4; styleAx(H.axH(1),'Real vs simulated pitch (°)','time (s)','');
        H.axH(2) = uiaxes(g); H.axH(2).Layout.Row = 5; styleAx(H.axH(2),'Real vs simulated motor command (PWM)','time (s)','');
        H.hwTxt = uitextarea(g,'Editable','off','FontName','Courier New','FontSize',11, ...
            'BackgroundColor',TH.monoBg,'FontColor',TH.mono,'Value',{'No real-robot data yet.'});
        H.hwTxt.Layout.Row = 6;
    end

    % ---------- small UI helpers ----------
    function r = nextRow(hgt)
        row = row + 1; r = row; rowH(r) = hgt;
    end

    function put(h, r, c)
        h.Layout.Row = r;
        if numel(c) > 1, h.Layout.Column = [c(1) c(end)]; else, h.Layout.Column = c; end
    end

    function hdr(txt)
        r = nextRow(26);
        lb = uilabel(L,'Text',txt,'FontWeight','bold','FontSize',13, ...
            'FontColor',TH.accent,'VerticalAlignment','bottom');
        put(lb,r,[1 3]);
    end

    function addNum(key, txt, lims, val, fmt, tip)
        r = nextRow(22);
        lb = uilabel(L,'Text',txt,'FontSize',11,'FontColor',TH.text); put(lb,r,1);
        sl = uislider(L,'Limits',lims,'Value',val,'MajorTicks',[],'MinorTicks',[]);
        put(sl,r,2);
        ed = uieditfield(L,'numeric','Limits',lims,'Value',val, ...
            'ValueDisplayFormat',fmt,'FontSize',11, ...
            'BackgroundColor',TH.field,'FontColor',TH.text);
        put(ed,r,3);
        sl.ValueChangingFcn = @(~,e) sliderMoved(key, e.Value);
        sl.ValueChangedFcn  = @(s,~) sliderMoved(key, s.Value);
        ed.ValueChangedFcn  = @(e,~) fieldEdited(key, e.Value);
        if nargin > 5
            lb.Tooltip = tip; ed.Tooltip = tip;
        end
        C.(key) = ed;
        C.([key '_sl']) = sl;
    end

    function addDrop(key, txt, items, data, val)
        r = nextRow(24);
        lb = uilabel(L,'Text',txt,'FontSize',11,'FontColor',TH.text); put(lb,r,1);
        dd = uidropdown(L,'Items',items,'ItemsData',data,'Value',val,'FontSize',11, ...
            'BackgroundColor',TH.field,'FontColor',TH.text);
        put(dd,r,[2 3]);
        C.(key) = dd;
    end

    function g = btnRow(n, hgt)
        r = nextRow(hgt);
        g = uigridlayout(L,[1 n]);
        g.Padding = [0 0 0 0]; g.ColumnSpacing = 4; g.BackgroundColor = TH.panel;
        put(g,r,[1 3]);
    end

    function b = mkBtn(parent, col, txt, bg, fcn)
        b = uibutton(parent,'push','Text',txt,'BackgroundColor',bg, ...
            'FontColor',[1 1 1],'FontWeight','bold','ButtonPushedFcn',fcn);
        b.Layout.Row = 1; b.Layout.Column = col;
    end

    function setVal(key, v)
        ed = C.(key);
        v = min(max(v, ed.Limits(1)), ed.Limits(2));
        ed.Value = v;
        C.([key '_sl']).Value = v;
    end

    function sliderMoved(key, v)
        C.(key).Value = v;
        afterChange(key);
    end

    function fieldEdited(key, v)
        C.([key '_sl']).Value = v;
        afterChange(key);
    end

    function afterChange(key)
        % Keep linked values in sync and refresh the 3-D model when idle.
        if strcmp(key,'h')
            hNew = C.h.Value;
            if C.linkCoM.Value && hPrev > 0
                setVal('l', C.l.Value*hNew/hPrev);
            end
            hPrev = hNew;
        end
        if ~isBusy && (ismember(key, PHYS) || strcmp(key,'surface') || strcmp(key,'th0'))
            P = readParams();
            q = scene.q;
            if strcmp(key,'th0'), q = [0; 0; deg2rad(P.th0); 0]; end
            refreshModel(P);
            updateScene(q, 0, [], P);
        end
    end

    function refreshModel(P)
        % Rebuild the robot geometry (and floor colour) without touching the state.
        scene.G = robotGeometry(P);
        if scene.surface ~= P.surface
            [~, ~, Cf] = floorMesh(P.surface);
            scene.floor.FaceVertexCData = Cf;
            scene.surface = P.surface;
        end
        H.ax3.ZLim = [0 max(0.4, P.h + 0.12)];
    end

    % =====================================================================
    %                          PARAMETER I/O
    % =====================================================================
    function P = readParams()
        P = defaultParams();
        keys = numKeys();
        for i = 1:numel(keys), P.(keys{i}) = C.(keys{i}).Value; end
        P.surface = C.surface.Value;
        P.mode    = C.mode.Value;
        P.dsrc    = C.dsrc.Value;
        P.comp    = C.comp.Value;
        P.T       = C.dur.Value;
        P.seed    = round(P.seed);
        P.f       = round(P.f);
    end

    function onPreset()
        switch C.preset.Value
            case 2, C.mode.Value = 2; setVal('Kp',120); setVal('Ki',5); setVal('Kd',1.5);
                    setVal('Kv',10); setVal('Kx',3); setVal('f',200); setVal('alpha',0.98);
            case 3, C.mode.Value = 1; setVal('Kp',120); setVal('Ki',5); setVal('Kd',1.5);
                    setVal('f',200); setVal('alpha',0.98);
            case 4, C.mode.Value = 1; setVal('Kp',18); setVal('Ki',140); setVal('Kd',0.6);
                    setVal('f',200); setVal('alpha',0.98); C.dsrc.Value = 2;
            case 5, C.mode.Value = 2; setVal('Kp',40); setVal('Ki',5); setVal('Kd',1.5);
                    setVal('Kv',10); setVal('Kx',3);
            case 6, C.mode.Value = 2; setVal('Kp',120); setVal('Ki',5); setVal('Kd',5);
                    setVal('Kv',10); setVal('Kx',3); setVal('f',50);
            case 7, C.mode.Value = 2; setVal('Kp',120); setVal('Ki',0); setVal('Kd',1.5);
                    setVal('Kv',10); setVal('Kx',3); setVal('alpha',0);
        end
    end

    function onResetDefaults()
        D = defaultParams();
        keys = numKeys();
        for i = 1:numel(keys), setVal(keys{i}, D.(keys{i})); end
        C.surface.Value = D.surface; C.mode.Value = D.mode; C.dsrc.Value = D.dsrc;
        C.comp.Value = D.comp; C.preset.Value = 1; C.dur.Value = D.T;
        hPrev = D.h;
        resetScene(D);
        updateScene([0; 0; deg2rad(D.th0); 0], 0, [], D);
    end

    % =====================================================================
    %                              LIVE RUN
    % =====================================================================
    function onRunLive()
        if isBusy, return; end
        P = readParams();
        isBusy = true; stopReq = false; pushReq = 0;
        setBusyUI(true,'live');
        H.TG.SelectedTab = H.tabLive;
        try
            rng(P.seed);
            resetScene(P);
            clearLive();
            sim = simInit(P);
            spd = C.speed.Value;
            nextT = 0; t0 = tic; tBase = 0; status = 'finished'; nEdits = 0;
            while ~sim.done
                if stopReq, status = 'stopped by user'; break; end
                if pushReq ~= 0
                    sim.livePushT = sim.t;
                    sim.livePushF = pushReq*max(abs(P.Fpush), 1.5);
                    pushReq = 0;
                end
                sim = simStep(sim, P);
                if sim.t >= nextT || sim.done
                    % ---- live edits: everything on the left panel applies now ----
                    if C.liveTune.Value
                        Pn = readParams(); Pn.th0 = P.th0; Pn.seed = P.seed;
                        [phys, other] = paramDiff(P, Pn);
                        if phys
                            sim = simRebuild(sim, Pn);
                            refreshModel(Pn);
                        end
                        if phys || other, nEdits = nEdits + 1; end
                        P = Pn;
                    end
                    % ---- playback speed can change mid-run ----
                    if C.speed.Value ~= spd
                        spd = C.speed.Value; t0 = tic; tBase = sim.t;
                    end
                    if isinf(spd), simPerFrame = 0.25; else, simPerFrame = spd/30; end
                    nextT = sim.t + simPerFrame;
                    k0 = max(1, sim.k - 4000);
                    updateScene(sim.q, sim.Fnow, sim.L(k0:sim.k,5), P);
                    updateLive(sim.L, sim.k);
                    updateStatus(sim, P);
                    if isinf(spd)
                        drawnow limitrate;
                    else
                        lag = (sim.t - tBase)/spd - toc(t0);
                        if lag > 0.001, pause(lag); else, drawnow limitrate; end
                    end
                end
            end
            if sim.fallen && ~strcmp(status,'stopped by user'), status = 'fell'; end
            if nEdits > 0, status = sprintf('%s, edited live (%d changes)', status, nEdits); end
            if sim.k >= 2
                lg  = simFinish(sim, P);
                met = computeMetrics(lg, P);
                addRun(P, lg, met, status);
            end
        catch err
            safeAlert(err.message, 'Simulation error');
        end
        endBusy();
    end

    function onFastRun()
        if isBusy, return; end
        P = readParams();
        if isinf(P.T), P.T = 60; end          % "until stopped" has no end without animation
        isBusy = true; setBusyUI(true,'fast');
        try
            H.status.Text = sprintf(' Simulating %.0f s (no animation)...', P.T); drawnow;
            [lg, met, sim] = runHeadless(P);
            resetScene(P);
            updateScene(sim.q, 0, lg.x, P);
            clearLive(); updateLive(sim.L, sim.k); updateStatus(sim, P);
            status = 'finished'; if met.fell, status = 'fell'; end
            addRun(P, lg, met, status);
        catch err
            safeAlert(err.message, 'Simulation error');
        end
        endBusy();
    end

    function onStop(), stopReq = true; end

    function setPush(dirn), if isBusy, pushReq = dirn; end, end

    function onKey(~, evt)
        switch evt.Key
            case 'escape',     stopReq = true;
            case 'leftarrow',  setPush(-1);
            case 'rightarrow', setPush(1);
        end
    end

    function onClose(~, ~)
        stopReq = true;
        delete(fig);
    end

    function setBusyUI(b, mode)
        if isempty(fig) || ~isvalid(fig), return; end
        onOff = {'on','off'};
        idle = [H.btnRun H.btnFast H.btnTest H.btnExport H.btnReset H.btnClear ...
                H.btnImport H.btnSimHw H.btnSend];
        set(idle, 'Enable', onOff{1+b});
        H.btnCapture.Enable = onOff{1+b};
        H.btnStop.Enable = onOff{2-(b && ~strcmp(mode,'test'))};
        live = b && strcmp(mode,'live');
        set([H.btnPushB H.btnPushF], 'Enable', onOff{2-live});
    end

    function endBusy()
        isBusy = false;
        if isempty(fig) || ~isvalid(fig), return; end
        setBusyUI(false,'');
        updateApply();
    end

    function updateApply()
        if isempty(fig) || ~isvalid(fig), return; end
        if ~isempty(bestGains) && ~isBusy, H.btnApply.Enable = 'on';
        else, H.btnApply.Enable = 'off'; end
    end

    function safeAlert(msg, ttlTxt)
        if ~isempty(fig) && isvalid(fig), uialert(fig, msg, ttlTxt); end
    end

    function updateStatus(sim, P)
        k = sim.k; if k < 1, return; end
        Lk = sim.L(k,:);
        if sim.fallen, tag = 'FALLEN'; else, tag = 'balancing'; end
        if isinf(P.T)
            tt = sprintf('%s / until Esc', fmtClock(Lk(1)));
        else
            tt = sprintf('%s / %s (%3.0f%%)', fmtClock(Lk(1)), fmtClock(P.T), 100*Lk(1)/P.T);
        end
        H.status.Text = sprintf([' %s | tilt %+6.2f° | est %+6.2f° | x %+6.1f cm |' ...
            ' v %+5.1f cm/s | PWM %+4.0f | %s'], tt, Lk(2), Lk(3), 100*Lk(5), 100*Lk(6), Lk(7), tag);
    end

    % =====================================================================
    %                              3-D SCENE
    % =====================================================================
    function setup3D(ax)
        ax.Color = [0.10 0.11 0.14];
        ax.XColor = [0.5 0.52 0.56]; ax.YColor = [0.5 0.52 0.56]; ax.ZColor = [0.5 0.52 0.56];
        ax.GridColor = [0.4 0.42 0.46]; ax.XGrid = 'on'; ax.YGrid = 'on'; ax.ZGrid = 'on';
        ax.DataAspectRatio = [1 1 1];
        ax.Projection = 'perspective';
        ax.YLim = [-0.25 0.25]; ax.ZLim = [0 0.4]; ax.XLim = [-0.4 0.4];
        ax.YTick = []; ax.FontSize = 9;
        xlabel(ax,'x (m)'); zlabel(ax,'z (m)');
        try, ax.BackgroundColor = [0.10 0.11 0.14]; catch, end
        view(ax, 35, 18);
    end

    function resetScene(P)
        ax = H.ax3;
        delete(ax.Children);
        [Vf, Ff, Cf] = floorMesh(P.surface);
        scene.floor = patch(ax,'Vertices',Vf,'Faces',Ff,'FaceVertexCData',Cf, ...
            'FaceColor','flat','EdgeColor','none');
        scene.surface = P.surface;
        G = robotGeometry(P);
        scene.G = G;
        scene.parts = gobjects(1, numel(G.parts));
        for i = 1:numel(G.parts)
            scene.parts(i) = patch(ax,'Vertices',G.parts(i).V,'Faces',G.parts(i).F, ...
                'FaceColor',G.parts(i).c,'EdgeColor',[0.08 0.08 0.1],'LineWidth',0.5);
        end
        scene.wheels = gobjects(1,2);
        for w = 1:2
            scene.wheels(w) = patch(ax,'Vertices',G.wheelV,'Faces',G.wheelF, ...
                'FaceColor',[0.17 0.17 0.19],'EdgeColor','none');
        end
        scene.spokes = line(ax,nan,nan,nan,'Color',[0.95 0.78 0.2],'LineWidth',2);
        scene.trail  = line(ax,nan,nan,nan,'Color',[0.30 0.75 1.0],'LineWidth',1.5);
        scene.arrow  = line(ax,nan,nan,nan,'Color',[1 0.3 0.3],'LineWidth',4);
        scene.com    = line(ax,nan,nan,nan,'Marker','o','MarkerSize',8, ...
            'MarkerFaceColor',[1 0.25 0.25],'Color',[1 1 1],'LineStyle','none');
        try
            light(ax,'Position',[1 -2 3],'Style','infinite');
            light(ax,'Position',[-2 1 1],'Style','infinite','Color',[0.5 0.5 0.55]);
            set([scene.parts scene.wheels],'FaceLighting','gouraud');
        catch
        end
        ax.ZLim = [0 max(0.4, P.h + 0.12)];
        applyCamera();
    end

    function applyCamera()
        switch C.cam.Value
            case 1, view(H.ax3, 35, 18);
            case 2, view(H.ax3, 0, 0);
            case 3, view(H.ax3, 90, 0);
            case 4, view(H.ax3, 0, 90);
        end
    end

    function updateScene(q, Fnow, trailX, P)
        G = scene.G; scene.q = q;
        x = q(1); th = q(3); c = cos(th); s = sin(th); r = P.r;
        for i = 1:numel(G.parts)
            Vb = G.parts(i).V;
            scene.parts(i).Vertices = [x + Vb(:,1)*c + Vb(:,3)*s, Vb(:,2), r - Vb(:,1)*s + Vb(:,3)*c];
        end
        for w = 1:2
            scene.wheels(w).Vertices = G.wheelV + [x, G.wheelY(w), r];
        end
        ang = x/r + [0 2*pi/3 4*pi/3];
        X = nan(1,18); Y = X; Z = X; n = 0;
        for w = 1:2
            yo = G.wheelY(w) + sign(G.wheelY(w))*(G.ww/2 + 0.002);
            for a = ang
                X(n+(1:3)) = [x, x + 0.85*r*sin(a), NaN];
                Y(n+(1:3)) = [yo, yo, NaN];
                Z(n+(1:3)) = [r, r + 0.85*r*cos(a), NaN];
                n = n + 3;
            end
        end
        set(scene.spokes,'XData',X,'YData',Y,'ZData',Z);
        xc = x + G.lc*s; zc = r + G.lc*c;
        set(scene.com,'XData',xc,'YData',0,'ZData',zc);
        if Fnow ~= 0
            d = -sign(Fnow)*0.12;
            set(scene.arrow,'XData',[xc+d xc],'YData',[0 0],'ZData',[zc zc]);
        else
            set(scene.arrow,'XData',nan,'YData',nan,'ZData',nan);
        end
        if numel(trailX) > 1
            set(scene.trail,'XData',trailX(:)','YData',zeros(1,numel(trailX)), ...
                'ZData',0.002*ones(1,numel(trailX)));
        else
            set(scene.trail,'XData',nan,'YData',nan,'ZData',nan);
        end
        H.ax3.XLim = [x-0.4, x+0.4];
    end

    % =====================================================================
    %                              LIVE PLOTS
    % =====================================================================
    function clearLive()
        set([H.lnTh H.lnEst H.lnRef H.lnU H.lnX H.lnV],'XData',nan,'YData',nan);
        for i = 1:3, H.axL(i).XLim = [0 8]; end
    end

    function updateLive(Lm, k)
        if k < 1, return; end
        w = 8; tn = Lm(k,1);
        k0 = k; while k0 > 1 && Lm(k0-1,1) >= tn - w, k0 = k0 - 1; end   % last w seconds
        i = k0:k; t = Lm(i,1);
        set(H.lnTh, 'XData',t,'YData',Lm(i,2));
        set(H.lnEst,'XData',t,'YData',Lm(i,3));
        set(H.lnRef,'XData',t,'YData',Lm(i,11));
        set(H.lnU,  'XData',t,'YData',Lm(i,7));
        set(H.lnX,  'XData',t,'YData',100*Lm(i,5));
        set(H.lnV,  'XData',t,'YData',100*Lm(i,6));
        xl = [max(0, tn-w), max(w, tn)];
        for i = 1:3, H.axL(i).XLim = xl; end
    end

    % =====================================================================
    %                       RUN HISTORY + RESULTS
    % =====================================================================
    function addRun(P, lg, met, status)
        runCount = runCount + 1;
        s = struct('id',runCount,'P',P,'log',lg,'met',met,'show',true,'status',status);
        runs(end+1) = s;
        refreshTable();
        showResults(numel(runs));
        refreshCompare();
        H.TG.SelectedTab = H.tabRes;
    end

    function refreshTable()
        n = numel(runs);
        D = cell(n, 11);
        modes = {'angle','casc.'};
        for i = 1:n
            R = runs(i); P = R.P; m = R.met;
            D(i,:) = {R.show, R.id, modes{P.mode}, fmtClock(m.duration), sprintf('%.0f',P.Kp), ...
                sprintf('%.1f',P.Ki), sprintf('%.2f',P.Kd), sprintf('%.1f',P.Fpush), ...
                sprintf('%.1f',m.maxTilt), sprintf('%.0f',m.maxDrift), m.verdict};
        end
        H.table.Data = D;
        try
            removeStyle(H.table);
            v = cellfun(@(R) R.verdict, {runs.met}, 'UniformOutput', false);
            iF = find(strcmp(v,'FAIL')); iW = find(strcmp(v,'WARN'));
            if ~isempty(iF), addStyle(H.table, uistyle('BackgroundColor',[0.45 0.12 0.14], ...
                    'FontColor',[1 0.88 0.88]),'row',iF); end
            if ~isempty(iW), addStyle(H.table, uistyle('BackgroundColor',[0.40 0.31 0.08], ...
                    'FontColor',[1 0.95 0.78]),'row',iW); end
        catch
        end
    end

    function onTableEdit(~, evt)
        i = evt.Indices(1);
        if evt.Indices(2) == 1 && i <= numel(runs)
            runs(i).show = logical(evt.NewData);
            refreshCompare();
            H.TG.SelectedTab = H.tabCmp;
        end
    end

    function onTableSelect(~, evt)
        if isempty(evt.Indices), return; end
        i = evt.Indices(1,1);
        if evt.Indices(1,2) ~= 1 && i <= numel(runs)
            showResults(i);
            H.TG.SelectedTab = H.tabRes;
        end
    end

    function onClearRuns()
        if isBusy, return; end
        runs = runs([]);
        refreshTable();
        refreshCompare();
        for a = H.axR, cla(a); legend(a,'off'); end
        H.summary.Value = {'History cleared.'};
    end

    function showResults(i)
        R = runs(i); lg = R.log; P = R.P; ax = H.axR; t = lg.t;
        for a = ax, cla(a); legend(a,'off'); hold(a,'on'); end

        h1 = plot(ax(1), t, lg.th, 'Color', CL.th, 'LineWidth', 1.4);
        h2 = plot(ax(1), t, lg.est, 'Color', CL.est, 'LineWidth', 1);
        h3 = plot(ax(1), t, lg.ref, '--', 'Color', CL.ref);
        legendDark(ax(1), [h1 h2 h3], {'true','estimate','reference'});
        ttl(ax(1), sprintf('Run #%d  pitch: true vs estimate (°)', R.id));

        h1 = plot(ax(2), t, lg.Pt, 'Color', CL.P);
        h2 = plot(ax(2), t, lg.It, 'Color', CL.I);
        h3 = plot(ax(2), t, lg.Dt, 'Color', CL.D);
        h4 = plot(ax(2), t, lg.u, 'Color', [1 1 1], 'LineWidth', 1.2);
        plot(ax(2), [t(1) t(end)], [255 255], ':', 'Color', [1 0.4 0.4]);
        plot(ax(2), [t(1) t(end)], [-255 -255], ':', 'Color', [1 0.4 0.4]);
        legendDark(ax(2), [h1 h2 h3 h4], {'P','I','D','output'});

        h1 = plot(ax(3), t, 100*lg.x, 'Color', CL.x, 'LineWidth', 1.4);
        h2 = plot(ax(3), t, 100*lg.xd, 'Color', CL.v);
        legendDark(ax(3), [h1 h2], {'position cm','speed cm/s'});

        gInt = lg.est(1) + cumsum(lg.gyro)*lg.Tc;
        h1 = plot(ax(4), t, lg.acc, 'Color', 0.55*CL.acc + 0.45*[0.11 0.12 0.15], 'LineWidth', 0.5);
        h2 = plot(ax(4), t, gInt, '--', 'Color', CL.v);
        h3 = plot(ax(4), t, lg.est, 'Color', CL.est, 'LineWidth', 1.2);
        h4 = plot(ax(4), t, lg.th, 'Color', CL.th, 'LineWidth', 1);
        legendDark(ax(4), [h1 h2 h3 h4], {'accel only','gyro only','filtered','true'});
        yl = [min(lg.th) max(lg.th)]; pad = max(2, 0.3*diff(yl));
        ax(4).YLim = [yl(1)-pad, yl(2)+pad];

        plot(ax(5), lg.th, lg.thd, 'Color', CL.th);
        plot(ax(5), lg.th(1), lg.thd(1), 'o', 'MarkerFaceColor', CL.x, 'Color', CL.x);
        plot(ax(5), lg.th(end), lg.thd(end), 's', 'MarkerFaceColor', CL.acc, 'Color', CL.acc);

        h1 = plot(ax(6), t, lg.V, 'Color', CL.u);
        h2 = plot(ax(6), t, 100*lg.tau, 'Color', CL.x);
        legendDark(ax(6), [h1 h2], {'voltage V','torque N·cm'});

        if P.Fpush ~= 0 && P.tPush < t(end)
            for k = [1 2 3 6]
                yl = ax(k).YLim;
                plot(ax(k), [P.tPush P.tPush], yl, ':', 'Color', [1 0.35 0.35]);
                ax(k).YLim = yl;
            end
        end
        for a = ax, hold(a,'off'); end
        for k = [1 2 3 4 6], ax(k).XLim = [0 max(t(end), 0.1)]; end
        H.summary.Value = summaryLines(R);
    end

    function refreshCompare()
        ax = H.axC;
        for a = ax, cla(a); legend(a,'off'); hold(a,'on'); end
        sel = find([runs.show]);
        if numel(sel) > 8, sel = sel(end-7:end); end
        hs = gobjects(1, numel(sel)); labels = cell(1, numel(sel));
        modes = {'angle','casc'};
        for j = 1:numel(sel)
            R = runs(sel(j)); col = CL.pal(1+mod(j-1,8),:);
            hs(j) = plot(ax(1), R.log.t, R.log.th, 'Color', col, 'LineWidth', 1.2);
            plot(ax(2), R.log.t, 100*R.log.x, 'Color', col, 'LineWidth', 1.2);
            plot(ax(3), R.log.t, R.log.u, 'Color', col);
            labels{j} = sprintf('#%d %s Kp%.0f Ki%.1f Kd%.2f %s', R.id, modes{R.P.mode}, ...
                R.P.Kp, R.P.Ki, R.P.Kd, R.met.verdict);
        end
        if ~isempty(sel), legendDark(ax(1), hs, labels); end
        for a = ax, hold(a,'off'); end
    end

    % =====================================================================
    %                                 TESTS
    % =====================================================================
    function onRunTest()
        if isBusy, return; end
        P = readParams();
        isBusy = true; setBusyUI(true,'test');
        H.TG.SelectedTab = H.tabTest;
        dlg = uiprogressdlg(fig,'Title','Running test','Message','Starting...','Cancelable','on');
        msg = '';
        try
            switch C.test.Value
                case 1, testGainMap(P, dlg, 'KpKd');
                case 2, testGainMap(P, dlg, 'KvKx');
                case 3, testPush(P, dlg);
                case 4, testLoopRate(P, dlg);
                case 5, testFilter(P, dlg);
                case 6, testMonteCarlo(P, dlg);
                case 7, testCEM(P, dlg);
            end
        catch err
            msg = err.message;
        end
        if isvalid(dlg), close(dlg); end
        if ~isempty(msg), safeAlert(msg, 'Test error'); end
        endBusy();
    end

    function [a1, a2] = prepTestAxes()
        a1 = H.axT1; a2 = H.axT2;
        for a = [a1 a2]
            cla(a); legend(a,'off'); colorbar(a,'off');
            a.XScale = 'linear'; a.YDir = 'normal';
            a.XLimMode = 'auto'; a.YLimMode = 'auto';
            a.XTickMode = 'auto'; a.XTickLabelMode = 'auto';
            hold(a,'on');
        end
    end

    function testGainMap(P, dlg, which)
        Pt = P; Pt.T = min(P.T, 6);
        if Pt.Fpush == 0 || Pt.tPush >= Pt.T, Pt.Fpush = 1.5; end
        Pt.tPush = 2.5;
        if strcmp(which,'KpKd')
            xn = 'Kp'; yn = 'Kd'; xv = linspace(20,260,13); yv = linspace(0,4,11);
        else
            xn = 'Kv'; yn = 'Kx'; xv = linspace(0,30,11); yv = linspace(0,12,9); Pt.mode = 2;
        end
        Z = nan(numel(yv), numel(xv)); J = inf(size(Z));
        total = numel(Z); n = 0; cancelled = false;
        for i = 1:numel(xv)
            for j = 1:numel(yv)
                if dlg.CancelRequested, cancelled = true; break; end
                Pc = Pt; Pc.(xn) = xv(i); Pc.(yn) = yv(j);
                [~, met] = runHeadless(Pc);
                if ~met.fell, Z(j,i) = met.rmsTilt; end
                J(j,i) = met.cost; n = n + 1;
                dlg.Value = n/total;
                dlg.Message = sprintf('%s = %.1f   %s = %.2f   (%d of %d)', xn, xv(i), yn, yv(j), n, total);
            end
            if cancelled, break; end
        end
        [ax, ax2] = prepTestAxes();
        imagesc(ax, xv, yv, Z, 'AlphaData', ~isnan(Z));
        ax.YDir = 'normal'; colormap(ax, parula);
        cb = colorbar(ax); cb.Color = [0.85 0.86 0.9]; cb.Label.String = 'RMS tilt (°)';
        dx = xv(2)-xv(1); dy = yv(2)-yv(1);
        ax.XLim = [xv(1)-dx/2, xv(end)+dx/2]; ax.YLim = [yv(1)-dy/2, yv(end)+dy/2];
        plot(ax, P.(xn), P.(yn), 'w+', 'MarkerSize', 14, 'LineWidth', 2);
        [Jb, ib] = min(J(:));
        if isfinite(Jb) && Jb < 100
            [jb, ibx] = ind2sub(size(J), ib);
            plot(ax, xv(ibx), yv(jb), 'p', 'MarkerSize', 16, ...
                'MarkerFaceColor', [1 0.85 0.2], 'MarkerEdgeColor', 'k');
            bestGains = struct(xn, xv(ibx), yn, yv(jb));
            if strcmp(which,'KvKx'), bestGains.mode = 2; end
            bestTxt = sprintf('Best (lowest cost): %s = %.1f, %s = %.2f  -> APPLY BEST GAINS', ...
                xn, xv(ibx), yn, yv(jb));
        else
            bestGains = [];
            bestTxt = 'No combination balanced.';
        end
        hold(ax,'off');
        ttl(ax, sprintf('Stability map %s × %s   (dark = fell, + = current, ★ = best)', xn, yn));
        xlabel(ax, xn); ylabel(ax, yn);
        bar(ax2, xv, 100*mean(~isnan(Z),1), 'FaceColor', CL.th);
        ttl(ax2, sprintf('%% of %s values that balance, for each %s', yn, xn));
        xlabel(ax2, xn); ylabel(ax2, '%'); ax2.YLim = [0 105]; hold(ax2,'off');
        H.testTxt.Value = {sprintf('Stability map %s x %s: %d sims, %.0f s each, push %.1f N at %.1f s', ...
            xn, yn, n, Pt.T, Pt.Fpush, Pt.tPush), ...
            sprintf('Balanced in %d of %d cases (%.0f %%).', sum(~isnan(Z(:))), n, 100*sum(~isnan(Z(:)))/max(n,1)), ...
            bestTxt, ...
            'Colour = RMS tilt (lower is better). Transparent/dark cells = robot fell.'};
    end

    function testPush(P, dlg)
        forces = 0.5:0.5:6; Pt = P; Pt.T = 7; Pt.tPush = 2.5;
        MT = nan(2, numel(forces)); RT = MT; FELL = false(size(MT));
        total = numel(MT); n = 0;
        for mi = 1:2
            for k = 1:numel(forces)
                if dlg.CancelRequested, break; end
                Pc = Pt; Pc.mode = mi; Pc.Fpush = forces(k);
                [~, met] = runHeadless(Pc);
                FELL(mi,k) = met.fell;
                if ~met.fell, MT(mi,k) = met.pushTilt; RT(mi,k) = met.recovery; end
                n = n + 1; dlg.Value = n/total;
                dlg.Message = sprintf('mode %d, push %.1f N  (%d of %d)', mi, forces(k), n, total);
            end
        end
        [ax, ax2] = prepTestAxes();
        cols = [CL.est; CL.th]; names = {'angle PID only','angle + speed loop'};
        hs = gobjects(1,2);
        for mi = 1:2
            hs(mi) = plot(ax, forces, MT(mi,:), '-o', 'Color', cols(mi,:), ...
                'MarkerFaceColor', cols(mi,:), 'LineWidth', 1.4);
            f = forces(FELL(mi,:));
            if ~isempty(f), plot(ax, f, 45*ones(size(f)), 'x', 'Color', cols(mi,:), 'MarkerSize', 10, 'LineWidth', 2); end
            plot(ax2, forces, RT(mi,:), '-o', 'Color', cols(mi,:), 'MarkerFaceColor', cols(mi,:));
        end
        legendDark(ax, hs, names);
        ttl(ax, 'Max tilt after the push (°)   (× at 45° = fell)');
        xlabel(ax, 'push force (N), 0.1 s at the CoM'); ylabel(ax, 'deg');
        ttl(ax2, 'Recovery time to within 1° (s)'); xlabel(ax2, 'push force (N)');
        hold(ax,'off'); hold(ax2,'off');
        txt = {'Push recovery sweep (both controller modes, current robot + noise settings)'};
        for mi = 1:2
            i1 = find(FELL(mi,:), 1);
            if isempty(i1), mx = forces(end); s = '>= '; elseif i1 == 1, mx = 0; s = '< ';
            else, mx = forces(i1-1); s = ''; end
            if mx == 0, s2 = sprintf('%s%.1f N', s, forces(1)); else, s2 = sprintf('%s%.1f N', s, mx); end
            txt{end+1} = sprintf('  %-22s max recoverable push: %s', names{mi}, s2); %#ok<AGROW>
        end
        txt{end+1} = 'Angle-only control often drifts and runs away even after small pushes: no encoders = no speed feedback.';
        H.testTxt.Value = txt;
    end

    function testLoopRate(P, dlg)
        rates = [25 40 50 75 100 150 200 300 500 1000];
        RMS = nan(size(rates)); EE = RMS; SAT = RMS; FELL = false(size(rates));
        for k = 1:numel(rates)
            if dlg.CancelRequested, break; end
            Pc = P; Pc.f = rates(k); Pc.T = min(P.T, 8);
            [~, met] = runHeadless(Pc);
            FELL(k) = met.fell;
            if ~met.fell, RMS(k) = met.rmsTilt; end
            EE(k) = met.estErr; SAT(k) = met.satPct;
            dlg.Value = k/numel(rates); dlg.Message = sprintf('%d Hz', rates(k));
        end
        [ax, ax2] = prepTestAxes();
        plot(ax, rates, RMS, '-o', 'Color', CL.th, 'MarkerFaceColor', CL.th, 'LineWidth', 1.4);
        f = rates(FELL);
        if ~isempty(f)
            yl = max([RMS(~isnan(RMS)) 1]);
            plot(ax, f, yl*ones(size(f)), 'x', 'Color', [1 0.3 0.3], 'MarkerSize', 12, 'LineWidth', 2);
        end
        ax.XScale = 'log'; ttl(ax, 'RMS tilt vs control loop rate   (red × = fell)');
        xlabel(ax, 'loop rate (Hz)'); ylabel(ax, 'deg');
        h1 = plot(ax2, rates, EE, '-o', 'Color', CL.est);
        h2 = plot(ax2, rates, SAT, '-s', 'Color', CL.u);
        ax2.XScale = 'log'; legendDark(ax2, [h1 h2], {'estimate error (°)','PWM saturated (%)'});
        ttl(ax2, 'Estimator error and motor saturation'); xlabel(ax2, 'loop rate (Hz)');
        hold(ax,'off'); hold(ax2,'off');
        ok = rates(~FELL);
        if isempty(ok), s = 'none'; else, s = sprintf('%d Hz', min(ok)); end
        H.testTxt.Value = {'Loop rate sweep (same gains, same robot).', ...
            sprintf('Lowest rate that still balanced: %s', s), ...
            'Gains tuned at one rate change meaning at another (Ki and D-from-error scale with dt).'};
    end

    function testFilter(P, dlg)
        alphas = [0 0.5 0.8 0.9 0.95 0.97 0.98 0.99 0.995 0.998];
        scales = [0.5 1 2 4];
        RMS = nan(numel(scales), numel(alphas)); EE = RMS;
        total = numel(RMS); n = 0;
        g0 = max(P.gyroNoise, 0.5); a0 = max(P.accNoise, 1);
        for si = 1:numel(scales)
            for k = 1:numel(alphas)
                if dlg.CancelRequested, break; end
                Pc = P; Pc.alpha = alphas(k); Pc.T = min(P.T, 8);
                Pc.gyroNoise = g0*scales(si); Pc.accNoise = a0*scales(si);
                [~, met] = runHeadless(Pc);
                if ~met.fell, RMS(si,k) = met.rmsTilt; end
                EE(si,k) = met.estErr;
                n = n + 1; dlg.Value = n/total;
                dlg.Message = sprintf('noise x%.1f, alpha %.3f  (%d of %d)', scales(si), alphas(k), n, total);
            end
        end
        [ax, ax2] = prepTestAxes();
        hs = gobjects(1,numel(scales)); hs2 = hs; labels = cell(1,numel(scales));
        for si = 1:numel(scales)
            col = CL.pal(si,:);
            hs(si)  = plot(ax, 1:numel(alphas), RMS(si,:), '-o', 'Color', col, 'MarkerFaceColor', col);
            hs2(si) = plot(ax2, 1:numel(alphas), EE(si,:), '-o', 'Color', col);
            labels{si} = sprintf('noise x%.1f', scales(si));
        end
        for a = [ax ax2]
            a.XTick = 1:numel(alphas);
            a.XTickLabel = arrayfun(@(v) sprintf('%.3g', v), alphas, 'UniformOutput', false);
            xlabel(a, 'complementary filter alpha');
            hold(a,'off');
        end
        legendDark(ax, hs, labels);
        ttl(ax, 'RMS tilt (°) vs filter alpha  (gaps = fell)');
        ttl(ax2, 'Estimate error RMS (°)');
        H.testTxt.Value = {'Filter alpha vs sensor noise.', ...
            'alpha -> 0 trusts the noisy accelerometer (and its motion error); alpha -> 1 trusts the drifting gyro.', ...
            sprintf('Base noise: gyro %.2f °/s, accel %.2f °, gyro bias %.2f °/s.', g0, a0, P.gyroBias)};
    end

    function testMonteCarlo(P, dlg)
        nMC = 40;
        FELL = false(1,nMC); MT = nan(1,nMC); DR = nan(1,nMC); VER = cell(1,nMC);
        for k = 1:nMC
            if dlg.CancelRequested, nMC = k-1; break; end
            rs = RandStream('mt19937ar','Seed',k);
            Pc = P; Pc.seed = P.seed*1000 + k; Pc.T = min(P.T, 8); Pc.tPush = 2.5;
            Pc.Mb = P.Mb*(1 + uSym(rs,0.15));  Pc.l  = P.l*(1 + uSym(rs,0.15));
            Pc.Ts = P.Ts*(1 + uSym(rs,0.20));  Pc.w0 = P.w0*(1 + uSym(rs,0.15));
            Pc.db = max(0, P.db + uSym(rs,10)); Pc.mountOff = P.mountOff + uSym(rs,1);
            Pc.gyroBias = P.gyroBias + uSym(rs,0.5);
            sgn = 1; if rand(rs) < 0.5, sgn = -1; end
            Pc.Fpush = sgn*max(abs(P.Fpush),1.5)*(0.6 + 0.8*rand(rs));
            [~, met] = runHeadless(Pc);
            FELL(k) = met.fell; MT(k) = met.maxTilt; DR(k) = met.maxDrift; VER{k} = met.verdict;
            dlg.Value = k/40; dlg.Message = sprintf('robot %d of 40', k);
        end
        FELL = FELL(1:nMC); MT = MT(1:nMC); DR = DR(1:nMC); VER = VER(1:nMC);
        [ax, ax2] = prepTestAxes();
        if any(~FELL), histogram(ax, MT(~FELL), 12, 'FaceColor', CL.th, 'EdgeColor', 'none'); end
        ttl(ax, 'Max tilt of robots that stayed up (°)'); xlabel(ax, 'deg'); ylabel(ax, 'count');
        pass = strcmp(VER,'PASS'); warn = strcmp(VER,'WARN');
        h = gobjects(1,3); lb = {'PASS','WARN','FAIL'};
        h(1) = plot(ax2, DR(pass), MT(pass), 'o', 'MarkerFaceColor', CL.x, 'Color', CL.x);
        h(2) = plot(ax2, DR(warn), MT(warn), 'o', 'MarkerFaceColor', CL.v, 'Color', CL.v);
        h(3) = plot(ax2, DR(FELL), MT(FELL), 'x', 'Color', [1 0.3 0.3], 'MarkerSize', 9, 'LineWidth', 2);
        keep = [any(pass) any(warn) any(FELL)];
        if any(keep), legendDark(ax2, h(keep), lb(keep)); end
        ttl(ax2, 'Each robot: max drift vs max tilt'); xlabel(ax2, 'max drift (cm)'); ylabel(ax2, 'max tilt (°)');
        hold(ax,'off'); hold(ax2,'off');
        H.testTxt.Value = {sprintf('Monte-Carlo: %d robots with ±15%% mass/CoM, ±20%% torque, ±10 PWM deadband,', nMC), ...
            '±1° mount offset, ±0.5 °/s bias, random push 0.6-1.4x in random direction.', ...
            sprintf('PASS %d   WARN %d   FAIL %d   ->  success rate %.0f %%', sum(pass), sum(warn), sum(FELL), ...
            100*sum(pass)/max(nMC,1)), ...
            'A controller that only works for the exact nominal robot will not survive the real one.'};
    end

    function testCEM(P, dlg)
        Pt = P; Pt.T = 6;
        if Pt.Fpush == 0 || Pt.tPush >= Pt.T, Pt.Fpush = 1.5; end
        Pt.tPush = 2.5;
        if P.mode == 2
            names = {'Kp','Ki','Kd','Kv','Kx'}; ub = [400 200 8 40 20]; s0 = [30 3 0.6 4 1.5];
        else
            names = {'Kp','Ki','Kd'}; ub = [400 200 8]; s0 = [30 3 0.6];
        end
        d = numel(names); mu = zeros(1,d);
        for i = 1:d, mu(i) = P.(names{i}); end
        sig = max(0.3*mu, s0);
        nIter = 10; nPop = 16; nElite = 4;
        hb = nan(1,nIter); hm = hb; hmu = nan(nIter,d);
        rsC = RandStream('mt19937ar','Seed',P.seed);
        J0 = evalPair(applyX(Pt,names,mu), P.seed);
        bestJ = J0; bestX = mu;
        [ax, ax2] = prepTestAxes();
        total = nIter*nPop; n = 0; cancelled = false;
        for it = 1:nIter
            X = mu + sig.*randn(rsC, nPop, d);
            X = min(max(X, 0), ub);
            Jv = zeros(nPop,1);
            for p = 1:nPop
                if dlg.CancelRequested, cancelled = true; break; end
                Jv(p) = evalPair(applyX(Pt,names,X(p,:)), P.seed + it);
                n = n + 1; dlg.Value = n/total;
                dlg.Message = sprintf('iteration %d of %d, candidate %d of %d   best cost %.2f', ...
                    it, nIter, p, nPop, bestJ);
            end
            if cancelled, break; end
            [Js, ord] = sort(Jv);
            E = X(ord(1:nElite),:);
            mu = mean(E,1);
            sig = max(std(E,0,1), 0.02*mu + 0.01);
            hb(it) = Js(1); hm(it) = mean(Jv); hmu(it,:) = mu;
            if Js(1) < bestJ, bestJ = Js(1); bestX = X(ord(1),:); end
            cla(ax); hold(ax,'on');
            plot(ax, 1:it, hb(1:it), '-o', 'Color', CL.x, 'MarkerFaceColor', CL.x, 'LineWidth', 1.4);
            plot(ax, 1:it, hm(1:it), '-s', 'Color', CL.est);
            plot(ax, [0.5 nIter+0.5], [J0 J0], '--', 'Color', CL.ref);
            ax.XLim = [0.5 nIter+0.5];
            drawnow limitrate;
        end
        cla(ax); hold(ax,'on');
        h1 = plot(ax, 1:nIter, hb, '-o', 'Color', CL.x, 'MarkerFaceColor', CL.x, 'LineWidth', 1.4);
        h2 = plot(ax, 1:nIter, hm, '-s', 'Color', CL.est);
        h3 = plot(ax, [0.5 nIter+0.5], [J0 J0], '--', 'Color', CL.ref);
        ax.XLim = [0.5 nIter+0.5];
        legendDark(ax, [h1 h2 h3], {'best in iteration','population mean','your current gains'});
        ttl(ax, 'CEM auto-tune: cost per iteration (lower = better)');
        xlabel(ax, 'iteration'); ylabel(ax, 'cost');
        nrm = max(abs(hmu), [], 1); nrm(nrm == 0) = 1;
        hs = gobjects(1,d);
        for i = 1:d
            hs(i) = plot(ax2, 1:nIter, hmu(:,i)'./nrm(i), '-o', 'Color', CL.pal(i,:));
        end
        legendDark(ax2, hs, names);
        ttl(ax2, 'Elite mean of each gain (normalised to its max)'); xlabel(ax2, 'iteration');
        hold(ax,'off'); hold(ax2,'off');
        bestGains = cell2struct(num2cell(bestX), names, 2);
        if P.mode == 2, bestGains.mode = 2; end
        g = strjoin(cellfun(@(nm,v) sprintf('%s %.2f', nm, v), names, num2cell(bestX), ...
            'UniformOutput', false), '   ');
        H.testTxt.Value = {'CEM (cross-entropy method) - population-based RL / policy search over the PID gains.', ...
            sprintf('%d iterations x %d candidates x 2 episodes (push forward + backward, noise on).', nIter, nPop), ...
            sprintf('Cost of your gains: %.2f     best found: %.2f', J0, bestJ), ...
            ['Best gains: ' g], ...
            'Press APPLY BEST GAINS, then RUN LIVE to watch the tuned robot.'};
    end

    function onApplyBest()
        if isempty(bestGains), return; end
        f = fieldnames(bestGains);
        for i = 1:numel(f)
            if strcmp(f{i},'mode'), C.mode.Value = bestGains.mode;
            else, setVal(f{i}, bestGains.(f{i})); end
        end
        C.preset.Value = 1;
        safeAlert('Best gains copied to the controller panel. Press RUN LIVE to try them.', 'Gains applied');
    end

    % =====================================================================
    %                                EXPORT
    % =====================================================================
    function onExport()
        if isempty(runs), safeAlert('No runs yet - run a simulation first.', 'Export'); return; end
        d = uigetdir(pwd, 'Choose a folder for the export');
        try, figure(fig); catch, end
        if isequal(d, 0), return; end
        out = fullfile(d, ['balance_lab_' char(datetime('now','Format','yyyyMMdd_HHmmss'))]);
        mkdir(out);
        n = numel(runs);
        id = zeros(n,1); mode = id; Kp = id; Ki = id; Kd = id; Kv = id; Kx = id; f = id;
        alpha = id; Fpush = id; maxTilt = id; rmsTilt = id; settle = id; recovery = id;
        drift = id; peakPWM = id; verdict = cell(n,1); status = cell(n,1);
        for i = 1:n
            R = runs(i); P = R.P; m = R.met;
            lg = rmfield(R.log, {'fallen','tFall','Tc','th0'});
            writetable(struct2table(lg), fullfile(out, sprintf('run_%02d.csv', R.id)));
            id(i) = R.id; mode(i) = P.mode; Kp(i) = P.Kp; Ki(i) = P.Ki; Kd(i) = P.Kd;
            Kv(i) = P.Kv; Kx(i) = P.Kx; f(i) = P.f; alpha(i) = P.alpha; Fpush(i) = P.Fpush;
            maxTilt(i) = m.maxTilt; rmsTilt(i) = m.rmsTilt; settle(i) = m.settle;
            recovery(i) = m.recovery; drift(i) = m.drift; peakPWM(i) = m.peakPWM;
            verdict{i} = m.verdict; status{i} = R.status;
        end
        S = table(id, mode, Kp, Ki, Kd, Kv, Kx, f, alpha, Fpush, maxTilt, rmsTilt, ...
            settle, recovery, drift, peakPWM, verdict, status);
        writetable(S, fullfile(out, 'summary.csv'));
        runsExport = runs; %#ok<NASGU>
        save(fullfile(out, 'runs.mat'), 'runsExport');
        try, exportapp(fig, fullfile(out, 'dashboard.png')); catch, end
        axs = [H.axR H.axC H.axT1 H.axT2];
        nm = {'res_pitch','res_pid','res_position','res_fusion','res_phase','res_motor', ...
              'cmp_pitch','cmp_position','cmp_pwm','test_main','test_second'};
        for i = 1:numel(axs)
            try, exportgraphics(axs(i), fullfile(out, [nm{i} '.png']), 'Resolution', 150); catch, end
        end
        safeAlert(['Saved CSV logs, summary.csv, runs.mat and PNG graphs to:  ' out], 'Export done');
    end

    function onHelp()
        hf = uifigure('Name','BALANCE-LAB help','Position',[200 150 660 600],'Color',TH.bg);
        try, theme(hf, 'dark'); catch, end
        gl = uigridlayout(hf,[1 1]); gl.BackgroundColor = TH.bg;
        uitextarea(gl,'Editable','off','FontSize',12,'BackgroundColor',TH.monoBg, ...
            'FontColor',TH.text,'Value',{ ...
            'BALANCE-LAB - two-wheel self-balancing robot simulator', '', ...
            'WORKFLOW', ...
            ' 1. Set robot parameters (measure your real robot: mass, CoM height, wheel radius).', ...
            ' 2. Pick a controller mode and gains (or a preset).', ...
            ' 3. Set the scenario: start tilt, push, sensor noise, duration.', ...
            ' 4. RUN LIVE to watch it, or FAST RUN for instant results.', ...
            ' 5. Every run is stored in the history table: click a row for its graphs,', ...
            '    tick Show to overlay runs in the Compare tab.', ...
            ' 6. TESTS run many simulations and plot the trends. CEM auto-tune searches gains.', ...
            ' 7. EXPORT saves CSV logs, a summary and PNGs for your report.', '', ...
            'DURING A LIVE RUN', ...
            ' Esc = stop.  Left/Right arrow or the PUSH buttons = shove the robot.', ...
            ' With Live edits on, ANY change applies immediately: gains, noise, pushes, and the', ...
            ' robot itself (height, wheel size, mass, motors) - the 3-D model and physics update.', ...
            ' Duration: 5 s ... 10 min, or Until stopped (Esc). Playback speed can change mid-run.', '', ...
            'CONTROLLER MODES', ...
            ' Angle PID only  = the current firmware: PWM = Kp*e + Ki*int(e) + Kd*rate.', ...
            ' Angle + speed/position loop = adds  ref = -(Kv*speed + Kx*position)  as the tilt', ...
            '   setpoint. This needs wheel encoders on the real robot.', '', ...
            'MODEL', ...
            ' Wheeled inverted pendulum, RK4 at ~1 ms; DC motor torque = kT*(V - kE*w);', ...
            ' L298N drop 1.8 V; PWM deadband; rolling resistance per floor;', ...
            ' gyro noise + bias; accelerometer sees gravity + robot acceleration;', ...
            ' complementary filter; motors cut when tilt > 45° (like the firmware).', '', ...
            'REAL ROBOT TAB', ...
            ' Print  t_ms,pitch_deg,pwm  over serial each loop, CAPTURE, then SIM SAME START', ...
            ' to compare. Fit mass / CoM / motor values until sim and real match.'});
    end

    % =====================================================================
    %                         REAL ROBOT (SERIAL)
    % =====================================================================
    function onRefreshPorts()
        try
            ports = serialportlist("available");
        catch
            ports = strings(0);
        end
        if isempty(ports), C.port.Items = {'(none found)'};
        else, C.port.Items = cellstr(ports); end
    end

    function onCapture()
        if isBusy, return; end
        port = C.port.Value;
        if startsWith(port, '('), safeAlert('Press Refresh and choose a serial port first.', 'Serial'); return; end
        isBusy = true; stopReq = false; setBusyUI(true,'hw');
        H.TG.SelectedTab = H.tabHw;
        msg = '';
        try
            s = serialport(port, str2double(C.baud.Value), 'Timeout', 1);
            configureTerminator(s, "LF");
            flush(s);
            buf = nan(300000, 3); n = 0; t0 = tic; lastDraw = 0;
            for a = H.axH, cla(a); legend(a,'off'); end
            hP = line(H.axH(1), nan, nan, 'Color', CL.real, 'LineWidth', 1.2);
            hU = line(H.axH(2), nan, nan, 'Color', CL.real, 'LineWidth', 1);
            while toc(t0) < C.capDur.Value && ~stopReq
                if s.NumBytesAvailable > 0
                    ln = readline(s);
                    v = sscanf(char(ln), '%f,%f,%f');
                    if numel(v) == 3, n = n + 1; buf(n,:) = v';
                    elseif numel(v) == 2, n = n + 1; buf(n,:) = [toc(t0)*1000, v'];
                    end
                else
                    pause(0.002);
                end
                if toc(t0) - lastDraw > 0.1 && n > 1
                    lastDraw = toc(t0);
                    tt = (buf(1:n,1) - buf(1,1))/1000;
                    set(hP, 'XData', tt, 'YData', buf(1:n,2));
                    set(hU, 'XData', tt, 'YData', buf(1:n,3));
                    H.status.Text = sprintf(' Capturing %s  |  %d lines  |  %.1f s', port, n, toc(t0));
                    drawnow limitrate;
                end
            end
            delete(s);
            if n < 5, error('Received fewer than 5 valid lines. Check port, baud rate and line format.'); end
            setRealLog(buf(1:n,:));
        catch err
            msg = err.message;
        end
        if ~isempty(msg), safeAlert(msg, 'Serial capture'); end
        endBusy();
    end

    function onImportCSV()
        [f, p] = uigetfile({'*.csv;*.txt','Robot log (t_ms,pitch_deg,pwm)'}, 'Import real robot log');
        try, figure(fig); catch, end
        if isequal(f, 0), return; end
        try
            M = readmatrix(fullfile(p, f));
            if size(M,2) < 3, error('Need 3 columns: t_ms, pitch_deg, pwm.'); end
            M = M(all(~isnan(M(:,1:3)), 2), 1:3);
            if size(M,1) < 5, error('Fewer than 5 valid rows.'); end
            setRealLog(M);
            H.TG.SelectedTab = H.tabHw;
        catch err
            safeAlert(err.message, 'Import');
        end
    end

    function setRealLog(M)
        realLog = struct('t', (M(:,1) - M(1,1))/1000, 'pitch', M(:,2), 'pwm', M(:,3));
        for a = H.axH, cla(a); legend(a,'off'); end
        plot(H.axH(1), realLog.t, realLog.pitch, 'Color', CL.real, 'LineWidth', 1.2);
        plot(H.axH(2), realLog.t, realLog.pwm, 'Color', CL.real);
        dt = median(diff(realLog.t));
        H.hwTxt.Value = {sprintf('Real log: %d samples, %.2f s, loop about %.0f Hz', ...
            numel(realLog.t), realLog.t(end), 1/max(dt, eps)), ...
            sprintf('Pitch: mean %+.2f°, RMS %.2f°, max |%.2f|°', mean(realLog.pitch), ...
            sqrt(mean(realLog.pitch.^2)), max(abs(realLog.pitch))), ...
            'Press SIM SAME START to run the simulator from the same initial tilt.'};
    end

    function onSimVsReal()
        if isempty(realLog), safeAlert('Capture or import a real log first.', 'Sim vs real'); return; end
        P = readParams();
        P.th0 = min(max(realLog.pitch(1), -30), 30);
        P.T = min(max(realLog.t(end), 1), 60);
        P.Fpush = 0;
        [lg, met] = runHeadless(P);
        ax = H.axH;
        for a = ax, cla(a); legend(a,'off'); hold(a,'on'); end
        h1 = plot(ax(1), realLog.t, realLog.pitch, 'Color', CL.real, 'LineWidth', 1.2);
        h2 = plot(ax(1), lg.t, lg.th, 'Color', CL.sim, 'LineWidth', 1.2);
        legendDark(ax(1), [h1 h2], {'real','simulated'});
        h1 = plot(ax(2), realLog.t, realLog.pwm, 'Color', CL.real);
        h2 = plot(ax(2), lg.t, lg.u, 'Color', CL.sim);
        legendDark(ax(2), [h1 h2], {'real','simulated'});
        for a = ax, hold(a,'off'); end
        ths = interp1(lg.t, lg.th, realLog.t, 'linear', NaN);
        us  = interp1(lg.t, lg.u, realLog.t, 'linear', NaN);
        ok = ~isnan(ths);
        H.hwTxt.Value = {sprintf('Sim vs real over %.1f s (sim verdict: %s)', P.T, met.verdict), ...
            sprintf('Pitch RMS: real %.2f°  sim %.2f°   |   RMSE between them %.2f°', ...
            sqrt(mean(realLog.pitch.^2)), met.rmsTilt, sqrt(mean((ths(ok)-realLog.pitch(ok)).^2))), ...
            sprintf('PWM RMS: real %.0f  sim %.0f', sqrt(mean(realLog.pwm.^2)), sqrt(mean(us(ok).^2))), ...
            'Tune mass, CoM height, deadband and motor values until both curves look alike.'};
    end

    function onSendGains()
        port = C.port.Value;
        if startsWith(port, '('), safeAlert('Press Refresh and choose a serial port first.', 'Serial'); return; end
        try
            s = serialport(port, str2double(C.baud.Value));
            configureTerminator(s, "LF");
            cmd = sprintf('PID,%.4f,%.4f,%.4f', C.Kp.Value, C.Ki.Value, C.Kd.Value);
            writeline(s, cmd);
            delete(s);
            safeAlert(['Sent:  ' cmd], 'Gains sent');
        catch err
            safeAlert(err.message, 'Serial');
        end
    end
end

% =========================================================================
%                     LOCAL FUNCTIONS (no UI state)
% =========================================================================
function P = defaultParams()
    % --- robot (SI units) ---
    P.Mb = 0.45;  P.pay = 0;  P.l = 0.07;  P.h = 0.17;  P.r = 0.0325;
    P.mw = 0.06;              % both wheels together
    P.Vbat = 7.4; P.drop = 1.8; P.db = 35;
    P.Ts = 0.078; P.w0 = 21;  P.Vn = 6;    % BO motor 1:48 at 6 V
    P.surface = 1;
    % --- controller (angles in degrees, output in PWM) ---
    P.mode = 2;  P.Kp = 120; P.Ki = 5; P.Kd = 1.5; P.Kv = 10; P.Kx = 3;
    P.f = 200;   P.alpha = 0.98; P.trim = 0; P.dsrc = 1; P.comp = false;
    P.iLim = 255; P.refLim = 8; P.fallAngle = 45;   % firmware: |Ki*I| <= 255
    % --- scenario ---
    P.th0 = 5; P.Fpush = 1.5; P.tPush = 3; P.pushDur = 0.1;
    P.gyroNoise = 0.5; P.accNoise = 1; P.gyroBias = 0.3; P.mountOff = 0;
    P.rough = 0; P.T = 10; P.seed = 1;
end

function k = numKeys()
    k = {'Mb','pay','l','h','r','Vbat','db','Ts','w0','Kp','Ki','Kd','Kv','Kx','f', ...
         'alpha','trim','th0','Fpush','tPush','pushDur','gyroNoise','accNoise', ...
         'gyroBias','mountOff','rough','seed'};
end

% ------------------------------- simulator -------------------------------
function m = buildModel(P)
    m.g   = 9.81;
    m.M   = P.Mb + P.pay;
    m.l   = (P.Mb*P.l + P.pay*P.h)/m.M;                         % CoM incl. payload
    m.Ib  = P.Mb*P.h^2/12 + P.Mb*(P.l - m.l)^2 + P.pay*(P.h - m.l)^2;
    m.mw  = P.mw; m.r = P.r; m.Iw = 0.5*P.mw*P.r^2;
    crr   = [0.005 0.012 0.03];
    m.Frr = crr(P.surface)*(m.M + m.mw)*m.g;                    % rolling resistance
    m.m11 = m.M + m.mw + m.Iw/m.r^2;
    m.m22 = m.Ib + m.M*m.l^2;
    m.Ml  = m.M*m.l;
    m.kT  = 2*P.Ts/P.Vn;                                        % both motors, N·m per V
    m.kE  = P.Vn/P.w0;                                          % V per rad/s
    m.Veff = max(P.Vbat - P.drop, 0);                           % L298N drop
end

function sim = simInit(P)
    sim.m    = buildModel(P);
    sim.Tc   = 1/P.f;
    sim.nsub = max(1, round(sim.Tc/1e-3));
    sim.dt   = sim.Tc/sim.nsub;
    if isinf(P.T), sim.N = 60*P.f; else, sim.N = max(2, ceil(P.T*P.f) + 2); end
    % L columns: 1 t | 2 th | 3 est | 4 thd | 5 x | 6 xd | 7 u | 8 P | 9 I | 10 D
    %            11 ref | 12 acc | 13 gyro | 14 F | 15 tau | 16 V | 17 Kp | 18 Ki | 19 Kd
    sim.L    = nan(sim.N, 19);
    sim.q    = [0; 0; deg2rad(P.th0); 0];                       % x, xdot, theta, thetadot
    sim.dq   = zeros(4,1);
    sim.est  = P.th0 + P.mountOff;
    sim.I    = 0;  sim.ePrev = NaN;
    sim.k    = 0;  sim.t = 0;
    sim.fallen = false; sim.tFall = NaN; sim.lying = false; sim.done = false;
    sim.livePushT = -Inf; sim.livePushF = 0; sim.Fnow = 0;
end

function sim = simStep(sim, P)
    % One control period: sense -> filter -> PID -> driver -> physics.
    m = sim.m; Tc = sim.Tc; q = sim.q; t = sim.t;
    th = q(3); thd = q(4); a = sim.dq;
    % ---- IMU (at CoM height) ----
    ax = a(2) + m.l*cos(th)*a(4) - m.l*sin(th)*thd^2;
    ay = -m.l*sin(th)*a(4) - m.l*cos(th)*thd^2;
    fu = ax*sin(th) + (ay + m.g)*cos(th);
    fe = ax*cos(th) - (ay + m.g)*sin(th);
    acc  = atan2d(-fe, fu) + P.mountOff + P.accNoise*randn;
    gyro = rad2deg(thd) + P.gyroBias + P.gyroNoise*randn;
    % ---- complementary filter ----
    sim.est = P.alpha*(sim.est + gyro*Tc) + (1 - P.alpha)*acc;
    % ---- outer speed/position loop (needs encoders) ----
    if P.mode == 2
        ref = -(P.Kv*q(2) + P.Kx*q(1));
        ref = min(max(ref, -P.refLim), P.refLim);
    else
        ref = 0;
    end
    % ---- PID on tilt ----
    e = sim.est - P.trim - ref;
    sim.I = sim.I + e*Tc;
    if P.Ki > 0
        Imax = P.iLim/P.Ki;
        sim.I = min(max(sim.I, -Imax), Imax);
    end
    if P.dsrc == 1
        D = gyro;
    elseif isnan(sim.ePrev)
        D = 0;
    else
        D = (e - sim.ePrev)/Tc;
    end
    sim.ePrev = e;
    uP = P.Kp*e; uI = P.Ki*sim.I; uD = P.Kd*D;
    u = min(max(uP + uI + uD, -255), 255);
    % ---- fall detection: firmware cuts the motors ----
    if ~sim.fallen && abs(rad2deg(th)) > P.fallAngle
        sim.fallen = true; sim.tFall = t;
    end
    if sim.fallen, u = 0; sim.I = 0; end
    % ---- motor driver: deadband (+ optional compensation), L298N ----
    pwm = u;
    if P.comp && abs(u) > 0.5
        pwm = sign(u)*(P.db + abs(u)*(255 - P.db)/255);
    end
    if abs(pwm) < P.db, V = 0; else, V = m.Veff*pwm/255; end
    % ---- disturbances ----
    F = 0;
    if P.Fpush ~= 0 && t >= P.tPush && t < P.tPush + P.pushDur, F = F + P.Fpush; end
    if t >= sim.livePushT && t < sim.livePushT + P.pushDur, F = F + sim.livePushF; end
    if P.rough > 0, F = F + P.rough*randn; end
    % ---- physics (RK4 sub-steps) ----
    tau = 0;
    if ~sim.lying
        dt = sim.dt;
        for i = 1:sim.nsub
            if sim.fallen
                tau = 0;
            else
                tau = m.kT*(V - m.kE*(q(2)/m.r - q(4)));        % back-EMF on relative speed
            end
            k1 = wipDeriv(q, tau, F, m);
            k2 = wipDeriv(q + 0.5*dt*k1, tau, F, m);
            k3 = wipDeriv(q + 0.5*dt*k2, tau, F, m);
            k4 = wipDeriv(q + dt*k3, tau, F, m);
            q = q + dt/6*(k1 + 2*k2 + 2*k3 + k4);
        end
        sim.dq = wipDeriv(q, tau, F, m);
        if abs(q(3)) >= deg2rad(85)                             % lying on the floor
            q(3) = sign(q(3))*deg2rad(85); q(2) = 0; q(4) = 0;
            sim.lying = true; sim.dq = zeros(4,1);
        end
    end
    sim.q = q;
    sim.k = sim.k + 1; sim.t = sim.t + Tc; sim.Fnow = F;
    if sim.k > size(sim.L,1), sim.L = [sim.L; nan(size(sim.L,1), 19)]; end   % grow log
    sim.L(sim.k,:) = [sim.t, rad2deg(q(3)), sim.est, rad2deg(q(4)), q(1), q(2), u, ...
        uP, uI, uD, ref, acc, gyro, F, tau, V, P.Kp, P.Ki, P.Kd];
    if sim.t >= P.T - 1e-9 || (sim.lying && sim.t > sim.tFall + 0.6)
        sim.done = true;
    end
end

function dq = wipDeriv(q, tau, F, m)
    % Wheeled inverted pendulum. q = [x; xdot; theta; thetadot], theta > 0 = leaning forward.
    xd = q(2); th = q(3); thd = q(4);
    c = cos(th); s = sin(th);
    m12 = m.Ml*c;
    f1 = m.Ml*s*thd^2 + tau/m.r + F - m.Frr*tanh(xd/0.01);
    f2 = m.Ml*m.g*s - tau + F*m.l*c;
    dn = m.m11*m.m22 - m12^2;
    dq = [xd; (f1*m.m22 - m12*f2)/dn; thd; (m.m11*f2 - m12*f1)/dn];
end

function sim = simRebuild(sim, P)
    % Apply changed physical parameters / loop rate mid-run, keeping the state.
    sim.m    = buildModel(P);
    sim.Tc   = 1/P.f;
    sim.nsub = max(1, round(sim.Tc/1e-3));
    sim.dt   = sim.Tc/sim.nsub;
end

function [phys, other] = paramDiff(A, B)
    pk = {'Mb','pay','l','h','r','Vbat','db','Ts','w0','surface','f'};
    phys = false;
    for i = 1:numel(pk)
        if A.(pk{i}) ~= B.(pk{i}), phys = true; break; end
    end
    ok = {'Kp','Ki','Kd','Kv','Kx','alpha','trim','mode','dsrc','comp','Fpush','tPush', ...
          'pushDur','gyroNoise','accNoise','gyroBias','mountOff','rough','T'};
    other = false;
    for i = 1:numel(ok)
        if A.(ok{i}) ~= B.(ok{i}), other = true; break; end
    end
end

function [lg, met, sim] = runHeadless(P)
    if isinf(P.T), P.T = 60; end
    rng(P.seed);
    sim = simInit(P);
    while ~sim.done
        sim = simStep(sim, P);
    end
    lg  = simFinish(sim, P);
    met = computeMetrics(lg, P);
end

function lg = simFinish(sim, P)
    Lm = sim.L(1:sim.k, :);
    names = {'t','th','est','thd','x','xd','u','Pt','It','Dt','ref','acc','gyro', ...
             'F','tau','V','Kp','Ki','Kd'};
    for i = 1:numel(names), lg.(names{i}) = Lm(:,i); end
    lg.fallen = sim.fallen; lg.tFall = sim.tFall; lg.Tc = sim.Tc; lg.th0 = P.th0;
end

function met = computeMetrics(lg, P)
    t = lg.t; th = lg.th;
    met.fell = lg.fallen; met.tFall = lg.tFall; met.duration = t(end);
    met.maxTilt = max(abs(th));
    i0 = t >= min(1, t(end)/2);
    met.rmsTilt = sqrt(mean(th(i0).^2));
    hasPush = P.Fpush ~= 0 && P.tPush < t(end);
    tEnd1 = t(end); if hasPush, tEnd1 = P.tPush; end
    met.settle = settleTime(t, th, 0, tEnd1, 1.0);
    if hasPush
        met.recovery = settleTime(t, th, P.tPush, t(end), 1.0);
        met.pushTilt = max(abs(th(t >= P.tPush)));
    else
        met.recovery = NaN; met.pushTilt = NaN;
    end
    met.drift    = 100*lg.x(end);
    met.maxDrift = 100*max(abs(lg.x));
    met.maxSpeed = 100*max(abs(lg.xd));
    met.peakPWM  = max(abs(lg.u));
    met.satPct   = 100*mean(abs(lg.u) >= 254.5);
    met.rmsPWM   = sqrt(mean(lg.u.^2));
    met.estErr   = sqrt(mean((lg.est - lg.th).^2));
    if met.fell
        met.verdict = 'FAIL'; met.reason = sprintf('fell over at %.2f s', met.tFall);
    elseif met.maxDrift > 50
        met.verdict = 'WARN'; met.reason = sprintf('stayed up but drifted %.0f cm (no position hold)', met.maxDrift);
    elseif hasPush && isnan(met.recovery)
        met.verdict = 'WARN'; met.reason = 'did not settle within 1° after the push';
    elseif isnan(met.settle)
        met.verdict = 'WARN'; met.reason = 'still oscillating more than 1°';
    else
        met.verdict = 'PASS';
        if hasPush
            met.reason = sprintf('balanced and recovered from a %.1f N push', P.Fpush);
        else
            met.reason = 'balanced and settled';
        end
    end
    met.cost = costFn(met, P, hasPush);
end

function ts = settleTime(t, th, t0, t1, tol)
    % Time after t0 from which |th| stays below tol until t1 (NaN if never).
    idx = t >= t0 & t <= t1;
    tt = t(idx); a = abs(th(idx));
    if isempty(a), ts = NaN; return; end
    last = find(a > tol, 1, 'last');
    if isempty(last), ts = 0;
    elseif last == numel(a), ts = NaN;
    else, ts = tt(last+1) - t0;
    end
end

function J = costFn(met, P, hasPush)
    % Scalar cost for tuning (lower = better). Falling dominates everything.
    if met.fell
        J = 100 + 20*max(0, P.T - met.tFall);
        return
    end
    rec = 0;
    if hasPush
        rec = met.recovery; if isnan(rec), rec = 3; end
    end
    J = met.rmsTilt + 0.2*met.maxTilt + 0.05*met.maxDrift + 0.004*met.rmsPWM ...
        + 0.5*rec + 0.02*met.satPct;
end

function J = evalPair(P, seed)
    % Two episodes (push forward + backward) with common random numbers.
    P.seed = seed;        [~, m1] = runHeadless(P);
    P.Fpush = -P.Fpush; P.seed = seed + 500; [~, m2] = runHeadless(P);
    J = 0.5*(m1.cost + m2.cost);
end

function P = applyX(P, names, x)
    for i = 1:numel(names), P.(names{i}) = x(i); end
end

function v = uSym(rs, a)
    v = (2*rand(rs) - 1)*a;
end

% ------------------------------- showcase -------------------------------
function makeShowcase(outDir)
    % Runs a fixed set of experiments headless and writes CSV logs, a
    % summary table and dark-themed PNG figures into outDir.
    if ~exist(outDir, 'dir'), mkdir(outDir); end
    isOct = exist('OCTAVE_VERSION', 'builtin') > 0;
    if isOct, vis = 'on'; else, vis = 'off'; end
    D = defaultParams();

    % ---- 1. scenario runs (10 s, start tilt 5 deg, 1.5 N push at 3 s) ----
    sc = {};
    sc(end+1,:) = {'cascaded_default', 'Angle PID + speed/position loop (tuned)', D};
    P = D; P.mode = 1;
    sc(end+1,:) = {'angle_only_tuned', 'Angle PID only, same gains', P};
    P = D; P.mode = 1; P.Kp = 18; P.Ki = 140; P.Kd = 0.6; P.dsrc = 2;
    sc(end+1,:) = {'firmware_config_defaults', 'Firmware Config.h gains (Kp 18, Ki 140, Kd 0.6)', P};
    P = D; P.mountOff = 2;
    sc(end+1,:) = {'imu_mount_offset_2deg', 'Cascaded, IMU mounted 2 deg off level', P};
    P = D; P.pay = 0.2;
    sc(end+1,:) = {'payload_200g', 'Cascaded, 200 g payload on top plate', P};
    n = size(sc, 1);
    L = cell(n, 1); M = cell(n, 1);
    fid = fopen(fullfile(outDir, 'summary.csv'), 'w');
    fprintf(fid, ['run,description,mode,Kp,Ki,Kd,Kv,Kx,verdict,max_tilt_deg,rms_tilt_deg,' ...
        'settle_s,push_recovery_s,final_drift_cm,max_drift_cm,peak_pwm,pwm_saturated_pct,reason\n']);
    for i = 1:n
        P = sc{i,3};
        [L{i}, M{i}] = runHeadless(P);
        writeLogCsv(fullfile(outDir, ['run_' sc{i,1} '.csv']), L{i});
        m = M{i};
        fprintf(fid, '%s,"%s",%d,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.0f,%.2f,"%s"\n', ...
            sc{i,1}, sc{i,2}, P.mode, P.Kp, P.Ki, P.Kd, P.Kv, P.Kx, m.verdict, m.maxTilt, ...
            m.rmsTilt, m.settle, m.recovery, m.drift, m.maxDrift, m.peakPWM, m.satPct, m.reason);
        fprintf('%-26s %-4s max %6.2f  rms %5.2f  recov %5.2f  drift %7.1f cm  (%s)\n', ...
            sc{i,1}, m.verdict, m.maxTilt, m.rmsTilt, m.recovery, m.maxDrift, m.reason);
    end
    fclose(fid);

    % ---- figure 1: full results of the default run ----
    lg = L{1};
    f = darkFig(1400, 900, vis);
    ax = subplot(3,2,1); hold(ax,'on');
    plot(ax, lg.t, lg.th, 'Color', [0.30 0.75 1.00], 'LineWidth', 1.4);
    plot(ax, lg.t, lg.est, 'Color', [1.00 0.62 0.20]);
    plot(ax, lg.t, lg.ref, '--', 'Color', [0.75 0.75 0.78]);
    sax(ax, 'Pitch: true vs estimate (deg)', '', 'deg'); dlegend(ax, {'true','estimate','reference'});
    ax = subplot(3,2,2); hold(ax,'on');
    plot(ax, lg.t, lg.Pt, 'Color', [0.30 0.75 1.00]); plot(ax, lg.t, lg.It, 'Color', [1.00 0.62 0.20]);
    plot(ax, lg.t, lg.Dt, 'Color', [0.40 0.90 0.50]); plot(ax, lg.t, lg.u, 'w', 'LineWidth', 1.1);
    sax(ax, 'PID terms (PWM)', '', 'PWM'); dlegend(ax, {'P','I','D','output'});
    ax = subplot(3,2,3); hold(ax,'on');
    plot(ax, lg.t, 100*lg.x, 'Color', [0.40 0.90 0.50], 'LineWidth', 1.4);
    plot(ax, lg.t, 100*lg.xd, 'Color', [1.00 0.85 0.30]);
    sax(ax, 'Position (cm) and speed (cm/s)', '', ''); dlegend(ax, {'position','speed'});
    ax = subplot(3,2,4); hold(ax,'on');
    gInt = lg.est(1) + cumsum(lg.gyro)*lg.Tc;
    plot(ax, lg.t, lg.acc, 'Color', [0.55 0.27 0.36]);
    plot(ax, lg.t, gInt, '--', 'Color', [1.00 0.85 0.30]);
    plot(ax, lg.t, lg.est, 'Color', [1.00 0.62 0.20], 'LineWidth', 1.2);
    plot(ax, lg.t, lg.th, 'Color', [0.30 0.75 1.00]);
    ylim(ax, [min(lg.th)-3, max(lg.th)+3]);
    sax(ax, 'Sensor fusion (deg)', '', 'deg'); dlegend(ax, {'accel only','gyro only','filtered','true'});
    ax = subplot(3,2,5); hold(ax,'on');
    plot(ax, lg.th, lg.thd, 'Color', [0.30 0.75 1.00]);
    plot(ax, lg.th(1), lg.thd(1), 'o', 'MarkerFaceColor', [0.4 0.9 0.5], 'Color', [0.4 0.9 0.5]);
    sax(ax, 'Phase portrait', 'pitch (deg)', 'pitch rate (deg/s)');
    ax = subplot(3,2,6); hold(ax,'on');
    plot(ax, lg.t, lg.V, 'Color', [0.80 0.45 1.00]); plot(ax, lg.t, 100*lg.tau, 'Color', [0.40 0.90 0.50]);
    sax(ax, 'Motor voltage (V) and wheel torque (N cm)', 'time (s)', ''); dlegend(ax, {'voltage','torque'});
    for k = [1 2 3 4 6], xlim(subplot(3,2,k), [0 lg.t(end)]); end
    savePng(f, fullfile(outDir, 'fig1_default_run_results.png'));

    % ---- figure 2: controller comparison ----
    f = darkFig(1400, 820, vis);
    cols = [0.30 0.75 1.00; 1.00 0.62 0.20; 0.95 0.40 0.60];
    labels = {'angle + speed loop', 'angle PID only', 'firmware Config.h gains'};
    ttls = {'Pitch (deg)', 'Position (cm)', 'Motor command (PWM)'};
    for k = 1:3
        ax = subplot(3,1,k); hold(ax,'on');
        for i = 1:3
            lg = L{i};
            switch k
                case 1, y = lg.th; case 2, y = 100*lg.x; otherwise, y = lg.u;
            end
            plot(ax, lg.t, y, 'Color', cols(i,:), 'LineWidth', 1.2);
        end
        if k == 1, ylim(ax, [-50 50]); end
        xlim(ax, [0 D.T]);
        if k == 3, xl = 'time (s)'; else, xl = ''; end
        sax(ax, ttls{k}, xl, '');
        if k == 1, dlegend(ax, labels); end
    end
    savePng(f, fullfile(outDir, 'fig2_controller_comparison.png'));

    % ---- figure 3: Kp x Kd stability map ----
    Pt = D; Pt.T = 6; Pt.tPush = 2.5;
    xv = linspace(20, 260, 13); yv = linspace(0, 4, 11);
    Z = nan(numel(yv), numel(xv));
    for i = 1:numel(xv)
        for j = 1:numel(yv)
            Pc = Pt; Pc.Kp = xv(i); Pc.Kd = yv(j);
            [~, m] = runHeadless(Pc);
            if ~m.fell, Z(j,i) = m.rmsTilt; end
        end
        fprintf('stability map column %d/%d\n', i, numel(xv));
    end
    writeMatrixCsv(fullfile(outDir, 'stability_map_kp_kd.csv'), xv, yv, Z, 'Kd_vs_Kp');
    f = darkFig(1000, 700, vis);
    ax = axes('Parent', f); hold(ax,'on');
    ok = ~isnan(Z);
    lo = min(Z(ok)); hi = max(Z(ok)); stp = max((hi-lo)/64, 1e-3);
    Zp = Z; Zp(~ok) = lo - stp;
    anc = [0.27 0.00 0.33; 0.23 0.32 0.55; 0.13 0.57 0.55; 0.37 0.79 0.38; 0.99 0.91 0.14];
    cm = interp1(linspace(0,1,5), anc, linspace(0,1,64));
    imagesc(ax, xv, yv, Zp); colormap(ax, [0.05 0.05 0.06; cm]); caxis(ax, [lo-stp hi]);
    cb = colorbar(ax); set(cb, 'Color', [0.85 0.86 0.9]);
    ylabel(cb, 'RMS tilt (deg)', 'Color', [0.85 0.86 0.9]);
    plot(ax, D.Kp, D.Kd, 'w+', 'MarkerSize', 14, 'LineWidth', 2);
    axis(ax, [xv(1)-10 xv(end)+10 yv(1)-0.2 yv(end)+0.2]); set(ax, 'YDir', 'normal');
    sax(ax, sprintf('Stability map Kp x Kd (angle + speed loop): black = fell, + = default. %d of %d balanced', ...
        sum(ok(:)), numel(Z)), 'Kp (PWM/deg)', 'Kd (PWM s/deg)');
    savePng(f, fullfile(outDir, 'fig3_stability_map_kp_kd.png'));

    % ---- figure 4: push recovery sweep, both modes ----
    forces = 0.5:0.5:6; MT = nan(2, numel(forces)); RT = MT; FELL = false(size(MT));
    for mi = 1:2
        for k = 1:numel(forces)
            Pc = D; Pc.mode = mi; Pc.T = 7; Pc.tPush = 2.5; Pc.Fpush = forces(k);
            [~, m] = runHeadless(Pc);
            FELL(mi,k) = m.fell;
            if ~m.fell, MT(mi,k) = m.pushTilt; RT(mi,k) = m.recovery; end
        end
    end
    fid = fopen(fullfile(outDir, 'push_recovery_sweep.csv'), 'w');
    fprintf(fid, 'push_N,angle_only_max_tilt_deg,cascaded_max_tilt_deg,angle_only_recovery_s,cascaded_recovery_s\n');
    for k = 1:numel(forces)
        fprintf(fid, '%.1f,%.3f,%.3f,%.3f,%.3f\n', forces(k), MT(1,k), MT(2,k), RT(1,k), RT(2,k));
    end
    fclose(fid);
    f = darkFig(1200, 520, vis);
    names = {'angle PID only', 'angle + speed loop'}; cc = [1.00 0.62 0.20; 0.30 0.75 1.00];
    for sp = 1:2
        ax = subplot(1,2,sp); hold(ax,'on');
        if sp == 1, Y = MT; else, Y = RT; end
        for mi = 1:2
            plot(ax, forces, Y(mi,:), '-o', 'Color', cc(mi,:), 'MarkerFaceColor', cc(mi,:), 'LineWidth', 1.4);
        end
        for mi = 1:2                                   % falls: x markers on the axis
            ff = forces(FELL(mi,:));
            yy = (0.04 + 0.06*(mi-1))*max([Y(~isnan(Y)); 1])*ones(size(ff));
            plot(ax, ff, yy, 'x', 'Color', cc(mi,:), 'MarkerSize', 10, 'LineWidth', 2);
        end
        xlim(ax, [0 6.5]); ylim(ax, [0 1.15*max([Y(~isnan(Y)); 1])]);
        if sp == 1
            sax(ax, 'Max tilt after push  (x = fell)', 'push force (N, 0.1 s)', 'deg');
            dlegend(ax, names);
        else
            sax(ax, 'Recovery time to within 1 deg  (x = fell)', 'push force (N, 0.1 s)', 's');
        end
    end
    savePng(f, fullfile(outDir, 'fig4_push_recovery_sweep.png'));

    % ---- figure 5: live physical edit (+5 cm body height at t = 4 s) ----
    P = D; rng(P.seed); sim = simInit(P); hTrace = []; lTrace = []; edited = false;
    while ~sim.done
        if ~edited && sim.t >= 4
            P.h = P.h + 0.05; P.l = D.l*P.h/D.h;     % CoM moves with the body
            sim = simRebuild(sim, P); edited = true;
        end
        sim = simStep(sim, P);
        hTrace(end+1) = P.h; lTrace(end+1) = P.l; %#ok<AGROW>
    end
    lg = simFinish(sim, P);
    writeLogCsv(fullfile(outDir, 'run_live_height_edit.csv'), lg);
    f = darkFig(1200, 760, vis);
    ax = subplot(3,1,1); hold(ax,'on');
    plot(ax, lg.t, lg.th, 'Color', [0.30 0.75 1.00], 'LineWidth', 1.3);
    plot(ax, [4 4], [-6 6], '-', 'Color', [1 0.35 0.35], 'LineWidth', 1.5);
    sax(ax, 'Pitch (deg): body raised by 5 cm at t = 4 s (red line) while balancing', '', 'deg');
    ax = subplot(3,1,2); hold(ax,'on');
    plot(ax, lg.t, 100*lg.x, 'Color', [0.40 0.90 0.50], 'LineWidth', 1.3);
    plot(ax, [4 4], [-10 40], '-', 'Color', [1 0.35 0.35], 'LineWidth', 1.5);
    sax(ax, 'Position (cm)', '', 'cm');
    ax = subplot(3,1,3); hold(ax,'on');
    plot(ax, lg.t, 100*hTrace(:), 'Color', [1.00 0.85 0.30], 'LineWidth', 1.6);
    plot(ax, lg.t, 100*lTrace(:), 'Color', [0.95 0.40 0.60], 'LineWidth', 1.6);
    ylim(ax, [0 25]);
    sax(ax, 'Body height and CoM height (cm)', 'time (s)', 'cm'); dlegend(ax, {'body height','CoM height'});
    savePng(f, fullfile(outDir, 'fig5_live_height_edit.png'));
    fprintf('Showcase written to %s\n', outDir);
end

function f = darkFig(w, h, vis)
    f = figure('Color', [0.09 0.10 0.12], 'Position', [40 40 w h], 'Visible', vis, ...
        'InvertHardcopy', 'off');
end

function sax(ax, t, xl, yl)
    set(ax, 'Color', [0.11 0.12 0.15], 'XColor', [0.78 0.80 0.84], 'YColor', [0.78 0.80 0.84], ...
        'FontSize', 9, 'Box', 'on');
    try, set(ax, 'GridColor', [0.55 0.57 0.62], 'GridAlpha', 0.3); catch, end
    grid(ax, 'on');
    title(ax, t, 'Color', [0.92 0.93 0.96], 'FontWeight', 'normal');
    xlabel(ax, xl); ylabel(ax, yl);
end

function dlegend(ax, labels)
    lgd = legend(ax, labels, 'Location', 'northeast');
    try, set(lgd, 'TextColor', [0.9 0.9 0.92], 'Color', [0.16 0.17 0.21], 'EdgeColor', [0.35 0.36 0.40]); catch, end
end

function savePng(f, file)
    drawnow;
    print(f, '-dpng', '-r110', file);
    close(f);
end

function writeLogCsv(file, lg)
    names = {'t','th','est','thd','x','xd','u','Pt','It','Dt','ref','acc','gyro','F','tau','V','Kp','Ki','Kd'};
    hdr = {'t_s','pitch_deg','pitch_est_deg','pitch_rate_dps','x_m','v_mps','pwm','P_term','I_term', ...
           'D_term','ref_deg','accel_angle_deg','gyro_dps','push_N','wheel_torque_Nm','motor_V','Kp','Ki','Kd'};
    A = zeros(numel(lg.t), numel(names));
    for i = 1:numel(names), A(:,i) = lg.(names{i}); end
    fid = fopen(file, 'w');
    fprintf(fid, '%s\n', strjoin(hdr, ','));
    fprintf(fid, [repmat('%.5g,', 1, numel(names)-1) '%.5g\n'], A');
    fclose(fid);
end

function writeMatrixCsv(file, xv, yv, Z, corner)
    fid = fopen(file, 'w');
    fprintf(fid, '%s', corner); fprintf(fid, ',%.4g', xv); fprintf(fid, '\n');
    for j = 1:numel(yv)
        fprintf(fid, '%.4g', yv(j)); fprintf(fid, ',%.4g', Z(j,:)); fprintf(fid, '\n');
    end
    fclose(fid);
end

% --------------------------------- text ---------------------------------
function s = summaryLines(R)
    P = R.P; m = R.met;
    modes = {'Angle PID only (firmware)','Angle PID + speed/position loop'};
    dsrc = {'gyro','error diff'}; surf = {'tile','wood','carpet'};
    s = {sprintf('RUN #%d  %s  -  %s simulated', R.id, upper(R.status), fmtClock(m.duration))
         sprintf('Setup     %s | %d Hz | alpha %.3f | D from %s', modes{P.mode}, P.f, P.alpha, dsrc{P.dsrc})
         sprintf('Gains     Kp %.1f  Ki %.2f  Kd %.2f  |  Kv %.1f  Kx %.1f', P.Kp, P.Ki, P.Kd, P.Kv, P.Kx)
         sprintf('Robot     %.2f+%.2f kg | CoM %.0f mm | wheel r %.1f mm | %.1f V | %s', ...
                 P.Mb, P.pay, 1000*P.l, 1000*P.r, P.Vbat, surf{P.surface})
         sprintf('Scenario  start %+.1f° | push %+.1f N @ %.1f s | gyro %.2f°/s bias %+.2f | mount %+.1f°', ...
                 P.th0, P.Fpush, P.tPush, P.gyroNoise, P.gyroBias, P.mountOff)
         sprintf('Tilt      max %.2f°   RMS %.2f°   settle %s   push recovery %s', ...
                 m.maxTilt, m.rmsTilt, fmtT(m.settle), fmtT(m.recovery))
         sprintf('Motion    final %+.1f cm   max drift %.1f cm   max speed %.1f cm/s', ...
                 m.drift, m.maxDrift, m.maxSpeed)
         sprintf('Motors    peak PWM %.0f   saturated %.1f %%   RMS PWM %.0f', m.peakPWM, m.satPct, m.rmsPWM)
         sprintf('Estimate  RMS error (estimate - true) %.2f°', m.estErr)
         sprintf('Verdict   %s - %s', m.verdict, m.reason)};
end

function s = fmtClock(t)
    % 75.3 -> '1:15.3'  ;  12.04 -> '12.0 s'
    if t < 60, s = sprintf('%.1f s', t);
    else, s = sprintf('%d:%04.1f', floor(t/60), mod(t,60)); end
end

function s = fmtT(v)
    if isnan(v), s = 'n/a'; else, s = sprintf('%.2f s', v); end
end

% ------------------------------- graphics -------------------------------
function g = darkGrid(parent, sz, col)
    g = uigridlayout(parent, sz);
    g.Padding = [6 6 6 6]; g.RowSpacing = 6; g.ColumnSpacing = 6;
    g.BackgroundColor = col;
end

function styleAx(ax, t, xl, yl)
    ax.Color = [0.11 0.12 0.15];
    ax.XColor = [0.75 0.77 0.80]; ax.YColor = [0.75 0.77 0.80];
    ax.GridColor = [0.55 0.57 0.62]; ax.GridAlpha = 0.3;
    ax.XGrid = 'on'; ax.YGrid = 'on'; ax.Box = 'on'; ax.FontSize = 9;
    try, ax.BackgroundColor = [0.16 0.17 0.20]; catch, end
    ttl(ax, t);
    xlabel(ax, xl); ylabel(ax, yl);
end

function ttl(ax, t)
    title(ax, t, 'Color', [0.90 0.91 0.94], 'FontWeight', 'normal', 'FontSize', 10);
end

function legendDark(ax, h, labels)
    lgd = legend(ax, h, labels, 'TextColor', [0.9 0.9 0.92], 'Color', [0.16 0.17 0.21], ...
        'EdgeColor', [0.35 0.36 0.40], 'FontSize', 8, 'Location', 'northeast');
    lgd.AutoUpdate = 'off';
end

function [V, F] = boxMesh(c, s)
    V = [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1].*(s/2) + c;
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
end

function G = robotGeometry(P)
    % Body frame: x forward, y along the axle, z up from the axle centre.
    h = P.h; zb = 0.022; zm = 0.55*h; zt = h;
    acr = [0.82 0.86 0.92]; brass = [0.80 0.65 0.30];
    spec = { ...
        [0 0 zb],  [0.07 0.09 0.004], acr;              % bottom plate
        [0 0 zm],  [0.07 0.09 0.004], acr;              % middle plate
        [0 0 zt],  [0.07 0.09 0.004], acr;              % top plate
        [ 0.03  0.04 (zb+zt)/2], [0.004 0.004 zt-zb], brass;
        [ 0.03 -0.04 (zb+zt)/2], [0.004 0.004 zt-zb], brass;
        [-0.03  0.04 (zb+zt)/2], [0.004 0.004 zt-zb], brass;
        [-0.03 -0.04 (zb+zt)/2], [0.004 0.004 zt-zb], brass;
        [0  0.035 0], [0.065 0.022 0.019], [0.95 0.80 0.15];   % BO motor
        [0 -0.035 0], [0.065 0.022 0.019], [0.95 0.80 0.15];   % BO motor
        [0 0 zb+0.004], [0.043 0.043 0.004], [0.80 0.15 0.15];  % L298N board
        [0 0 zb+0.016], [0.023 0.016 0.020], [0.10 0.10 0.10];  % heatsink
        [0 0 zm+0.013], [0.042 0.078 0.022], [0.15 0.30 0.75];  % 2x18650 holder
        [0 0.005 zt+0.005], [0.028 0.052 0.006], [0.10 0.33 0.30]; % ESP32 devkit
        [0 0.020 zt+0.009], [0.018 0.016 0.003], [0.75 0.75 0.78]; % ESP32 shield can
        [0 -0.034 zt+0.004], [0.016 0.020 0.003], [0.45 0.20 0.70]}; % IMU board
    parts = struct('V',{},'F',{},'c',{});
    for i = 1:size(spec,1)
        [V, F] = boxMesh(spec{i,1}, spec{i,2});
        parts(i).V = V; parts(i).F = F; parts(i).c = spec{i,3};
    end
    G.parts = parts;
    % wheel: cylinder mesh around its own centre
    ww = 0.026; nS = 32; a = linspace(0, 2*pi, nS+1); a(end) = [];
    r = P.r;
    Vw = [r*sin(a'), -ww/2*ones(nS,1), r*cos(a'); r*sin(a'), ww/2*ones(nS,1), r*cos(a')];
    Fw = nan(nS+2, nS);
    for i = 1:nS
        j = mod(i, nS) + 1;
        Fw(i,1:4) = [i, j, nS+j, nS+i];
    end
    Fw(nS+1,:) = 1:nS;
    Fw(nS+2,:) = nS+1:2*nS;
    G.wheelV = Vw; G.wheelF = Fw; G.ww = ww;
    G.wheelY = [0.062 -0.062];
    G.lc = (P.Mb*P.l + P.pay*P.h)/(P.Mb + P.pay);
end

function [V, F, Cd] = floorMesh(surface)
    xs = -10:0.05:10; ys = -0.30:0.05:0.30;
    [X, Y] = meshgrid(xs, ys);
    V = [X(:), Y(:), zeros(numel(X),1)];
    ny = numel(ys); nx = numel(xs);
    nf = (nx-1)*(ny-1);
    F = zeros(nf, 4); Cd = zeros(nf, 3);
    pal = {[0.78 0.80 0.83; 0.68 0.70 0.74], ...
           [0.72 0.56 0.38; 0.62 0.47 0.30], ...
           [0.44 0.35 0.38; 0.38 0.30 0.33]};
    cc = pal{surface};
    n = 0;
    for i = 1:nx-1
        for j = 1:ny-1
            n = n + 1;
            a = (i-1)*ny + j;
            F(n,:) = [a, a+ny, a+ny+1, a+1];
            col = cc(1 + mod(i+j, 2), :);
            if mod(i-1, 10) == 0, col = col*0.75; end         % marker every 0.5 m
            Cd(n,:) = col;
        end
    end
end
