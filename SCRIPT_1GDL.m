%% Animazione 1GDL (gomito fisso) con retta di forza infinita (ellisse degenere)
% GUI avanzata stile 2GDL + freccia curva che mostra l'angolo articolare

clear; clc; close all

l1 = 0.3;   % Lunghezza omero [m] (spalla-gomito)
l2 = 0.25;  % Lunghezza avambraccio [m] (gomito-mano)
g  = 9.81;

filename = 'dati_marker_1.csv';
data = readtable(filename);

xS = data.x_spalla;    yS = data.y_spalla;
xG = data.x_gomito;    yG = data.y_gomito;
xH = data.x_mano;      yH = data.y_mano;
t  = data.time;

nframe = length(t);

row_names = { 'x_{spalla}','y_{spalla}','x_{gomito}','y_{gomito}', ...
  'x_{mano}','y_{mano}','tempo [s]','θ_2 [deg]','J_1','J_2','τ [Nm]'};
data_tab = cell(numel(row_names),2);
data_tab(:,1) = row_names;
data_tab(:,2) = {0};

fig = figure('Name','Animazione 1GDL Gomito fisso','Position',[200 20 1100 700]);
axAnim = axes('Parent',fig,'Position',[0.05 0.15 0.57 0.8]);
hold(axAnim,'on');
axis(axAnim,'equal');
grid(axAnim,'on');
xlabel(axAnim,'x [m]');
ylabel(axAnim,'y [m]');
title(axAnim,'Braccio planare 1 GDL (gomito fisso)');
xlim(axAnim,[-l2 l2]*1.2);
ylim(axAnim,[-0.05 l1+l2+0.1]);

panel = uipanel('Parent',fig,'Title','Dati e Controlli','FontSize',12,...
    'Position',[0.67 0.02 0.31 0.95]);
tTab = uitable(panel, ...
    'Data', data_tab, ...
    'ColumnName', {'Parametro','Valore'}, ...
    'ColumnWidth', {110 90}, ...
    'FontSize', 12, ...
    'RowName', [], ...
    'Position', [15 10 260 320], ...
    'ColumnEditable', [false false]);

btnPlay = uicontrol(panel,'Style','pushbutton','String','Play',...
    'Position',[30 350 70 38],'FontSize',12,'BackgroundColor',[0.6 1 0.6]);
btnStop = uicontrol(panel,'Style','pushbutton','String','Stop',...
    'Position',[115 350 70 38],'FontSize',12,'BackgroundColor',[1 0.6 0.6]);
btnRestart = uicontrol(panel,'Style','pushbutton','String','Restart',...
    'Position',[200 350 70 38],'FontSize',12,'BackgroundColor',[0.9 0.9 1]);

editTime = uicontrol(panel, 'Style', 'edit', 'String', '0', ...
    'Position', [30 410 60 30], 'FontSize', 12);

intervalText = uicontrol(panel, 'Style', 'text', ...
    'String', sprintf('Intervallo valido:\n%.2f - %.2f s', t(1), t(end)), ...
    'Position', [30 450 280 38], 'FontSize', 13, ...
    'HorizontalAlignment','left', ...
    'BackgroundColor', get(panel, 'BackgroundColor'));

btnGoToTime = uicontrol(panel, 'Style', 'pushbutton', 'String', 'Vai a tempo', ...
    'Position', [100 410 100 30], 'FontSize', 12, 'BackgroundColor', [0.9 0.9 0.7]);

% Variabili di controllo globali
k = 1;
isPlaying = false;
restartFlag = false;
goToFrame = nan;

btnPlay.Callback    = @(src, event) assignin('base','isPlaying',true);
btnStop.Callback    = @(src, event) assignin('base','isPlaying',false);
btnRestart.Callback = @(src, event) assignin('base','restartFlag',true);
btnGoToTime.Callback = @(src, event) goToTimeCallback(editTime, t(1), t(end), intervalText);

assignin('base','isPlaying',isPlaying);
assignin('base','restartFlag',restartFlag);
assignin('base','goToFrame',goToFrame);

while ishandle(fig)
    isPlaying   = evalin('base','isPlaying');
    restartFlag = evalin('base','restartFlag');
    goToFrame   = evalin('base','goToFrame');

    if ~isnan(goToFrame)
        goToFrame = max(min(goToFrame, t(end)), t(1));
        [~, k_new] = min(abs(t - goToFrame));
        k = k_new;
        assignin('base','goToFrame',nan);
        assignin('base','isPlaying',false);
        cla(axAnim);
        data_tab = draw_frame(xS, yS, xG, yG, xH, yH, l2, data_tab, tTab, axAnim, t, k);
    end

    if isPlaying && t(k) > t(end)
        assignin('base','isPlaying',false);
        continue
    end

    if restartFlag
        k = 1;
        assignin('base','restartFlag',false);
        assignin('base','isPlaying',false);
        cla(axAnim);
        data_tab = draw_frame(xS, yS, xG, yG, xH, yH, l2, data_tab, tTab, axAnim, t, k);
        continue
    end

    if isPlaying
        cla(axAnim);
        data_tab = draw_frame(xS, yS, xG, yG, xH, yH, l2, data_tab, tTab, axAnim, t, k);
        pause(0.07)
        if k < nframe
            k = k + 1;
        end
    else
        pause(0.03);
    end
end

function goToTimeCallback(editHandle, tmin, tmax, intervalText)
    userTime = str2double(get(editHandle, 'String'));
    if isnan(userTime) || userTime < tmin || userTime > tmax
        set(intervalText, 'String', sprintf('Valore non valido!\nUsa %.2f - %.2f s', tmin, tmax));
        set(intervalText, 'ForegroundColor', [1 0 0]);
        pause(1);
        set(intervalText, 'String', sprintf('Intervallo valido:\n%.2f - %.2f s', tmin, tmax));
        set(intervalText, 'ForegroundColor', [0 0 0]);
        return;
    end
    assignin('base','goToFrame', userTime);
    assignin('base','isPlaying', false);
end

function data_tab = draw_frame(xS, yS, xG, yG, xH, yH, l2, data_tab, tTab, axAnim, t, k)
    % Manipolatore
    plot(axAnim, [xS(k), xG(k)], [yS(k), yG(k)], 'k-', 'LineWidth', 4);
    plot(axAnim, [xG(k), xH(k)], [yG(k), yH(k)], 'k-', 'LineWidth', 4);
    plot(axAnim, xS(k), yS(k), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 10);
    plot(axAnim, xG(k), yG(k), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
    plot(axAnim, xH(k), yH(k), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 10);

    text(axAnim, xS(k), yS(k), '  Spalla', 'FontWeight', 'bold', 'FontSize', 10, 'VerticalAlignment', 'bottom');
    text(axAnim, xG(k), yG(k), '  Gomito', 'FontWeight', 'bold', 'FontSize', 10, 'VerticalAlignment', 'top');
    text(axAnim, xH(k), yH(k), '  Mano', 'FontWeight', 'bold', 'FontSize', 10, 'VerticalAlignment', 'bottom');

    % --- FRECCIA PER L'ANGOLO ARTICOLARE (θ2) ---
    R = 0.06; % raggio dell'arco
    N = 25;   % punti per l'arco
    theta_start = atan2(yS(k)-yG(k), xS(k)-xG(k)); % direzione spalla-gomito
    theta_end   = atan2(yH(k)-yG(k), xH(k)-xG(k)); % direzione mano-gomito

    % Gestione verso: arco sempre nel senso giusto
    if theta_end < theta_start
        theta_end = theta_end + 2*pi;
    end
    theta_arc = linspace(theta_start, theta_end, N);

    x_arc = xG(k) + R * cos(theta_arc);
    y_arc = yG(k) + R * sin(theta_arc);
    plot(axAnim, x_arc, y_arc, 'r', 'LineWidth', 2); % arco

    % Freccia sulla punta dell'arco
    arrow_length = 0.015;
    arrow_angle = atan2(y_arc(end)-y_arc(end-1), x_arc(end)-x_arc(end-1));
    alpha = pi/7; % apertura freccia
    x_arrow1 = x_arc(end) - arrow_length * cos(arrow_angle - alpha);
    y_arrow1 = y_arc(end) - arrow_length * sin(arrow_angle - alpha);
    x_arrow2 = x_arc(end) - arrow_length * cos(arrow_angle + alpha);
    y_arrow2 = y_arc(end) - arrow_length * sin(arrow_angle + alpha);
    plot(axAnim, [x_arc(end) x_arrow1], [y_arc(end) y_arrow1], 'r', 'LineWidth', 2);
    plot(axAnim, [x_arc(end) x_arrow2], [y_arc(end) y_arrow2], 'r', 'LineWidth', 2);

    % Testo con il simbolo dell’angolo
    mid_idx = round(N*0.6);
    text(axAnim, x_arc(mid_idx), y_arc(mid_idx), '\theta_2', ...
        'FontWeight','bold','FontSize',14,'Color','r', 'HorizontalAlignment','center');

    % Cinematica/dinamica: θ2 = angolo gomito-mano
    theta2 = atan2(yH(k)-yG(k), xH(k)-xG(k));
    J = [-l2*sin(theta2); l2*cos(theta2)];
    dirJ = J / norm(J);

    % Retta di forza infinita: attraversa tutta la finestra
    xl = xlim(axAnim);
    yl = ylim(axAnim);
    m = dirJ(2)/dirJ(1);
    yx1 = yH(k) + m*(xl(1)-xH(k));
    yx2 = yH(k) + m*(xl(2)-xH(k));
    if abs(dirJ(2)) > 1e-12
        mx = dirJ(1)/dirJ(2);
        xx1 = xH(k) + mx*(yl(1)-yH(k));
        xx2 = xH(k) + mx*(yl(2)-yH(k));
    else
        xx1 = xH(k); xx2 = xH(k);
    end
    pts = [xl(1) yx1; xl(2) yx2; xx1 yl(1); xx2 yl(2)];
    inx = pts(:,1)>=xl(1)-1e-6 & pts(:,1)<=xl(2)+1e-6 & pts(:,2)>=yl(1)-1e-6 & pts(:,2)<=yl(2)+1e-6;
    pts = pts(inx,:);
    if size(pts,1)>=2
        plot(axAnim, pts(:,1), pts(:,2), 'm-', 'LineWidth', 2);
    end

    % Cinematica/dinamica per tabella
    F = [0; -20*9.81];
    tau = J(1)*F(1) + J(2)*F(2);

    angolo_gradi = rad2deg(theta2);
    txt = sprintf('\\theta_2 = %.1f^\\circ', angolo_gradi);
    text(axAnim, xG(k)+0.05, yG(k)+0.05, txt, 'FontWeight', 'bold', 'FontSize', 13, 'Color', 'b');

    % Tabella aggiornata
    valori = [xS(k); yS(k); xG(k); yG(k); xH(k); yH(k); t(k); angolo_gradi; J(1); J(2); tau];
    for ii = 1:numel(valori)
        if ii<=7
            data_tab{ii,2} = sprintf('%.4f', valori(ii));
        else
            data_tab{ii,2} = sprintf('%.2f', valori(ii));
        end
    end
    set(tTab, 'Data', data_tab);
end
