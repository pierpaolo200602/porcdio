%% braccio 2 GDL 

clear; clc; close all

l1 = 0.3; l2 = 0.25; g = 9.81;
F = [0; -20*g];

data = readtable('dati_marker_2.csv');
nframe = height(data);
x1 = data.x1; y1 = data.y1;
x2 = data.x2; y2 = data.y2;
x3 = data.x3; y3 = data.y3;
t = data.time;

x1 = x1 - 0.7;
x2 = x2 - 0.7;
x3 = x3 - 0.7;

row_names = {'J11','J12','J21','J22',...
    'x_{spalla}','y_{spalla}','x_{gomito}','y_{gomito}','x_{mano}','y_{mano}',...
    'tempo [s]','θ₁ [deg]','θ₂ [deg]','τ₁ [Nm]','τ₂ [Nm]'};
data_tab = cell(15,2);
data_tab(:,1) = row_names;
data_tab(:,2) = {0};

fig = figure('Name','Animazione 2 GDL integrata','Position',[200 20 1100 700]);
axAnim = axes('Parent',fig,'Position',[0.02 0.15 0.57 0.8]);
hold(axAnim,'on');
axis(axAnim,'equal');
grid(axAnim,'on');
xlabel(axAnim,'x [m]');
ylabel(axAnim,'y [m]');
title(axAnim,'Braccio planare 2 GDL: ellisse forza e diagonale maggiore');
xlim(axAnim,[-1.5 0.3]);
ylim(axAnim,[-0.2 l1+l2+0.3]);

panel = uipanel('Parent',fig,'Title','Dati e Controlli','FontSize',12,...
    'Position',[0.67 0.02 0.31 0.95]);
tTab = uitable(panel, ...
    'Data', data_tab, ...
    'ColumnName', {'Parametro','Valore'}, ...
    'ColumnWidth', {110 90}, ...
    'FontSize', 12, ...
    'RowName', [], ...
    'Position', [15 10 260 480], ...
    'ColumnEditable', [false false]);

btnPlay = uicontrol(panel,'Style','pushbutton','String','Play',...
    'Position',[30 500 70 38],'FontSize',12,'BackgroundColor',[0.6 1 0.6]);
btnStop = uicontrol(panel,'Style','pushbutton','String','Stop',...
    'Position',[115 500 70 38],'FontSize',12,'BackgroundColor',[1 0.6 0.6]);
btnRestart = uicontrol(panel,'Style','pushbutton','String','Restart',...
    'Position',[200 500 70 38],'FontSize',12,'BackgroundColor',[0.9 0.9 1]);

editTime = uicontrol(panel, 'Style', 'edit', 'String', '0', ...
    'Position', [30 560 60 30], 'FontSize', 12);

% Scritta ben visibile e non tagliata (due righe, font grande, label larga)
intervalText = uicontrol(panel, 'Style', 'text', ...
    'String', sprintf('Intervallo valido:\n%.2f - %.2f s', t(1), t(end)), ...
    'Position', [30 600 280 38], 'FontSize', 13, ...
    'HorizontalAlignment','left', ...
    'BackgroundColor', get(panel, 'BackgroundColor'));

btnGoToTime = uicontrol(panel, 'Style', 'pushbutton', 'String', 'Vai a tempo', ...
    'Position', [100 560 100 30], 'FontSize', 12, 'BackgroundColor', [0.9 0.9 0.7]);

% Variabili di controllo globali (usate anche nei callback!)
k = 1;
isPlaying = false;
restartFlag = false;
goToFrame = nan;

btnPlay.Callback    = @(src, event) assignin('base','isPlaying',true);
btnStop.Callback    = @(src, event) assignin('base','isPlaying',false);
btnRestart.Callback = @(src, event) assignin('base','restartFlag',true);

% Callback "Vai a tempo": blocca e mostra il frame desiderato
btnGoToTime.Callback = @(src, event) goToTimeCallback(editTime, t(1), t(end), intervalText);

assignin('base','isPlaying',isPlaying);
assignin('base','restartFlag',restartFlag);
assignin('base','goToFrame',goToFrame);

while ishandle(fig)
    % Leggi gli stati dai bottoni
    isPlaying   = evalin('base','isPlaying');
    restartFlag = evalin('base','restartFlag');
    goToFrame   = evalin('base','goToFrame');

    % --- Gestione bottone Vai a tempo: blocca animazione e mostra frame scelto
    if ~isnan(goToFrame)
        goToFrame = max(min(goToFrame, t(end)), t(1)); 
        [~, k_new] = min(abs(t - goToFrame));
        k = k_new;
        assignin('base','goToFrame',nan); % Reset subito!
        assignin('base','isPlaying',false); % Blocca animazione!
        cla(axAnim);
        data_tab = draw_frame(x1, y1, x2, y2, x3, y3, l1, l2, F, data_tab, tTab, axAnim, t, k);
    end

    % Blocca animazione a t > 10
    if isPlaying && t(k) > 10
        disp(['Animazione stoppata a t = ', num2str(t(k)), ' s (frame ', num2str(k), ')']);
        assignin('base','isPlaying',false);
        continue
    end

    if restartFlag
        k = 1;
        assignin('base','restartFlag',false);
        assignin('base','isPlaying',false);
        cla(axAnim);
        data_tab = draw_frame(x1, y1, x2, y2, x3, y3, l1, l2, F, data_tab, tTab, axAnim, t, k);
        continue
    end

    if isPlaying
        cla(axAnim);
        data_tab = draw_frame(x1, y1, x2, y2, x3, y3, l1, l2, F, data_tab, tTab, axAnim, t, k);
        pause(0.1)
        if k < nframe
            k = k + 1;
        end
    else
        pause(0.05);
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
    assignin('base','isPlaying', false); % Blocca la riproduzione sull'istante scelto
end

function data_tab = draw_frame(x1, y1, x2, y2, x3, y3, l1, l2, F, data_tab, tTab, axAnim, t, k)
    S = [x1(k); y1(k)];
    G = [x2(k); y2(k)];
    H = [x3(k); y3(k)];

    theta1 = atan2(y2(k)-y1(k), x2(k)-x1(k));
    theta2 = atan2(y3(k)-y2(k), x3(k)-x2(k)) - theta1;

    J = [-l1*sin(theta1) - l2*sin(theta1+theta2), -l2*sin(theta1+theta2);
          l1*cos(theta1) + l2*cos(theta1+theta2),  l2*cos(theta1+theta2)];

    theta1_deg = rad2deg(theta1);
    theta2_deg = rad2deg(theta2);

    tau = J' * F;
    tau1 = tau(1);
    tau2 = tau(2);

    valori = [J(1,1); J(1,2); J(2,1); J(2,2); ...
              S(1); S(2); G(1); G(2); H(1); H(2); t(k); ...
              theta1_deg; theta2_deg; tau1; tau2];
    for ii = 1:numel(valori)
        if ii<=10
            data_tab{ii,2} = sprintf('%.4f', valori(ii));
        elseif ii==11
            data_tab{ii,2} = sprintf('%.2f', valori(ii));
        else
            data_tab{ii,2} = sprintf('%.2f', valori(ii));
        end
    end
    set(tTab, 'Data', data_tab);

    % --- Disegno manipolatore
    plot(axAnim, [S(1), G(1)], [S(2), G(2)], 'k-', 'LineWidth', 4);
    plot(axAnim, [G(1), H(1)], [G(2), H(2)], 'k-', 'LineWidth', 4);
    plot(axAnim, S(1), S(2), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 10);
    plot(axAnim, G(1), G(2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
    plot(axAnim, H(1), H(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
    phi = linspace(0, 2*pi, 200);
    manipulability = sqrtm(inv(J*J'));
    ellisse = manipulability * [cos(phi); sin(phi)];
    ellisse = 0.08 * ellisse;
    plot(axAnim, H(1) + ellisse(1,:), H(2) + ellisse(2,:), 'm-', 'LineWidth', 2);
    [V,D] = eig(manipulability*manipulability');
    [~, idx] = max(diag(D));
    v_max = V(:,idx);
    len_max = 0.08 * sqrt(D(idx,idx));
    pt1 = H + len_max * v_max;
    pt2 = H - len_max * v_max;
    plot(axAnim, [pt1(1) pt2(1)], [pt1(2) pt2(2)], 'r-', 'LineWidth', 3);
    text(axAnim, S(1), S(2), '  Spalla', 'FontWeight', 'bold', 'FontSize', 10, 'VerticalAlignment', 'bottom');
    text(axAnim, G(1), G(2), '  Gomito', 'FontWeight', 'bold', 'FontSize', 10, 'VerticalAlignment', 'top');
    text(axAnim, H(1), H(2), '  Mano', 'FontWeight', 'bold', 'FontSize', 10, 'VerticalAlignment', 'bottom');
    txt1 = sprintf('\\theta_1 = %.1f^\\circ', theta1_deg);
    txt2 = sprintf('\\theta_2 = %.1f^\\circ', theta2_deg);
    text(axAnim, -0.28, l1+l2+0.10, txt1, 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'b');
    text(axAnim, -0.28, l1+l2+0.05, txt2, 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'r');

    % --- Visualizzazione diretta degli angoli (archi colorati)
    r1 = 0.07;
    phi1 = linspace(0, theta1, 40);
    plot(axAnim, S(1)+r1*cos(phi1), S(2)+r1*sin(phi1), 'b-', 'LineWidth', 3);
    plot(axAnim, S(1)+r1*cos(theta1), S(2)+r1*sin(theta1),'b>','MarkerSize',8,'MarkerFaceColor','b');

    r2 = 0.07;
    phi2 = linspace(0,theta2,40);
    angle0 = atan2(H(2)-G(2),H(1)-G(1)) - theta2;
    x_arc2 = G(1) + r2*cos(angle0 + phi2);
    y_arc2 = G(2) + r2*sin(angle0 + phi2);
    plot(axAnim, x_arc2, y_arc2, 'r-', 'LineWidth', 3);
    plot(axAnim, x_arc2(end), y_arc2(end), 'r>','MarkerSize',8,'MarkerFaceColor','r');

    text(axAnim, S(1)+r1*1.2*cos(theta1/2), S(2)+r1*1.2*sin(theta1/2), ...
        sprintf('\\theta_1=%.1f°',theta1_deg), 'Color','b','FontWeight','bold','FontSize',13);

    text(axAnim, x_arc2(1)+r2*0.7, y_arc2(1), ...
        sprintf('\\theta_2=%.1f°',theta2_deg), 'Color','r','FontWeight','bold','FontSize',13);
end