function vechicle2dsim
clc

H = open('mainwind.fig');
handles = guihandles(H);

set(H,'KeyPressFcn',{@keys_contr, handles})

set(handles.pushbutton_Up,'Callback',{@pushbutton_Up_Callback, handles})
set(handles.pushbutton_Rt,'Callback',{@pushbutton_Rt_Callback, handles})
set(handles.pushbutton_Dn,'Callback',{@pushbutton_Dn_Callback, handles})
set(handles.pushbutton_Lf,'Callback',{@pushbutton_Lf_Callback, handles})
set(handles.pushbutton_Pl,'Callback',{@pushbutton_Pl_Callback, handles})
set(handles.pushbutton_Mn,'Callback',{@pushbutton_Mn_Callback, handles})
set(handles.pushbutton_SSD,'Callback',{@pushbutton_SaveSensData_Callback, handles})
set(handles.pushbutton_Start,'Callback',{@pushbutton_Start_Callback, handles})
set(handles.pushbutton_Stop,'Callback',{@pushbutton_Stop_Callback, handles})

set(handles.slider_X,'Value',0)
set(handles.slider_Y,'Value',0)
set(handles.slider_X,'Min',-1,'Max',1)
set(handles.slider_Y,'Min',-1,'Max',1)
set(handles.slider_X,'Callback',{@slider_X_Callback, handles})
set(handles.slider_Y,'Callback',{@slider_Y_Callback, handles})

set(handles.slider_coner,'Value',90)
set(handles.slider_coner,'Min',1,'Max',360)
set(handles.slider_coner,'Callback',{@slider_coner_Callback, handles})

handles.num_rays = [1,2,4,8,16,24,36,48,72,96,124,128,192,240,360];
set(handles.popupmenu1,'String',handles.num_rays,...
                       'Callback',{@popupmenu_Real_Callback, handles})
set(handles.popupmenu2,'String',handles.num_rays,...
                       'Callback',{@popupmenu_Vsbl_Callback, handles})                   

hFileMenu = uimenu(...       % File menu
                    'Parent',H,...
                    'HandleVisibility','callback', ...
                    'Label','File');
OpenMenuitem = uimenu(...       % Open menu item
                        'Parent',hFileMenu,...
                        'Label','Open',...
                        'HandleVisibility','callback', ...
                        'Callback', {@hOpenMenuitemCallback, handles});
hPrintMenuitem = uimenu(...       % Print menu item
                        'Parent',hFileMenu,...
                        'Label','Print',...
                        'HandleVisibility','callback', ...
                        'Callback', @hPrintMenuitemCallback);
hCloseMenuitem = uimenu(...       % Close menu item
                        'Parent',hFileMenu,...
                        'Label','Close',...
                        'Separator','on',...
                        'HandleVisibility','callback', ...
                        'Callback', 'close(gcf)');
                    
                    
hModeContr = uimenu(...       % Mode menu
                    'Parent',H,...
                    'HandleVisibility','callback', ...
                    'Label','Mode');
Manual_item = uimenu(...       % Open menu item
                        'Parent',hModeContr,...
                        'Label','Manual',...
                        'HandleVisibility','callback', ...
                        'Callback', {@hManual_Callback, handles});
Autonomous_item = uimenu(...       % Print menu item
                        'Parent',hModeContr,...
                        'Label','Autonomous',...
                        'HandleVisibility','callback', ...
                        'Callback', {@hAutonomous_Callback, handles});                    
                   
                   
guidata(H, handles)




function hOpenMenuitemCallback(src, evt, handles)

par = guidata(handles.win_main);
    par.scope = 1;
    par.autonomous = 0;
    
    set(gcf,'CurrentAxes',par.AxMap)
    cla
    set(gcf,'CurrentAxes',par.AxPolar)
    cla
    set(gcf,'CurrentAxes',par.AxSens_1)
    cla
    set(gcf,'CurrentAxes',par.AxSens_2)
    cla
    
    [fname, path] = uigetfile({'*.mat'},'Select a File');
    
    tmp = load([path, fname], 'map');
    par.map = tmp.map;
    
    if isfield(par.map,'NumRowSens')
        par.num = par.map.NumRowSens;
        set(handles.popupmenu1,'Value',find(par.num_rays == par.map.NumRowSens))
        set(handles.popupmenu2,'Value',find(par.num_rays == par.map.NumRowSens)) 
    else
        par.num = par.num_rays(get(handles.popupmenu1,'Value'));
    end
    
    par.d = 20;
    
    par.coner = pi / 2;
    
    set(par.edit_coner,'String',num2str(par.coner / pi * 360))
    set(handles.slider_coner,'Value',par.coner / pi * 360)
    par.alfa =  2 * par.coner / par.num;
    par.alf_step = pi / 16;
    
    par.stp = 2;
    par.bdy = 4;
    
    if isfield(par.map,'Rot')
        par.rot = par.map.Rot;
    else
        par.rot = pi / 2;
    end
    
    if isfield(par.map,'Start')
        par.X = par.map.Start(1);
        par.Y = par.map.Start(2);        
    else
        par.X = 30;
        par.Y = 50;
    end
    
    par.raspr = round(linspace(1,...
                               par.num_rays(get(handles.popupmenu1,'Value')),...
                               par.num_rays(get(handles.popupmenu2,'Value'))));
    
                           
    par.go = 1;
    
    par.W1 = zeros(64,40);
    par.W2 = zeros(2, 64);
    par.ML = rot90(hadamard(64),2);
    par.NUMC = 0;
    par.inp = ones(40,3);
    
    par.o = [];
       
guidata(handles.win_main, par)
DrawMap(handles) 


function DrawMap(handles)

par = guidata(handles.win_main);

    set(gcf,'CurrentAxes',par.AxMap)
    
    if isfield(par.map,'Rect')
        par.nBr = size(par.map.Rect, 1);
        for ii = 1:  par.nBr
            rectangle('Position', par.map.Rect(ii,:), 'FaceColor','g');
            hold on;
        end
    end
%     t = par.map.Rect
    par.map.Rect = [par.map.Rect; [par.map.Border(1:2) - par.map.Border(3:4), par.map.Border(1:2) + par.map.Border(3:4)]];
%     tt = par.map.Rect
    
    if isfield(par.map,'Circle')
        par.nCr = size(par.map.Circle, 1);
        for ii = 1:  par.nCr
            cxy = par.map.Circle(ii,:);
            rectangle('Position', [cxy(1)-cxy(3) cxy(2)-cxy(3) 2*cxy(3) 2*cxy(3)], 'FaceColor','g','Curvature',[1 1]);
            hold on;
        end
    end
%     axis([0 100 0 100])
    if par.map.Border(3) <= par.map.Border(4)
        par.WHL = par.map.Border(3);
    else
        par.WHL = par.map.Border(4);
    end

%     rectangle('Position', [par.map.Border(1:2) - par.map.Border(3:4), par.map.Border(1:2) + par.map.Border(3:4)])
    axis([par.map.Border(1) - par.WHL
          par.map.Border(1) + par.WHL
          par.map.Border(2) - par.WHL
          par.map.Border(2) + par.WHL])

 
% %     grid onA
%     AxMap_Scope_Monitor(handles)
    
    Ds = ones(1, par.num) * par.d;
    xy = sensyst(par.X, par.Y, Ds, par.rot, par.coner);
%     raspr = round(linspace(1,par.num_rays(get(par.popupmenu1,'Value')),par.num_rays(get(par.popupmenu2,'Value'))))
    
    par.RF = plot(xy(:,1),xy(:,2),'-ro');                          % Raws
    hold on
    
    xy = sens_obstacl(par.X, par.Y, ones(1,8)*8, pi/2);
    par.SB = fill(xy(1,1:4), xy(2,1:4),'r',...
                  xy(1,5:8), xy(2,5:8),'r',...
                  xy(1,9:12), xy(2,9:12),'r',...
                  xy(1,13:16), xy(2,13:16),'r',...
                  xy(1,17:20), xy(2,17:20),'r',...
                  xy(1,21:24), xy(2,21:24),'r',...
                  xy(1,25:28), xy(2,25:28),'r',...
                  xy(1,29:32), xy(2,29:32),'r','FaceAlpha',0.1);
              
%     set(par.SB(1),'FaceAlpha',0.5)
%     set(par.SB(4),'Color','m')


    hold on
              
    par.MR = fill(...                                              % MobRob 
            [par.X + par.bdy * cos(par.rot),...
             par.X + par.bdy * cos(par.rot + 3 * pi / 4),...
             par.X + par.bdy * cos(par.rot + 5 * pi / 4),...
             par.X + par.bdy * cos(par.rot)],...
            [par.Y + par.bdy * sin(par.rot),...
             par.Y + par.bdy * sin(par.rot + 3 * pi / 4),...
             par.Y + par.bdy * sin(par.rot + 5 * pi / 4),...
             par.Y + par.bdy * sin(par.rot)],...
            'b','FaceAlpha',0.999);              
%     hold on

   set(gcf,'CurrentAxes',par.AxPolar)
%      par.Radar = polar(pi/2:2*pi / length(raspr): 5*pi/2,[Ds(raspr),Ds(1)],'--ro');
   
%    par.Radar = polar(pi/2:2*pi / par.num : 5*pi/2,[Ds,Ds(1)],'--ro');
    par.Radar = polar(pi/2 - par.coner: 2*par.coner / par.num : pi/2 + par.coner,[Ds,Ds(1)],'--ro');
 
   hold on
   par.MRRadar = fill(...
        [par.bdy * cos(pi/2),...
         par.bdy * cos(5 * pi/4),...
         par.bdy * cos(7 * pi/4),...
         par.bdy * cos(pi / 2)],...
        [par.bdy * sin(pi / 2),...
         par.bdy * sin(5 * pi / 4),...
         par.bdy * sin(7 * pi/4),...
         par.bdy * sin(pi/2)],...
        'b');
    
    hold on    
    set(gcf,'CurrentAxes',par.AxSens_1)
    title('Real sensory data')
    par.LinSens = plot(1:par.num, Ds, '-ro');
    if par.num == 1
        axis([0 1 1 1.1 * par.d])
    else
        axis([1 par.num 1 1.1 * par.d])
    end
    
    grid on
    
    set(gcf,'CurrentAxes',par.AxSens_2)
    xy(1,:) = xy(1,:) - par.X;
    xy(2,:) = xy(2,:) - par.Y;
    par.SBpol = fill(xy(1,1:4),   xy(2,1:4),  'r',...
                     xy(1,5:8),   xy(2,5:8),  'r',...
                     xy(1,9:12),  xy(2,9:12), 'r',...
                     xy(1,13:16), xy(2,13:16),'r',...
                     xy(1,17:20), xy(2,17:20),'r',...
                     xy(1,21:24), xy(2,21:24),'r',...
                     xy(1,25:28), xy(2,25:28),'r',...
                     xy(1,29:32), xy(2,29:32),'r','FaceAlpha',0.5);
   
    guidata(handles.win_main, par)

RangeFinder(handles)


function RangeFinder(handles)

par = guidata(handles.win_main);

   xy = sens_obstacl(par.X, par.Y, ones(1,8)*8, par.rot);
   set(par.SB,'Xdata',xy(1,:),'ydata',xy(2,:))
%    set(par.SB(4),'Color','m')
%    set(par.SB(6),'FaceAlpha',0.7)
    otvet = zeros(1,8);
    Ds = ones(1, par.num) * par.d;

    if isfield(par.map,'Rect')
        for kk = 1: par.nBr + 1
            if sqrt((par.map.Rect(kk,1) + par.map.Rect(kk,3)/2 - par.X)^2 + (par.map.Rect(kk,2) + par.map.Rect(kk,4)/2 - par.Y)^2) <= par.d + sqrt((par.map.Rect(kk,3)/2)^2 + (par.map.Rect(kk,4)/2)^2)

                for jj = 1: par.num
                    [tmpD, tmpX, tmpY] = linecrossrect(...
                        [par.X,...
                         par.Y,...
                         par.X + par.d * cos(par.alfa * (jj - 1) + par.rot - par.coner),...
                         par.Y + par.d * sin(par.alfa * (jj - 1) + par.rot - par.coner)],...
                         par.map.Rect(kk,:) );

                    if tmpD < Ds(jj)
                        Ds(jj) = tmpD; 
                        P_XY(jj,1) = tmpX;
                        P_XY(jj,2) = tmpY;
                    end
                end

                for qq = 1: 8
                    if ~otvet(qq)
                        otvet(qq) = trianglecrossrect(xy(:,(qq-1)*4+1:(qq-1)*4+4), par.map.Rect(kk,:), 8);
                    end
                end

            end
        end
    end
    
    if isfield(par.map,'Circle')
        for kk = 1: par.nCr
            if sqrt((par.map.Circle(1) - par.X)^2 + (par.map.Circle(2) - par.Y)^2) < par.d + par.map.Circle(3)

                for jj = 1: par.num
                    [tmpD, tmpX, tmpY] = linecrosscircle(...
                        [par.X,...
                         par.Y,...
                         par.X + par.d * cos(par.alfa * (jj - 1) + par.rot - par.coner),...
                         par.Y + par.d * sin(par.alfa * (jj - 1) + par.rot - par.coner)],...
                         par.map.Circle(kk,:), par.d);
                  
     
                    if tmpD < Ds(jj)
                        Ds(jj) = tmpD; 
                        P_XY(jj,1) = tmpX;
                        P_XY(jj,2) = tmpY;
                    end
                end

                for qq = 1: 8
                    if ~otvet(qq)
%                         trianglecrosscircle(trn, crc, d)
                        otvet(qq) = trianglecrosscircle(xy(:,(qq-1)*4+1:(qq-1)*4+4), par.map.Circle(kk,:), 8);
                    end
                end

            end
        end
    end
    
%     otvet
    set(par.SBpol(1:8),'FaceAlpha',0.5)
    set(par.SBpol(logical(otvet)),'FaceAlpha',1)
    
    xy = sensyst(par.X, par.Y, Ds, par.rot, par.coner);

    tmp_x = reshape(xy(:,1),[2 par.num]);
    RFx = reshape(tmp_x(:,par.raspr), [length(par.raspr) * 2 1]);
    tmp_y = reshape(xy(:,2),[2 par.num]);
    RFy = reshape(tmp_y(:,par.raspr), [length(par.raspr) * 2 1]);
    
    set(par.RF, 'XData', RFx,'YData',RFy)
    set(par.MR, 'Xdata',...
        [par.X + par.bdy * cos(par.rot),...
         par.X + par.bdy * cos(par.rot + 3 * pi / 4),...
         par.X + par.bdy * cos(par.rot + 5 * pi / 4),...
         par.X + par.bdy * cos(par.rot)],...
         'Ydata',...
        [par.Y + par.bdy * sin(par.rot),...
         par.Y + par.bdy * sin(par.rot + 3 * pi / 4),...
         par.Y + par.bdy * sin(par.rot + 5 * pi / 4),...
         par.Y + par.bdy * sin(par.rot)]);
    
   [xx,yy] = pol2cart(pi/2 - par.coner : 2*par.coner / length(par.raspr): pi/2 + par.coner,[Ds(par.raspr),0]);
   set(par.Radar,'Xdata',xx,'Ydata',yy)

   set(par.LinSens,'Xdata', 1:par.num, 'Ydata',fliplr(Ds))
   
%    set(par.SBpol,'Xdata',xy(1,:) - par.X,'ydata',xy(2,:) - par.Y)
%    set(par.SBpol(1),'FaceAlpha',0.8)
    par.otvet = otvet;
     
guidata(handles.win_main, par)


function pushbutton_Pl_Callback(src, evt, handles)

par = guidata(handles.win_main);

    if par.scope > 0.2
        par.scope = par.scope - 0.1;
    end
   
guidata(handles.win_main, par)
    AxMap_Scope_Monitor(handles)

function pushbutton_Mn_Callback(src, evt, handles)

par = guidata(handles.win_main);

    if par.scope < 1
        par.scope = par.scope + 0.1;
    end
    
guidata(handles.win_main, par)
    AxMap_Scope_Monitor(handles)

function slider_Y_Callback(src, evt, handles)

AxMap_Scope_Monitor(handles)

function slider_X_Callback(src, evt, handles)

AxMap_Scope_Monitor(handles)   

function AxMap_Scope_Monitor(handles)

par = guidata(handles.win_main);

%     set(par.AxMap,'Xlim', par.map.Border(1) + [-1 1] * par.map.Border(3) * par.scope + (1 - par.scope) * par.map.Border(1) * get(par.slider_X,'Value') )
%     set(par.AxMap,'Ylim', par.map.Border(2) + [-1 1] * par.map.Border(4) * par.scope + (1 - par.scope) * par.map.Border(2) * get(par.slider_Y,'Value') )
%     set(par.AxMap,'Xlim', par.map.Border(1) + [-1 1] * par.WHL * par.scope + (1 - par.scope) * par.map.Border(1) * get(par.slider_X,'Value') * 2 * par.map.Border(3) / par.WHL )
%     set(par.AxMap,'Ylim', par.map.Border(2) + [-1 1] * par.WHL * par.scope + (1 - par.scope) * par.map.Border(2) * get(par.slider_Y,'Value') * 2 * par.map.Border(4) / par.WHL) 
    set(par.AxMap,'Xlim', par.map.Border(1) + [-1 1] * par.WHL * par.scope + get(par.slider_X,'Value') * (par.map.Border(3) - par.WHL * par.scope) )
    set(par.AxMap,'Ylim', par.map.Border(2) + [-1 1] * par.WHL * par.scope + get(par.slider_Y,'Value') * (par.map.Border(4) - par.WHL * par.scope) )

guidata(handles.win_main, par) 

function [xy] = sensyst(x, y, d, rot, coner)

if nargin == 3
    rot = 0;
    coner = 0;
end
num = length(d);
alfa = 2 * coner / num;
xy = zeros(2 * num, 2);
for ii = 1: num
    xy((ii - 1) * 2 + 1,:) = [x, y];
    xy(ii * 2 ,:) = [x + d(ii) * cos(alfa * (ii - 1) + rot - coner),...
                     y + d(ii) * sin(alfa * (ii - 1) + rot - coner)];
end

function [xy] = sens_obstacl(x, y, d, rot)

if nargin == 3
    rot = 0;
end
num = length(d);
z_angl = 2 * pi / num;
w_angl = z_angl - pi / 36;
xy = zeros(2, 4 * num);
rot = rot - w_angl/2;
for ii = 1: num
    xy(1, (ii - 1)*4 + 1)  = x;
    xy(1, (ii - 1)*4 + 2)  = x + d(ii) * cos(rot);    xy(1, (ii - 1)*4 + 3)  = x + d(ii) * cos(rot + w_angl);
    xy(1, (ii - 1)*4 + 4)  = x;
    
    xy(2, (ii - 1)*4 + 1)  = y;
    xy(2, (ii - 1)*4 + 2)  = y + d(ii) * sin(rot);
    xy(2, (ii - 1)*4 + 3)  = y + d(ii) * sin(rot + w_angl);
    xy(2, (ii - 1)*4 + 4)  = y;
    
    rot = rot + w_angl+ pi / 36;
end


function [d, x, y] = linecrossrect(ln, rct)

bias = [0 0
        0 1
        1 1
        1 0
        0 0 ]; % пересчёт точек прямоугольника
    
d = sqrt((ln(1) - ln(3))^2 + (ln(2) - ln(4))^2); % длина луча
x = ln(3); y = ln(4); % значение искомых точек по умолчанию (если не будет преграды)

for ii = 1: 4
    r12 = rct(1:2)+ bias(ii,1:2).* rct(3:4);      % координаты первой точки стороны
    r34 = rct(1:2) + bias(ii + 1,1:2).* rct(3:4); % координаты второй точки стороны
    [tx, ty] = linecross(ln, [r12, r34]); % пересечение линии и стороны прямоугольника   
    tmp = sqrt((ln(1) - tx)^2 + (ln(2) - ty)^2); % расст. пересечения

    if tmp < d &&...    % убираем мнимые точки пересечения
       abs(tx - r12(1)) <= rct(3) &&...
       abs(tx - r34(1)) <= rct(3) &&...
       abs(ty - r12(2)) <= rct(4) &&...
       abs(ty - r34(2)) <= rct(4) &&...
       sqrt((ln(1) - tx)^2 + (ln(2) - ty)^2) + sqrt((ln(3) - tx)^2 + (ln(4) - ty)^2) - sqrt((ln(1) - ln(3))^2 + (ln(2) - ln(4))^2) < 0.1
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% лищнее убрать
        d = tmp; x = tx; y = ty;   % новые значения
    end
end

function [otvet] = trianglecrossrect(trn, rct, d)

otvet = 0;

bias = [0 0
        0 1
        1 1
        1 0
        0 0 ]; % пересчёт точек прямоугольника
    
for ii = 1: 4
    r12 = rct(1:2) + bias(ii,1:2).* rct(3:4);     % координаты первой точки стороны
    r34 = rct(1:2) + bias(ii + 1,1:2).* rct(3:4); % координаты второй точки стороны

    for jj = 1: 3
    [tx(jj), ty(jj)] = linecross([trn(:,jj)' trn(:,jj+1)'], [r12, r34]); % пересечение линии и стороны прямоугольника   
    
        if sqrt((trn(1,jj) - tx(jj))^2 + (trn(2,jj) - ty(jj))^2) + sqrt((trn(1,jj+1) - tx(jj))^2 + (trn(2,jj+1) - ty(jj))^2) - sqrt((trn(1,jj) - trn(1,jj+1))^2 + (trn(2,jj) - trn(2,jj+1))^2) < 0.1 &&...    
           sqrt((r12(1) - tx(jj))^2 + (r12(2) - ty(jj))^2) + sqrt((r34(1) - tx(jj))^2 + (r34(2) - ty(jj))^2) == sqrt((r12(1) - r34(1))^2 + (r12(2) - r34(2))^2)
            otvet = 1;
            break
        end
    end
    if otvet, break, end
end


function [otvet] = trianglecrosscircle(trn, crc, d)

otvet = 0;
%     r12 = rct(1:2) + bias(ii,1:2).* rct(3:4);     % координаты первой точки стороны
%     r34 = rct(1:2) + bias(ii + 1,1:2).* rct(3:4); % координаты второй точки стороны
%     [tx(1), ty(1)] = linecrosscircle([trn(:,1)' trn(:,2)'], [r12, r34]); % пересечение линии и стороны прямоугольника   
clc
%     for jj = 1: 2:3
%         td = linecrosscircle([trn(:,jj)' trn(:,jj+1)'], crc, d)
%         if td && td < d
%             otvet = 1;
%             break
%         end
%     end

    for ii = 1: 3
        [x y] = circlecross(crc, [trn(:,ii)' trn(:,ii+1)']);
        for jj = 1: numel(x)
            if sqrt((trn(1,ii) - x(jj))^2 + (trn(2,ii) - y(jj))^2) + ...
               sqrt((trn(1,ii+1) - x(jj))^2 + (trn(2,ii+1) - y(jj))^2) - ...
               sqrt((trn(1,ii) - trn(1,ii+1))^2 + (trn(2,ii) - trn(2,ii+1))^2) < 0.001
                otvet = 1;
                break
            end
        end
        if otvet, break, end
    end



function [d, x, y] = linecrosscircle(ln, crc, d)
% % %  пересечение отрезка и линии

    x = ln(3); y = ln(4); % значение искомых крайних точек по умолчанию (если не будет преграды)

    [tx ty] = circlecross(crc, ln);

% % %     if numel([tx ty]) > 0 && sqrt((tx(1) - x)^2 + (ty(1) - y)^2)  - d < 0.001
% % % 
% % %         if numel([tx ty]) == 2
% % % 
% % %             d = sqrt((tx - par.X)^2+(ty - par.Y)^2);
% % %             x = tx;
% % %             y = ty;
% % %         elseif numel([tx ty]) == 4 
% % % 
% % %             tmp = sqrt((tx - ln(1)).^2+(ty - ln(2)).^2);
% % % 
% % %             if tmp(1) > tmp(2)
% % %                 d = tmp(2);
% % %                 x = tx(2);
% % %                 y = ty(2);
% % %             else
% % %                 d = tmp(1);
% % %                 x = tx(1);
% % %                 y = ty(1);
% % %             end
% % %         end
% % %     end

    a = sqrt((tx - ln(1)).^2 + (ty - ln(2)).^2);
    b = sqrt((tx - ln(3)).^2 + (ty - ln(4)).^2);
    td = d;
    if numel(a) > 1
        for ii = numel(a)/2 : size(a,2)
            if a(ii) + b(ii) - d < 0.001
                if a(ii) < td
                    td = a(ii);
                    x = tx(ii);
                    y = ty(ii);
                end
            end
        end
    end    
    d = td;

function [x, y] = linecross(xy1,xy2)
% % %  пересечение двух линий

A1 = xy1(2) - xy1(4);
B1 = xy1(3) - xy1(1);
C1 = xy1(1) * xy1(4) - xy1(3) * xy1(2);

A2 = xy2(2) - xy2(4);
B2 = xy2(3) - xy2(1);
C2 = xy2(1) * xy2(4) - xy2(3) * xy2(2);

if abs(A1 * B2 - A2 * B1) < 0.001 % если параллельны
    x = xy1(3);
    y = xy1(4);
else 
    x = (B1 * C2 - B2 * C1) / (A1 * B2 - A2 * B1);
    y = (C1 * A2 - C2 * A1) / (A1 * B2 - A2 * B1);
end


function [x, y] = circlecross(cxy, lxy)
% % %  пересечение окружности и линии

lxy = lxy - [cxy(1) cxy(2) cxy(1) cxy(2)];

EPS = 0.001;

A = lxy(2) - lxy(4);
B = lxy(3) - lxy(1);
C = lxy(1) * lxy(4) - lxy(3) * lxy(2);

    x0 = - A * C / (A*A + B*B);
    y0 = - B * C / (A*A + B*B);
    
if C*C - cxy(3)^2 * (A*A + B*B) > EPS
    x = [];
    y = [];
elseif abs(C*C - cxy(3)^2 * (A*A + B*B)) < EPS
    x = x0 + cxy(1);
    y = y0 + cxy(2);
else 
	D = cxy(3)^2  - C*C / (A*A + B*B);
	mult = sqrt (D / (A*A + B*B));
	x(1) = x0 + B * mult + cxy(1);
	x(2) = x0 - B * mult + cxy(1);
	y(1) = y0 - A * mult + cxy(2);
	y(2) = y0 + A * mult + cxy(2);
end





function radiobutton_manu_Callback( src, evt, handles)

    set(handles.radiobutton_auto,'Value',0)

function radiobutton_auto_Callback( src, evt, handles)
   
    set(handles.radiobutton_manu,'Value',0)
    
    
function pushbutton_Up_Callback(src, evt, handles)

par = guidata(handles.win_main);
%     tX = par.X;
%     tY = par.Y;
    if ~par.autonomous
       par.X = par.X + par.stp * cos(par.rot); 
       par.Y = par.Y + par.stp * sin(par.rot); 
    end
guidata(handles.win_main, par)
RangeFinder(handles)
% RotCheck(handles, tX, tY)

function pushbutton_Dn_Callback(src, evt, handles)

par = guidata(handles.win_main);
%     tX = par.X;
%     tY = par.Y;
    if ~par.autonomous
       par.X = par.X - par.stp * cos(par.rot); 
       par.Y = par.Y - par.stp * sin(par.rot);
    end

guidata(handles.win_main, par)
RangeFinder(handles)
% RotCheck(handles, tX, tY)

function pushbutton_Rt_Callback(src, evt, handles)

par = guidata(handles.win_main);
    if ~par.autonomous
        par.rot = par.rot - par.alf_step;
    end
guidata(handles.win_main, par)
RangeFinder(handles)

function pushbutton_Lf_Callback(src, evt, handles)

par = guidata(handles.win_main);
    if ~par.autonomous
        par.rot = par.rot + par.alf_step;
    end
guidata(handles.win_main, par)
RangeFinder(handles)

function RotCheck(handles, tX, tY)

par = guidata(handles.win_main);
    if par.X ~= tX
        par.rot = atan((par.Y - tY) / (par.X - tX));
    else
        if par.Y >= tY
            par.rot = pi / 2;
        else
            par.rot = -pi / 2;
        end
    end
    
    if par.X < tX, par.rot = par.rot + pi; end
    
guidata(handles.win_main, par)

RangeFinder(handles)

function popupmenu_Real_Callback(src, evt, handles)

par = guidata(handles.win_main);
set(handles.popupmenu2,'String',par.num_rays(1:get(handles.popupmenu1,'Value')),...
                       'Callback',{@popupmenu_Real_Callback, handles})
                   
par.num = par.num_rays(get(handles.popupmenu1,'Value'));
par.alfa =  2 * par.coner / par.num;
if get(handles.popupmenu1,'Value') < get(handles.popupmenu2,'Value')
    set(handles.popupmenu2,'Value',get(handles.popupmenu1,'Value'))
end

par.raspr = round(linspace(1,...
                           par.num_rays(get(handles.popupmenu1,'Value')),...
                           par.num_rays(get(handles.popupmenu2,'Value'))));

if par.num == 1
    set(handles.AxSens_1,'Xlim',[0 1])
else
    set(handles.AxSens_1,'Xlim',[1 par.num])
end

guidata(handles.win_main, par)
RangeFinder(handles)
                   
function popupmenu_Vsbl_Callback(src, evt, handles)

RangeFinder(handles)                  


function keys_contr(src,evt, handles)
% t = evt.Key
     switch evt.Key
         case 'uparrow'
             pushbutton_Up_Callback(src, evt, handles)
   
         case 'downarrow'
             pushbutton_Dn_Callback(src, evt, handles)
             
         case 'rightarrow'
             pushbutton_Rt_Callback(src, evt, handles)
             
         case 'leftarrow'
             pushbutton_Lf_Callback(src, evt, handles)
     end
  
  
function hManual_Callback(src, evt, handles)

par = guidata(handles.win_main);
    par.autonomous = 0;
guidata(handles.win_main, par)

function hAutonomous_Callback(src, evt, handles)

par = guidata(handles.win_main);
    par.autonomous = 1;
        
    [fname path] = uigetfile('*.m');
    
    par.cntrl_sys = fname(1:find(fname == '.') - 1);
    
guidata(handles.win_main, par)
    
function pushbutton_SaveSensData_Callback(src, evt, handles)

[filename, pathname] = uiputfile( ...
                                {'*.dat', 'Data files (*.dat)';
                                 '*.txt', 'Text files (*.txt)';...
                                 '*.*',   'All Files (*.*)'},...
                                 'Save as',...
                                 'sensdata.dat');    
par = guidata(handles.win_main);
    
    dlmwrite([pathname, filename], get(par.LinSens,'YData'))

guidata(handles.win_main, par)


function pushbutton_Start_Callback(src, evt, handles)
par = guidata(handles.win_main);
    par.go = 1;
while par.go == 1;
    
% =========================================================== 
    PDD = par.otvet

% =========================================================== 
    LSD = get(par.LinSens,'YData')
% ===========================================================    


% % p1 | p2 | Действие
% % --------------------
% % 0  | 0  | Стоять
% % 0  | 1  | Налево
% % 1  | 0  | Направо 
% % 1  | 1  | Вперёд
        
        p = [0 0];
        
        par.rot = par.rot + (p(2) - p(1)) * par.alf_step;
        par.X = par.X + (p(2) * p(1)) * par.stp * cos(par.rot); 
        par.Y = par.Y + (p(2) * p(1)) * par.stp * sin(par.rot);
    
        guidata(handles.win_main, par)
        RangeFinder(handles)
        pause(0.05)
        
        p(1) = 0;
        p(2) = 0;
     
%     guidata(handles.win_main, par)
    par = guidata(handles.win_main);
end




function pushbutton_Stop_Callback(src, evt, handles)
par = guidata(handles.win_main);
    
    par.go = 0;
    
guidata(handles.win_main, par)


function slider_coner_Callback(src, evt, handles)

par = guidata(handles.win_main);
    set(par.edit_coner,'String',num2str(round(get(par.slider_coner,'Value'))))
%     t = get(par.slider_coner,'Value')
    par.coner = get(par.slider_coner,'Value') * pi / 360;
    par.alfa =  2 * par.coner / par.num;
guidata(handles.win_main, par)
RangeFinder(handles)

