function [] = drawPicture(t,X, params, bookkeeping, ...
                          createVideoBool, videoName,hObject) %#ok<INUSD>

    dtPlot =2* 0.0075;%.03;
    
    % ---interpolate variables---
    badInd = find(t(2:end)-t(1:end-1)==0);
    t(badInd) = [];
    X(badInd,:) = []; 
    tInerp = 0:dtPlot:t(end);
    X = interp1(t,X,tInerp);
    t = tInerp;

    % unpack variables
    [x,y,phi,thLeg,dxdt,dydt,dphi_dt] = unpackState(X);  %#ok<ASGLU>
    [m,I,k,len0,g,b] = unpackParams(params); %#ok<ASGLU>
    [xF, lenLeg, dthLeg_dt] = calculateKinematics(X, params);  %#ok<ASGLU>
    
    % visualizations
    num_its = length(t);
    green = [ 49 102  30]./255;
    gray  = [124 125 123]./255;
    white = [255 255 255]./255;
    groundColor = gray;
    bodyColor   = green;
    legColor    = green;
    lineWidth   = 4;
    
    xWidth = 4;
    xmin = -1;%-xWidth/1;
    xmax =  4;%xWidth/2;
    ymin =  0-0.2;
    ymax =  xWidth-0.2;
     
    if(createVideoBool)
        myVideo = VideoWriter(videoName);
        open(myVideo);
    end
    
    %figure;
    handles = guidata(hObject);
    axes(handles.axes1);
    cla();
    for i=1:num_its
        %clf;
        cla();
        hold on;
         
        % plot leg
        xa = x(i);
        ya = y(i);
        xb = x(i) + lenLeg(i)*sin(thLeg(i));
        yb = y(i) - lenLeg(i)*cos(thLeg(i));
        Xline = [xa; xb];
        Yline = [ya; yb];
        line(Xline,Yline, 'Color', legColor,'LineWidth',lineWidth);
        
        % plot body
        rx = 0.5;
        ry = 0.25;
        plotEllipse(x(i),y(i),rx,ry,phi(i),bodyColor, lineWidth)
        xlim([xmin,xmax]);
        ylim([ymin,ymax]);
        
        % plot ground
        Xline = [xmin; xmax];
        Yline = [0; 0];
        line(Xline,Yline, 'Color', groundColor,'LineWidth',lineWidth);
        
        set(gca,'visible','off')
        set(gcf, 'color', white);
%         set(gca, 'color', 'none');
        drawnow;
        
        if(createVideoBool)
            frame = getframe;
            writeVideo(myVideo, frame);
        end
    end
    
    if(createVideoBool)
        close(myVideo);
    end
end


function R = rotation(theta)
    R = [cos(theta), -sin(theta);
         sin(theta),  cos(theta)];
end

function plotEllipse(x_ctr,y_ctr,rx,ry,phi,color, lineWidth)

    numPts = 100;
    t = linspace(0,2*pi,numPts);
    X = rotation(phi)*[rx*cos(t);ry*sin(t)];
    x = X(1,:) + x_ctr; 
    y = X(2,:) + y_ctr;

    plot(x,y,'Color', color,'LineWidth',lineWidth);
end