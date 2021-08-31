function drawFlightTrack(ax)
    hold(ax,'on');
    patch(ax,'XData',[-350,-350,350,350],'YData',[-10,150,150,-10],'ZData',[0,0,0,0],'FaceColor',[0.6510,0.6510,0.6510]); % track
    patch(ax,'XData',[-350,-350,350,350],'YData',[150,200,200,150],'ZData',[0,0,0,0],'FaceColor',[0.4667,0.6745,0.1882]); % track grass
    plot3(ax,[-260,260],[0,0],[0,0],'LineStyle',"--","Color",[1,0.4118,0.1608]); %pilot line
    plot3(ax,[0,-260],[0,150],[0,0],"Color",'w'); %left floor line
    plot3(ax,[0,260],[0,150],[0,0],"Color",'w'); %right floor line
    plot3(ax,[0,0],[0,150],[0,0],"Color",'w'); %mid line
    plot3(ax,[-260,260],[-7,-7],[0,0],'LineStyle',"--","Color",[0.7176,0.2745,1]); %judges line
    plot3(ax,[-260,-260,260,260,-260],[150,150,150,150,150],[0,-270,-270,0,0],'LineStyle',"--","Color",'b'); %window
    plot3(ax,[0,0],[150,150],[0,-270],'LineStyle',"--","Color",'b'); %mid line
    plot3(ax,0,0,0,'Marker',"o",'MarkerEdgeColor','k',"MarkerFaceColor",'k'); %pilot point
    hold(ax,'off');
    ax.DataAspectRatio = [1 1 1];
    ax.XLim = [-350,350];
    ax.YLim = [-10,200];
    ax.ZLim = [-300,0];
    ax.XDir = 'reverse';
    ax.ZDir = 'reverse';
    ax.XLabel.String = 'x North';
    ax.YLabel.String = 'y East';
    ax.ZLabel.String = 'z Down';
    grid(ax,'on');
    view(ax,30,20);
end

