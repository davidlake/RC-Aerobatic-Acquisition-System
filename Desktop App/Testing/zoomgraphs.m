clear all real
close all
clc

fig = figure;
ax = uiaxes(fig);
a = aircraft;
hold(ax);

moveAircraft(a,[50,50,50])
patch(ax,a.outPatchData,'EdgeColor','none');

%ax.Interactions = regionZoomInteraction;
view(ax,0,0)
ax.DataAspectRatio = [1 1 1];
ax.XLim = [-100,100];
ax.YLim = [-100,100];
ax.ZLim = [-100,100];



 