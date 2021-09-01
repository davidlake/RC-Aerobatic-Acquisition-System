function [oriPatches] = updatePlanePatch(estOri)
    initPatch = stlread('.\AircraftModels\Gemini.stl');
    oriPatches = initPatch;
    oriPatches.vertices = rotateframe(quaternion(0,0,0.707,0.707),initPatch.vertices);
    oriPatches.vertices = rotateframe(quaternion(0,1,0,0),oriPatches.vertices);    
    oriPatches.vertices(:,1) = oriPatches.vertices(:,1)+590;
%     rotPatch = initPatch;
%     rotPatch.vertices = rotateframe(quaternion(0,0,0.707,0.707),initPatch.vertices);
%     rotPatch.vertices = rotateframe(quaternion(0,1,0,0),rotPatch.vertices);
%     nElements = length(estOri);
%     oriPatches = zeros(sz1,...,szN,'quaternion') 
%     for ii = 1:nElements
%         
%     end
end

