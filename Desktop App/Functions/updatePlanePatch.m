function [oriPatch] = updatePlanePatch(estOri)
    initPatch = stlread('.\AircraftModels\Gemini.stl');
    oriPatch = initPatch;
    oriPatch.vertices = rotateframe(quaternion(0,0,0.707,0.707),initPatch.vertices);
    oriPatch.vertices = rotateframe(quaternion(0,1,0,0),oriPatch.vertices);
    %oriPatch.vertices
end

