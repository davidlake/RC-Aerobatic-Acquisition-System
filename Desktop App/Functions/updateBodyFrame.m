function [bodyFrameAxisE] = updateBodyFrame(estOri)
    NED_axis = [1 0 0; 0 1 0; 0 0 1]*30;
    nElements = length(estOri(:,1));
    bodyFrameAxisE = zeros(3,3,nElements);
    for c=1:nElements
        bodyFrameAxisE(:,:,c) = rotatepoint(estOri(c),NED_axis);
    end
end

