% function curvature = compute_curvature(PathTab, node)
%     % If the current node is the start node, return 0
%     if isequal(node, [530, 70])
%         curvature = 0;
%         return;
%     end
% 
%     % Otherwise, calculate the curvature
%     prev_node = squeeze(PathTab(node(1), node(2), :))';  % Parent node
%     next_node = squeeze(PathTab(prev_node(1), prev_node(2), :))';  % Grandparent node
%     
%     % Avoid computing curvature for nodes without a grandparent
%     if isequal(next_node, [0, 0])
%         curvature = 0;
%         return;
%     end
%     
%     psi_prev = atan2(prev_node(2) - node(2), prev_node(1) - node(1));
%     psi_next = atan2(node(2) - prev_node(2), node(1) - prev_node(1));
%     curvature = psi_next - psi_prev;
% end
function curvature = compute_curvature(PathTab, node)
    % If the current node is the start node, return 0
    if isequal(node, [530, 70])
        curvature = 0;
        return;
    end

    % Otherwise, calculate the curvature
    prev_node = squeeze(PathTab(node(1), node(2), :))';  % Parent node
    next_node = squeeze(PathTab(prev_node(1), prev_node(2), :))';  % Grandparent node
    
    % Avoid computing curvature for nodes without a grandparent
    if isequal(next_node, [0, 0])
        curvature = 0;
        return;
    end
    
    % compute the curvature
    x1 = next_node(1);
    y1 = next_node(2);
    x2 = prev_node(1);
    y2 = prev_node(2);
    x3 = node(1);
    y3 = node(2);

    curvature = 2*abs((x2-x1)*(y3-y1)-(y2-y1)*(x3-x1)) / sqrt(((x2-x1)^2 + (y2-y1)^2) * ((x3-x1)^2 + (y3-y1)^2) * ((x3-x2)^2 + (y3-y2)^2));
end
