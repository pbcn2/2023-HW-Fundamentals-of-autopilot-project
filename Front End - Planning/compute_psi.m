function psi = compute_psi(PathTab, node)
    % If the current node is the start node, return 0
    % Update the coordinates of the start node if necessary
    if isequal(node, [520, 80])
        psi = 0;
        return;
    end

    % Otherwise, calculate the psi
    parent_node = squeeze(PathTab(node(1), node(2), :))'; 
    psi = atan2(node(2) - parent_node(2), node(1) - parent_node(1));
    
    % If you need to convert psi to degrees, uncomment the following line:
    % psi = rad2deg(psi);
end
