function psi = compute_psi(PathTab, node)
    prev_node = squeeze(PathTab(node(1), node(2), :))';
    if any(prev_node == 0)
        psi = 0;
    else
        dx = node(1) - prev_node(1);
        dy = node(2) - prev_node(2);
        psi = atan2(dy, dx);
    end
end
