function curvature = compute_curvature(PathTab, node)
    prev_node = squeeze(PathTab(node(1), node(2), :))';
    if any(prev_node == 0)
        curvature = NaN;
    else
        prev_prev_node = squeeze(PathTab(prev_node(1), prev_node(2), :))';
        if any(prev_prev_node == 0)
            curvature = NaN;
        else
            dx = prev_node(1) - prev_prev_node(1);
            dy = prev_node(2) - prev_prev_node(2);
            ddx = node(1) - 2*prev_node(1) + prev_prev_node(1);
            ddy = node(2) - 2*prev_node(2) + prev_prev_node(2);
            curvature = sqrt(ddx^2 + ddy^2) / (dx^2 + dy^2)^1.5;
        end
    end
end
