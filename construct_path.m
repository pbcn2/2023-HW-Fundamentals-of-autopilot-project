function path = construct_path(node, close_list)
    path = node.pos;
    while isfield(node, 'parent') && isstruct(node.parent)
        node = node.parent;
        path = [node.pos; path];
    end
end

