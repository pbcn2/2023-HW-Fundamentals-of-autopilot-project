load('sysu_standard.mat','map');
map = ~map;
Imp_R = repmat(map*255, [1 1 3]);
imshow(Imp_R);