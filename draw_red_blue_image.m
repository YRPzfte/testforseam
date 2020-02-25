%IL = imread('result/dataset12/save_2/left.png');
%IR = imread('result/dataset12/save_2/right.png');
root_dir = 'datasets/飞碟/save_1/';
IL = imread([root_dir, 'left.png']);
IR = imread([root_dir, 'right.png']);
A = stereoAnaglyph(IL,IR);  % 创建立体图
imwrite(A, [root_dir, 'red_blue.png']);

