# -*- coding: utf-8 -*-
import cv2
import sys
import os
import argparse
import numpy as np
parser = argparse.ArgumentParser(description='detect and save sift')
parser.add_argument('--source-dir', type=str, default='datasets/dataset2/save_1')
parser.add_argument('--target-dir', type=str, default='datasets/dataset2/save_2')
parser.add_argument('--out-dir', type=str, default='build')
parser.add_argument('--show-sift', type=bool, default=False)
args = parser.parse_args()

def draw_match(source_img, target_img, source_xy, target_xy):
    source_height, source_width = source_img.shape[:2]
    target_height, target_width = target_img.shape[:2]

    vis = np.zeros((max(source_height, target_height), source_width + target_width + 30, 3), np.uint8)
    vis.fill(255)
    vis[:source_height, :source_width] = source_img
    vis[:target_height, source_width + 30:source_width + target_width + 30] = target_img
    assert len(source_xy) == len(target_xy)
    for i in range(0, len(source_xy)):
        cv2.circle(vis, (source_xy[i][0], source_xy[i][1]), 5, (150, 62, 255), -1)
        cv2.circle(vis, (30 + source_width + target_xy[i][0], target_xy[i][1]), 5, (0, 165, 255), -1)
        cv2.line(vis, (source_xy[i][0], source_xy[i][1]), (30 + source_width + target_xy[i][0], target_xy[i][1]), (0, 238, 0))
    cv2.imwrite("2D-sift.png", vis)

if __name__ == "__main__":
    if not args.source_dir or not args.target_dir:
        print("params not content, should be: ./detect_SIFT.py --source-dir={} --target-dir={}")
        sys.exit()
    source_dir = args.source_dir
    target_dir = args.target_dir
    source_img = cv2.imread(os.path.join(source_dir, 'left.png'))
    target_img = cv2.imread(os.path.join(target_dir, 'left.png'))
    source_gray = cv2.cvtColor(source_img, cv2.COLOR_BGR2GRAY)
    target_gray = cv2.cvtColor(target_img, cv2.COLOR_BGR2GRAY)

    source_depth = cv2.imread(os.path.join(source_dir, 'depth.png'), cv2.IMREAD_ANYDEPTH)
    target_depth = cv2.imread(os.path.join(target_dir, 'depth.png'), cv2.IMREAD_ANYDEPTH)

    sift = cv2.xfeatures2d.SIFT_create()
    kp1, des1 = sift.detectAndCompute(source_gray, None)
    kp2, des2 = sift.detectAndCompute(target_gray, None)

    # FLANN匹配参数
    FLANN_INDEX_KDTREE = 0
    indexParams = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    searchParams = dict(checks=50)
    flann = cv2.FlannBasedMatcher(indexParams, searchParams)
    matches = flann.knnMatch(des1, des2, k=2)
    if not os.path.exists(os.path.join(args.out_dir, source_dir)):
        os.makedirs(os.path.join(args.out_dir, source_dir))
    if not os.path.exists(os.path.join(args.out_dir, target_dir)):
        os.makedirs(os.path.join(args.out_dir, target_dir))
    sift1_f = open(os.path.join(os.path.join(args.out_dir, source_dir), 'sift.txt'), 'w')
    sift2_f = open(os.path.join(os.path.join(args.out_dir, target_dir), 'sift.txt'), 'w')
    count = 0
    xy1 = []
    xy2 = []
    matchesMask = [[0, 0] for i in range(len(matches))]
    for i, (m, n) in enumerate(matches):
        if m.distance < 0.8 * n.distance:
            matchesMask[i] = [1, 0]
            x1, y1 = kp1[m.queryIdx].pt
            x2, y2 = kp2[m.trainIdx].pt
            x1, y1 = int(x1), int(y1)
            x2, y2 = int(x2), int(y2)
            if [x1, y1] in xy1 or [x2, y2] in xy2 or source_depth[y1][x1] == 0 or target_depth[y2][x2] == 0:
                continue
            xy1.append([x1, y1])
            xy2.append([x2, y2])
            sift1_f.write('%d %d\n' % (x1, y1))
            sift2_f.write('%d %d\n' % (x2, y2))
            count += 1
    print("Total sift count:", count)
    sift1_f.close()
    sift2_f.close()
    if args.show_sift:
        draw_match(source_img, target_img, xy1, xy2)
        # drawParams = dict(matchColor=(0, 255, 0),
        #                   singlePointColor=(255, 0, 0),
        #                   matchesMask=matchesMask,
        #                   flags=0)
        # resultImage = cv2.drawMatchesKnn(source_img, kp1, target_img, kp2, matches, None, **drawParams)
        # cv2.imwrite('sift.png', resultImage)
        # cv2.namedWindow('Flann', cv2.WINDOW_NORMAL)
        # cv2.resizeWindow('Flann', 400, 600)
        # cv2.imshow('Flann', resultImage)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()