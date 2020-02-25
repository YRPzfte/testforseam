#include "common.h"

//根据视差图获得右视图拼接匹配点对
void get_mapping_pair(const vector<pair<Point2, Point2> >& source_left_mapping_pairs, const vector<pair<Point2, Point2> >& target_left_mapping_pairs,
                      vector<pair<Point2, Point2> >& source_mapping_pairs, vector<pair<Point2, Point2> >& target_mapping_pairs,
                      vector<pair<Point2, Point2> >& source_align_pairs, vector<pair<Point2, Point2> >& target_align_pairs,
                      float scale, const Mat& origin_source_depth, const Mat& origin_target_depth, const Mat& warping_source_depth, const Mat& warping_target_depth,
                      const Mat& source_left_img, const Mat& target_left_img, const Mat& source_right_img, Mat& target_right_img) {

    double baseline = 120 * scale;
    double f = 712;
    for (int i = 0 ; i < source_left_mapping_pairs.size() ; ++i) {
        Point2 originP = source_left_mapping_pairs[i].first;
        Point2 destP = source_left_mapping_pairs[i].second;
        Vec3b originC = getSubpix<uchar, 3>(source_left_img, originP);
        ushort originD = origin_source_depth.ptr<ushort>(int(originP.y))[int(originP.x)];
        ushort warpingD = warping_source_depth.ptr<ushort>(int(destP.y))[int(destP.x)];
        //视差过大的，可能为错误匹配点对，去除
        if (baseline * f / originD > baseline || baseline * f / warpingD > baseline) continue;
        if (originD == 0 || warpingD == 0) continue;
        //对应视差相差过大，可能为错误匹配点对，去除
        if (abs(baseline * f / originD - baseline * f / warpingD) > baseline / 4) continue;
        Point2 warping_originP;
        warping_originP.x = originP.x - baseline * f / originD;
        warping_originP.y = originP.y;

        Point2 warping_destP;
        warping_destP.x = destP.x - baseline * f / warpingD;
        warping_destP.y = destP.y;

        Vec3b destC = getSubpix<uchar, 3>(source_right_img, originP);
        //颜色相差过大的去除，可能为错误匹配点对
        if (abs(int(originC[0]) - int(destC[0])) > 2 || abs(int(originC[1]) - int(destC[1])) > 2 || abs(int(originC[2]) - int(destC[2])) > 2) continue;
        //超出边界的匹配点对去除
        if (warping_originP.x < 0 || warping_originP.x >= origin_source_depth.cols - 1|| warping_originP.y < 0 || warping_originP.y >= origin_source_depth.rows  - 1) continue;
        if (warping_destP.x < 0 || warping_destP.x >= warping_source_depth.cols - 1|| warping_destP.y < 0 || warping_destP.y >= warping_source_depth.rows  - 1) continue;
        source_mapping_pairs.push_back(make_pair(warping_originP, warping_destP));
        source_align_pairs.push_back(make_pair(warping_originP, destP));
    }
    for (int i = 0 ; i < target_left_mapping_pairs.size() ; ++i) {
        Point2 originP = target_left_mapping_pairs[i].first;
        Point2 destP = target_left_mapping_pairs[i].second;
        Vec3b originC = getSubpix<uchar, 3>(target_left_img, originP);
        ushort originD = origin_target_depth.ptr<ushort>(int(originP.y))[int(originP.x)];
        ushort warpingD = warping_target_depth.ptr<ushort>(int(destP.y))[int(destP.x)];
        if (baseline * f / originD > baseline || baseline * f / warpingD > baseline) continue;
        if (originD == 0 || warpingD == 0) continue;
        if (abs(baseline * f / originD - baseline * f / warpingD) > baseline / 4) continue;
        Point2 warping_originP;
        warping_originP.x = originP.x - baseline * f / originD;
        warping_originP.y = originP.y;

        Point2 warping_destP;
        warping_destP.x = destP.x - baseline * f / warpingD;
        warping_destP.y = destP.y;
        Vec3b destC = getSubpix<uchar, 3>(target_right_img, originP);
        if (abs(int(originC[0]) - int(destC[0])) > 2 || abs(int(originC[1]) - int(destC[1])) > 2 || abs(int(originC[2]) - int(destC[2])) > 2) continue;
        if (warping_originP.x < 0 || warping_originP.x >= origin_source_depth.cols - 1|| warping_originP.y < 0 || warping_originP.y >= origin_source_depth.rows  - 1) continue;
        if (warping_destP.x < 0 || warping_destP.x >= warping_source_depth.cols - 1|| warping_destP.y < 0 || warping_destP.y >= warping_source_depth.rows  - 1) continue;
        target_mapping_pairs.push_back(make_pair(warping_originP, warping_destP));
        target_align_pairs.push_back(make_pair(warping_originP, destP));
    }
}
