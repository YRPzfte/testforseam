//
//  MultiImages.h
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#ifndef __UglyMan_Stitiching__MultiImages__
#define __UglyMan_Stitiching__MultiImages__

#include <queue>

#include "../Configure.h"
#include "../Stitching/Parameter.h"
#include "../Feature/ImageData.h"
#include "../Util/Statistics.h"
#include "../Debugger/ImageDebugger.h"
#include "../Stitching/APAP_Stitching.h"
#include "../Util/Blending.h"
#include "../Debugger/ColorMap.h"

#include <opencv2/calib3d.hpp> /* CV_RANSAC */
#include <opencv2/stitching/detail/autocalib.hpp> /* ImageFeatures, MatchesInfo */
#include <opencv2/stitching/detail/camera.hpp> /* CameraParams */
#include <opencv2/stitching/detail/motion_estimators.hpp> /* BundleAdjusterBase */

const int PAIR_COUNT = 2;

class FeatureDistance {
public:
    double distance;
    int feature_index[PAIR_COUNT];
    FeatureDistance() {
        feature_index[0] = feature_index[1] = -1;
        distance = MAXFLOAT;
    }
    FeatureDistance(const double _distance,
                    const int _p_1,
                    const int _feature_index_1,
                    const int _feature_index_2) {
        distance = _distance;
        feature_index[ _p_1] = _feature_index_1;
        feature_index[!_p_1] = _feature_index_2;
    }
    bool operator < (const FeatureDistance & fd) const {
        return distance > fd.distance;
    }
private:
};

class SimilarityElements {
public:
    double scale;
    double theta;
    SimilarityElements(const double _scale,
                       const double _theta) {
        scale = _scale;
        theta = _theta;
    }
private:
};

class MultiImages {
public:
    MultiImages(const string & _file_name,
                LINES_FILTER_FUNC * _width_filter  = &LINES_FILTER_NONE,
                LINES_FILTER_FUNC * _length_filter = &LINES_FILTER_NONE);

    MultiImages(const string& source_dir, const string& target_dir, const string& view,
                         LINES_FILTER_FUNC * _width_filter  = &LINES_FILTER_NONE,
                         LINES_FILTER_FUNC * _length_filter = &LINES_FILTER_NONE);
    const vector<cv::detail::ImageFeatures> & getImagesFeaturesByMatchingPoints() const;
    const vector<cv::detail::MatchesInfo>   & getPairwiseMatchesByMatchingPoints() const;
    const vector<cv::detail::CameraParams>  & getCameraParams() const;
    
    const vector<vector<bool> > & getImagesFeaturesMaskByMatchingPoints() const;
    
    const vector<vector<vector<pair<int, int> > > > & getFeaturePairs() const;
    const vector<vector<vector<Point2> > >          & getFeatureMatches() const;
    const vector<pair<Point2, Point2> >& getFeaturePairMatches() const;
    const vector<vector<vector<bool> > >   & getAPAPOverlapMask() const;
    const vector<vector<vector<Mat> > >    & getAPAPHomographies() const;
    const vector<vector<vector<Point2> > > & getAPAPMatchingPoints() const;
    
    const vector<vector<InterpolateVertex> > & getInterpolateVerticesOfMatchingPoints() const;
    
    const vector<int> & getImagesVerticesStartIndex() const;
    const vector<SimilarityElements> & getImagesSimilarityElements(const enum GLOBAL_ROTATION_METHODS & _global_rotation_method) const;
    const vector<vector<pair<double, double> > > & getImagesRelativeRotationRange() const;
    
    const vector<vector<double> > & getImagesGridSpaceMatchingPointsWeight(const double _global_weight_gamma) const;

    const vector<Point2> & getImagesLinesProject(const int _from, const int _to) const;
    
    const vector<Mat> & getImages() const;
    
    FLOAT_TYPE getImagesMinimumLineDistortionRotation(const int _from, const int _to) const;
    
    void textureMapping(const vector<vector<Point2> > & _vertices,
                       const Size2 & _target_size,
                       const BLENDING_METHODS & _blend_method) const;
    
    void textureMapping(const vector<vector<Point2> > & _vertices,
                       const Size2 & _target_size,
                       const BLENDING_METHODS & _blend_method,
                       vector<Mat> & _warp_images) const;
    
    void writeResultWithMesh(const Mat & _result,
                             const vector<vector<Point2> > & _vertices,
                             const string & _postfix,
                             const bool _only_border) const;
    void update_images_data(string source_path, string target_path, LINES_FILTER_FUNC * _width_filter, LINES_FILTER_FUNC * _length_filter);
    vector<ImageData> images_data;
    vector<Mat> depth_images;
    Parameter parameter;
    mutable vector<vector<pair<Point2, Point2> > > mapping_pairs;
    mutable Mat source_warping, target_warping;
    mutable Mat source_depth, target_depth;
    float scale;
private:    
    /*** Debugger ***/
    void writeImageOfFeaturePairs(const string & _name,
                                  const pair<int, int> & _index_pair,
                                  const vector<pair<int, int> > & _pairs) const;
    /****************/
    
    void doFeatureMatching() const;
    void initialFeaturePairsSpace() const;
    
    vector<pair<int, int> > getInitialFeaturePairs(const pair<int, int> & _match_pair) const;
    
    vector<pair<int, int> > getFeaturePairsBySequentialRANSAC(const pair<int, int> & _match_pair,
                                                              const vector<Point2> & _X,
                                                              const vector<Point2> & _Y,
                                                              const vector<pair<int, int> > & _initial_indices) const;
    mutable vector<cv::detail::ImageFeatures> images_features;
    mutable vector<cv::detail::MatchesInfo>   pairwise_matches;
    mutable vector<cv::detail::CameraParams>  camera_params;
    
    mutable vector<vector<bool> > images_features_mask;
    
    mutable vector<vector<vector<pair<int, int> > > > feature_pairs;
    mutable vector<vector<vector<Point2> > > feature_matches; /* [m1][m2][j], img1 j_th matches */
    mutable vector<pair<Point2, Point2> > feature_pair_matches;
    mutable vector<vector<vector<bool> > >   apap_overlap_mask;
    mutable vector<vector<vector<Mat> > >    apap_homographies;
    mutable vector<vector<vector<Point2> > > apap_matching_points;
    
    mutable vector<vector<InterpolateVertex> > mesh_interpolate_vertex_of_feature_pts;
    mutable vector<vector<InterpolateVertex> > mesh_interpolate_vertex_of_matching_pts;
    
    mutable vector<int> images_vertices_start_index;
    mutable vector<SimilarityElements> images_similarity_elements_2D;
    mutable vector<SimilarityElements> images_similarity_elements_3D;
    mutable vector<vector<pair<double, double> > > images_relative_rotation_range;
    
    mutable vector<vector<double> > images_polygon_space_matching_pts_weight;
    
    /* Line */
    mutable vector<vector<FLOAT_TYPE> > images_minimum_line_distortion_rotation;
    mutable vector<vector<vector<Point2> > > images_lines_projects; /* [m1][m2] img1 lines project on img2 */
    
    mutable vector<Mat> images;
};

bool compareFeaturePair(const FeatureDistance & fd_1, const FeatureDistance & fd_2);
#endif /* defined(__UglyMan_Stitiching__MultiImages__) */
