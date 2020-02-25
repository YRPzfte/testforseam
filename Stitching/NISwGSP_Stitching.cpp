//
//  NISwGSP_Stitching.cpp
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#include "NISwGSP_Stitching.h"

NISwGSP_Stitching::NISwGSP_Stitching(const MultiImages & _multi_images) : MeshOptimization(_multi_images) {
    
}

void NISwGSP_Stitching::setWeightToAlignmentTerm(const double _weight) {
    MeshOptimization::setWeightToAlignmentTerm(_weight);
}

void NISwGSP_Stitching::setWeightToLocalSimilarityTerm(const double _weight) {
    MeshOptimization::setWeightToLocalSimilarityTerm(_weight);
}

void NISwGSP_Stitching::setWeightToGlobalSimilarityTerm(const double _weight_beta,
                                                        const double _weight_gamma,
                                                        const enum GLOBAL_ROTATION_METHODS _global_rotation_method) {
    MeshOptimization::setWeightToGlobalSimilarityTerm(_weight_beta, _weight_gamma, _global_rotation_method);
}


Mat NISwGSP_Stitching::solve(const BLENDING_METHODS & _blend_method) {
    const MultiImages & multi_images = getMultiImages();
    
    vector<Triplet<double> > triplets;
    vector<pair<int, double> > b_vector;
    
    reserveData(triplets, b_vector, DIMENSION_2D);
    
    triplets.emplace_back(0, 0, STRONG_CONSTRAINT);
    triplets.emplace_back(1, 1, STRONG_CONSTRAINT);
    b_vector.emplace_back(0,    STRONG_CONSTRAINT);
    b_vector.emplace_back(1,    STRONG_CONSTRAINT);
    prepareAlignmentTerm(triplets);
    prepareSimilarityTerm(triplets, b_vector);
    vector<vector<Point2> > original_vertices;


    original_vertices = getImageVerticesBySolving(triplets, b_vector);
    Size2 target_size = normalizeVertices(original_vertices);

    boundaryPairs.clear();
    boundaryPairs.resize(multi_images.images_data.size());
    for (int i = 0 ; i < boundaryPairs.size() ; ++i) {
        vector<Point2> grid_vertices = multi_images.images_data[i].mesh_2d->getVertices();
        vector<int> boundaryIndexs= multi_images.images_data[i].mesh_2d->getBoundaryVertexIndices();
        for (int j = 0 ; j < boundaryIndexs.size() ; ++j) {
            boundaryPairs[i].push_back(make_pair(grid_vertices[boundaryIndexs[j]], original_vertices[i][boundaryIndexs[j]]));
        }
    }
    multi_images.textureMapping(original_vertices, target_size, _blend_method);
    paths.clear();
    findBestLine(multi_images.source_warping, multi_images.target_warping, paths);
    Mat stitching_image = stitchingByBestLine(paths, multi_images.source_warping, multi_images.target_warping);
    stitching_disparity_by_poisson_blending(multi_images.source_depth, multi_images.target_depth, paths);
    return stitching_image;
}

vector<int> NISwGSP_Stitching::getMappingPaths() {
    if (paths.size() == 0) {
        std::cout << "Should Stitching First!" << std::endl;
    }
    const MultiImages & multi_images = getMultiImages();
    assert(paths.size() == multi_images.source_warping.rows);
    vector<int> mapping_paths;
    double baseline = 120 * multi_images.scale;
    double f = 712;
    int row = -1;
    ushort warpingD;
    do {
        row ++;
        if (row >= paths.size()) break;
        warpingD = multi_images.source_depth.ptr<ushort>(row)[paths[row]];
    } while (warpingD == 0 || paths[row] - baseline * f / warpingD < 0);
    assert(row < paths.size());
    for (int i = 0 ; i <= row ; ++i) {
        mapping_paths.push_back(paths[row] - baseline * f / multi_images.source_depth.ptr<ushort>(row)[paths[row]]);
    }
    int prevCol = mapping_paths[mapping_paths.size() - 1];
    int mapping_col;
    row ++;
    for ( ; row < paths.size() ; ++row) {
        ushort warpingD = multi_images.source_depth.ptr<ushort>(row)[paths[row]];
        //防止出现没有深度值对应
        if (warpingD == 0) {
            mapping_col = prevCol;
        } else {
            mapping_col = paths[row] - baseline * f / warpingD;
            if (abs(mapping_col - prevCol) > 2) {
                if (mapping_col < prevCol) {
                   mapping_col = prevCol - 1;
                } else {
                   mapping_col = prevCol + 1;
                }
            }
            prevCol = mapping_col;
        }
        mapping_paths.push_back(mapping_col);
    }
    return mapping_paths;
}

vector<vector<pair<Point2, Point2> > > NISwGSP_Stitching::getBounderPairs() {
    if (boundaryPairs.size() == 0) {
        std::cout << "Should Stitching First!" << std::endl;
    }
    return boundaryPairs;
}
void NISwGSP_Stitching::writeImage(const Mat & _image, const string _post_name) const {
    const MultiImages & multi_images = getMultiImages();
    const Parameter & parameter = multi_images.parameter;
    string file_name = parameter.file_name;
    
    imwrite(parameter.result_dir + file_name + "-" +
            "[NISwGSP]" +
            GLOBAL_ROTATION_METHODS_NAME[getGlobalRotationMethod()] +
            _post_name +
            ".png", _image);
}