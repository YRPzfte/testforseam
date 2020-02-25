#include "Consistent_Stitching.h"

Consistent_Stitching::Consistent_Stitching(const MultiImages & _multi_images) : MeshOptimization(_multi_images) {
    
}

void Consistent_Stitching::setWeightToAlignmentTerm(const double _weight) {
    MeshOptimization::setWeightToAlignmentTerm(_weight);
}

void Consistent_Stitching::setWeightToDisparityTerm(const double _weight) {
    MeshOptimization::setWeightToDisparityTerm(_weight);
}

void Consistent_Stitching::setWeightToSimilarityTerm(const double _weight, const enum GLOBAL_ROTATION_METHODS _global_rotation_method) {
    MeshOptimization::setWeightToSimilarityTerm(_weight, _global_rotation_method);
}

void Consistent_Stitching::set_mapping_pair(const vector<pair<Point2, Point2> >& source_mapping_pairs, const vector<pair<Point2, Point2> >& target_mapping_pairs) {
    //通过RANSAC过滤部分匹配点对
    const MultiImages & multi_images = getMultiImages();
    mapping_pairs.resize(multi_images.images_data.size());
    vector<Point2> source_origin_points, source_dest_points;
    for (int i = 0 ; i < source_mapping_pairs.size() ; ++i) {
        source_origin_points.push_back(source_mapping_pairs[i].first);
        source_dest_points.push_back(source_mapping_pairs[i].second);
    }
    vector<uchar> source_RansacStatus;
    vector<Point2> filter_source_origin_points, filter_source_dest_points;
    findFundamentalMat(source_origin_points, source_dest_points, source_RansacStatus, FM_RANSAC);
    for (int i = 0 ; i < source_RansacStatus.size() ; ++i) {
        if (source_RansacStatus[i]) {
            mapping_pairs[0].push_back(make_pair(source_origin_points[i], source_dest_points[i]));
        }
    }

    vector<Point2> target_origin_points, target_dest_points;
    for (int i = 0 ; i < target_mapping_pairs.size() ; ++i) {
        target_origin_points.push_back(target_mapping_pairs[i].first);
        target_dest_points.push_back(target_mapping_pairs[i].second);
    }
    vector<uchar> target_RansacStatus;
    vector<Point2> filter_target_origin_points, filter_target_dest_points;
    findFundamentalMat(target_origin_points, target_dest_points, target_RansacStatus, FM_RANSAC);
    for (int i = 0 ; i < target_RansacStatus.size() ; ++i) {
        if (target_RansacStatus[i]) {
            mapping_pairs[1].push_back(make_pair(target_origin_points[i], target_dest_points[i]));
        }
    }

}

void Consistent_Stitching::set_align_pair(const vector<pair<Point2, Point2> >& source_align_pairs, const vector<pair<Point2, Point2> >& target_align_pairs) {
    //视差约束项匹配点
    align_pairs.push_back(source_align_pairs);
    align_pairs.push_back(target_align_pairs);
}

vector<vector<pair<Point2, Point2> > > Consistent_Stitching::calulate_boundary_pair() {
    const MultiImages & multi_images = getMultiImages();
    //边界匹配点
    if (mapping_pairs.size() == 0) {
        std::cout << "Should set Mapping pair before !";
    }
    assert(mapping_pairs.size() == multi_images.images_data.size());

    vector<Point2> source_origin_points, source_dest_points;
    for (int i = 0 ; i < mapping_pairs[0].size() ; ++i) {
        source_origin_points.push_back(mapping_pairs[0][i].first);
        source_dest_points.push_back(mapping_pairs[0][i].second);
    }
    vector<Mat>   source_apap_homographies;
    vector<Point2> source_apap_matching_points;
    APAP_Stitching::apap_project(source_origin_points,
                                     source_dest_points,
                                     multi_images.images_data[0].mesh_2d->getVertices(), source_apap_matching_points, source_apap_homographies);


    vector<Point2> target_origin_points, target_dest_points;
    for (int i = 0 ; i < mapping_pairs[1].size() ; ++i) {
        target_origin_points.push_back(mapping_pairs[1][i].first);
        target_dest_points.push_back(mapping_pairs[1][i].second);
    }
    vector<Mat>   target_apap_homographies;
    vector<Point2> target_apap_matching_points;
    APAP_Stitching::apap_project(target_origin_points,
                                     target_dest_points,
                                     multi_images.images_data[1].mesh_2d->getVertices(), target_apap_matching_points, target_apap_homographies);

    vector<vector<pair<Point2, Point2> > > boundary_pairs;
    boundary_pairs.resize(multi_images.images_data.size());

    vector<Point2> source_grid_vertices = multi_images.images_data[0].mesh_2d->getVertices();
    vector<int> source_boundaryIndexs= multi_images.images_data[0].mesh_2d->getBoundaryVertexIndices();
    for (int i = 0 ; i < source_boundaryIndexs.size() ; ++i) {
        boundary_pairs[0].push_back(make_pair(source_grid_vertices[source_boundaryIndexs[i]], source_apap_matching_points[source_boundaryIndexs[i]]));
    }
    vector<Point2> target_grid_vertices = multi_images.images_data[1].mesh_2d->getVertices();
    vector<int> target_boundaryIndexs= multi_images.images_data[1].mesh_2d->getBoundaryVertexIndices();
    for (int i = 0 ; i < target_boundaryIndexs.size() ; ++i) {
        boundary_pairs[1].push_back(make_pair(target_grid_vertices[target_boundaryIndexs[i]], target_apap_matching_points[target_boundaryIndexs[i]]));
    }
    return boundary_pairs;
}

void Consistent_Stitching::set_boundary_pair(const vector<vector<pair<Point2, Point2> > >& boundary_pairs) {
    //边界匹配点
    if (mapping_pairs.size() == 0) {
        std::cout << "Should set Mapping pair before !";
    }
    assert(mapping_pairs.size() == boundary_pairs.size());
    for (int i = 0 ; i < boundary_pairs.size() ; ++i) {
        for (int j = 0 ; j < boundary_pairs[i].size() ; ++j) {
            mapping_pairs[i].push_back(boundary_pairs[i][j]);
        }
    }
}


void Consistent_Stitching::setSize(int _width, int _height) {
    width = _width;
    height = _height;
}

Mat Consistent_Stitching::solve(const BLENDING_METHODS & _blend_method) {
    const MultiImages & multi_images = getMultiImages();
    vector<Triplet<double> > triplets;
    vector<pair<int, double> > b_vector;
    int align_equation = 0;
    for (int i = 0 ; i < mapping_pairs.size() ; ++i) {
        align_equation += mapping_pairs[i].size();
    }
    int disparity_equation = 0;
    for (int i = 0 ; i < align_pairs.size() ; ++i) {
        disparity_equation += align_pairs[i].size();
    }
    reserveDataForConsistent(triplets, b_vector, 0, align_equation, disparity_equation);
    prepareAlignmentTerm(triplets, b_vector);
    prepareDisparityTerm(triplets, b_vector);
    prepareSimilarityTerm(triplets, b_vector);
    vector<vector<Point2> > original_vertices;
    original_vertices = getImageVerticesBySolving(triplets, b_vector);
    Size2 target_size = normalizeVertices(original_vertices);
    multi_images.textureMapping(original_vertices, target_size, _blend_method);
    if (paths.size() == 0) {
        findBestLine(multi_images.source_warping, multi_images.target_warping, paths);

    }
    return stitchingByBestLine(paths, multi_images.source_warping, multi_images.target_warping);
}

Size2 Consistent_Stitching::ready_mapping_size(vector<vector<Point2> >& vertices) {
    for(int i = 0; i < vertices.size(); ++i) {
        for(int j = 0; j < vertices[i].size(); ++j) {
            if (vertices[i][j].x < 0) vertices[i][j].x = 0;
            if (vertices[i][j].x >= width) vertices[i][j].x = width - 1;
            if (vertices[i][j].y < 0) vertices[i][j].y = 0;
            if (vertices[i][j].y >= height) vertices[i][j].y = height - 1;
        }
    }
    return Size2(width, height);
}

vector<int> Consistent_Stitching::getMappingPaths() {
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

Mat Consistent_Stitching::applyWarping(const BLENDING_METHODS & _blend_method, int overlap_width, int source_col, int target_col) {
    const MultiImages & multi_images = getMultiImages();
    vector<Triplet<double> > triplets;
    vector<pair<int, double> > b_vector;
    int align_equation = 0;
    for (int i = 0 ; i < mapping_pairs.size() ; ++i) {
        align_equation += mapping_pairs[i].size();
    }
    int disparity_equation = 0;
    for (int i = 0 ; i < align_pairs.size() ; ++i) {
        disparity_equation += align_pairs[i].size();
    }
    reserveDataForConsistent(triplets, b_vector, 0, align_equation, disparity_equation);
    prepareAlignmentTerm(triplets, b_vector);
    prepareDisparityTerm(triplets, b_vector);
    prepareSimilarityTerm(triplets, b_vector);
    vector<vector<Point2> > original_vertices;
    original_vertices = getImageVerticesBySolving(triplets, b_vector);
    Size2 target_size = ready_mapping_size(original_vertices);
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
    return blending_images(multi_images.source_warping, multi_images.target_warping, overlap_width, source_col, target_col, paths);
}

void Consistent_Stitching::setPaths(const vector<int>& _paths) {
    for (int i = 0 ; i < _paths.size() ; ++i) {
        paths.push_back(_paths[i]);
    }
}

vector<vector<pair<Point2, Point2> > > Consistent_Stitching::getBounderPairs() {
    if (boundaryPairs.size() == 0) {
        std::cout << "Should Stitching First!" << std::endl;
    }
    return boundaryPairs;
}

//对齐项
void Consistent_Stitching::prepareAlignmentTerm(vector<Triplet<double> > & _triplets, vector<pair<int, double> > & _b_vector) const{
    const MultiImages & multi_images = getMultiImages();
    if(alignment_equation.second) {
        const int equation = alignment_equation.first;

        const vector<Indices> & polygons_indices_1 = multi_images.images_data[0].mesh_2d->getPolygonsIndices();
        const vector<Indices> & polygons_indices_2 = multi_images.images_data[1].mesh_2d->getPolygonsIndices();

        vector<vector<InterpolateVertex> > mesh_interpolate_vertex_of_matching_pts;
        mesh_interpolate_vertex_of_matching_pts.resize(multi_images.images_data.size());
        for (int i = 0 ; i < mapping_pairs.size() ; ++i) {
            mesh_interpolate_vertex_of_matching_pts[i].reserve(mapping_pairs[i].size());
            for (int j = 0 ; j < mapping_pairs[i].size() ; ++j) {
                mesh_interpolate_vertex_of_matching_pts[i].emplace_back(multi_images.images_data[i].mesh_2d->getInterpolateVertex(mapping_pairs[i][j].first));
            }
        }
        const vector<int> & images_vertices_start_index = multi_images.getImagesVerticesStartIndex();
        int eq_count = 0;
        for (int i = 0 ; i < mapping_pairs.size() ; ++i) {
            for (int j = 0 ; j < mapping_pairs[i].size() ; ++j) {
                for(int dim = 0; dim < DIMENSION_2D; ++dim) {
                    for(int k = 0; k < multi_images.images_data[i].mesh_2d->getPolygonVerticesCount(); ++k) {
                        _triplets.emplace_back(equation + eq_count + dim,
                                               images_vertices_start_index[i] + dim +
                                               DIMENSION_2D * (polygons_indices_1[mesh_interpolate_vertex_of_matching_pts[i][j].polygon].indices[k]),
                                                alignment_weight * mesh_interpolate_vertex_of_matching_pts[i][j].weights[k]);
                    }
                }
                _b_vector.emplace_back(equation + eq_count + 0, mapping_pairs[i][j].second.x);
                _b_vector.emplace_back(equation + eq_count + 1, mapping_pairs[i][j].second.y);
                eq_count += DIMENSION_2D;
            }
        }
        assert(eq_count == alignment_equation.second);
    }
}

//视差项
void Consistent_Stitching::prepareDisparityTerm(vector<Triplet<double> > & _triplets, vector<pair<int, double> > & _b_vector) const{
    const MultiImages & multi_images = getMultiImages();
    if(disparity_equation.second) {
        const int equation = disparity_equation.first;

        const vector<Indices> & polygons_indices_1 = multi_images.images_data[0].mesh_2d->getPolygonsIndices();
        const vector<Indices> & polygons_indices_2 = multi_images.images_data[1].mesh_2d->getPolygonsIndices();

        vector<vector<InterpolateVertex> > mesh_interpolate_vertex_of_matching_pts;
        mesh_interpolate_vertex_of_matching_pts.resize(multi_images.images_data.size());
        for (int i = 0 ; i < align_pairs.size() ; ++i) {
            mesh_interpolate_vertex_of_matching_pts[i].reserve(align_pairs[i].size());
            for (int j = 0 ; j < align_pairs[i].size() ; ++j) {
                mesh_interpolate_vertex_of_matching_pts[i].emplace_back(multi_images.images_data[i].mesh_2d->getInterpolateVertex(align_pairs[i][j].first));
            }
        }
        const vector<int> & images_vertices_start_index = multi_images.getImagesVerticesStartIndex();
        int eq_count = 0;
        for (int i = 0 ; i < align_pairs.size() ; ++i) {
            for (int j = 0 ; j < align_pairs[i].size() ; ++j) {
                int dim = 1;//保证Y坐标相同
                for(int k = 0; k < multi_images.images_data[i].mesh_2d->getPolygonVerticesCount(); ++k) {
                    _triplets.emplace_back(equation + eq_count + 0,
                                           images_vertices_start_index[i] + dim +
                                           DIMENSION_2D * (polygons_indices_1[mesh_interpolate_vertex_of_matching_pts[i][j].polygon].indices[k]),
                                            disparity_weight * mesh_interpolate_vertex_of_matching_pts[i][j].weights[k]);
                }
                _b_vector.emplace_back(equation + eq_count + 0, align_pairs[i][j].second.y);
                eq_count += 1;
            }
        }
        assert(eq_count == disparity_equation.second);
    }
}

//自然项
void Consistent_Stitching::prepareSimilarityTerm(vector<Triplet<double> > & _triplets,
                                             vector<pair<int, double> > & _b_vector) const {
    const MultiImages & multi_images = getMultiImages();
    const bool local_similarity_term = local_similarity_equation.second;
    if(local_similarity_term) {
        const vector<int> & images_vertices_start_index = multi_images.getImagesVerticesStartIndex();
        const vector<SimilarityElements> & images_similarity_elements = multi_images.getImagesSimilarityElements(global_rotation_method);
        int eq_count = 0;
        for(int i = 0; i < multi_images.images_data.size(); ++i) {
            const vector<Edge> & edges = multi_images.images_data[i].mesh_2d->getEdges();
            const vector<Point2> & vertices = multi_images.images_data[i].mesh_2d->getVertices();
            const vector<Indices> & v_neighbors = multi_images.images_data[i].mesh_2d->getVertexStructures();
            const vector<Indices> & e_neighbors = multi_images.images_data[i].mesh_2d->getEdgeStructures();

            const double similarity[DIMENSION_2D] = {
                images_similarity_elements[i].scale * cos(images_similarity_elements[i].theta),
                images_similarity_elements[i].scale * sin(images_similarity_elements[i].theta)
            };

            for(int j = 0; j < edges.size(); ++j) {
                const int & ind_e1 = edges[j].indices[0];
                const int & ind_e2 = edges[j].indices[1];
                const Point2 & src = multi_images.images_data[i].mesh_2d->getVertices()[ind_e1];
                const Point2 & dst = multi_images.images_data[i].mesh_2d->getVertices()[ind_e2];
                set<int> point_ind_set;
                for(int e = 0; e < EDGE_VERTEX_SIZE; ++e) {
                    for(int v = 0; v < v_neighbors[edges[j].indices[e]].indices.size(); ++v) {
                        int v_index = v_neighbors[edges[j].indices[e]].indices[v];
                        if(v_index != ind_e1) {
                            point_ind_set.insert(v_index);
                        }
                    }
                }
                Mat Et, E_Main(DIMENSION_2D, DIMENSION_2D, CV_64FC1), E((int)point_ind_set.size() * DIMENSION_2D, DIMENSION_2D, CV_64FC1);
                set<int>::const_iterator it = point_ind_set.begin();
                for(int p = 0; it != point_ind_set.end(); ++p, ++it) {
                    Point2 e = vertices[*it] - src;
                    E.at<double>(DIMENSION_2D * p    , 0) =  e.x;
                    E.at<double>(DIMENSION_2D * p    , 1) =  e.y;
                    E.at<double>(DIMENSION_2D * p + 1, 0) =  e.y;
                    E.at<double>(DIMENSION_2D * p + 1, 1) = -e.x;
                }
                transpose(E, Et);
                Point2 e_main = dst - src;
                E_Main.at<double>(0, 0) =  e_main.x;
                E_Main.at<double>(0, 1) =  e_main.y;
                E_Main.at<double>(1, 0) =  e_main.y;
                E_Main.at<double>(1, 1) = -e_main.x;

                Mat G_W = (Et * E).inv(DECOMP_SVD) * Et;
                Mat L_W = - E_Main * G_W;

                double _local_similarity_weight = 1;
                it = point_ind_set.begin();
                for(int p = 0; it != point_ind_set.end(); ++p, ++it) {
                    for(int xy = 0; xy < DIMENSION_2D; ++xy) {
                        for(int dim = 0; dim < DIMENSION_2D; ++dim) {
                            if(local_similarity_term) {
                                _triplets.emplace_back(local_similarity_equation.first + eq_count + dim,
                                                       images_vertices_start_index[i]  + DIMENSION_2D * (*it) + xy,
                                                       _local_similarity_weight *
                                                       local_similarity_weight * L_W.at<double>(dim, DIMENSION_2D * p + xy));
                                _triplets.emplace_back(local_similarity_equation.first + eq_count + dim,
                                                       images_vertices_start_index[i]  + DIMENSION_2D * ind_e1 + xy,
                                                       _local_similarity_weight *
                                                      -local_similarity_weight * L_W.at<double>(dim, DIMENSION_2D * p + xy));
                            }
                        }
                    }
                }
                if(local_similarity_term) {
                    _triplets.emplace_back(local_similarity_equation.first + eq_count    , images_vertices_start_index[i] + DIMENSION_2D * ind_e2    ,
                                           _local_similarity_weight *  local_similarity_weight);
                    _triplets.emplace_back(local_similarity_equation.first + eq_count + 1, images_vertices_start_index[i] + DIMENSION_2D * ind_e2 + 1,
                                           _local_similarity_weight *  local_similarity_weight);
                    _triplets.emplace_back(local_similarity_equation.first + eq_count    , images_vertices_start_index[i] + DIMENSION_2D * ind_e1    ,
                                           _local_similarity_weight * -local_similarity_weight);
                    _triplets.emplace_back(local_similarity_equation.first + eq_count + 1, images_vertices_start_index[i] + DIMENSION_2D * ind_e1 + 1,
                                           _local_similarity_weight * -local_similarity_weight);
                }
                eq_count += DIMENSION_2D;
            }
        }
        assert(eq_count ==  local_similarity_equation.second);
    }
}