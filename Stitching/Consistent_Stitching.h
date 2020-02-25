

#ifndef __Consistent_Stitching__
#define __Consistent_Stitching__

#include "../Mesh/MeshOptimization.h"
#include "../CommonUtil/function.h"

class Consistent_Stitching : public MeshOptimization {
public:
    Consistent_Stitching(const MultiImages & _multi_images);
    Mat applyWarping(const BLENDING_METHODS & _blend_method, int overlap_width, int source_col, int target_col);
    void setWeightToAlignmentTerm(const double _weight);
    void setWeightToDisparityTerm(const double _weight);
    void setWeightToSimilarityTerm(const double _weight, const enum GLOBAL_ROTATION_METHODS _global_rotation_method);
    vector<vector<pair<Point2, Point2> > > mapping_pairs;
    vector<vector<pair<Point2, Point2> > > align_pairs;
    void set_mapping_pair(const vector<pair<Point2, Point2> >& source_mapping_pairs, const vector<pair<Point2, Point2> >& target_mapping_pairs);
    void set_align_pair(const vector<pair<Point2, Point2> >& source_align_pairs, const vector<pair<Point2, Point2> >& target_align_pairs);
    void set_boundary_pair(const vector<vector<pair<Point2, Point2> > >& _boundaryPairs);
    void prepareAlignmentTerm(vector<Triplet<double> > & _triplets, vector<pair<int, double> > & _b_vector) const;
    void prepareDisparityTerm(vector<Triplet<double> > & _triplets, vector<pair<int, double> > & _b_vector) const;
    void prepareSimilarityTerm(vector<Triplet<double> > & _triplets, vector<pair<int, double> > & _b_vector) const;
    Mat solve(const BLENDING_METHODS & _blend_method);
    void setSize(int _width, int _height);
    Size2 ready_mapping_size(vector<vector<Point2> >& vertices);
    vector<vector<pair<Point2, Point2> > > calulate_boundary_pair();
    void setPaths(const vector<int>& _paths);
    vector<vector<pair<Point2, Point2> > > getBounderPairs();
    vector<int> getMappingPaths();
    vector<int> paths;
private:
    int width, height;
    vector<vector<pair<Point2, Point2> > > boundaryPairs;
};

#endif
