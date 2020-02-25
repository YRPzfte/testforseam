#include "function.h"

vector<pair<int, int> > getInitialFeaturePairs(const vector<FeatureDescriptor>& feature_descriptors_1, const vector<FeatureDescriptor>& feature_descriptors_2) {
    const int nearest_size = 2, pair_count = 1;
    const bool ratio_test = true, intersect = true;
    assert(nearest_size > 0);

    const int feature_size_1 = (int)feature_descriptors_1.size();
    const int feature_size_2 = (int)feature_descriptors_2.size();
    const int PAIR_COUNT = 2;
    const int feature_size[PAIR_COUNT] = { feature_size_1, feature_size_2 };
    vector<FeatureDistance> feature_pairs[PAIR_COUNT];

    for(int p = 0; p < pair_count; ++p) {
        const int another_feature_size = feature_size[1 - p];
        const int nearest_k = min(nearest_size, another_feature_size);
        for(int f1 = 0; f1 < feature_size[p]; ++f1) {
            set<FeatureDistance> feature_distance_set;
            feature_distance_set.insert(FeatureDistance(MAXFLOAT, p, -1, -1));
            for(int f2 = 0; f2 < feature_size[!p]; ++f2) {
                const double dist = FeatureDescriptor::getDistance(feature_descriptors_1[f1], feature_descriptors_2[f2], feature_distance_set.begin()->distance);
                if(dist < feature_distance_set.begin()->distance) {
                    if(feature_distance_set.size() == nearest_k) {
                        feature_distance_set.erase(feature_distance_set.begin());
                    }
                    feature_distance_set.insert(FeatureDistance(dist, p, f1, f2));
                }
            }
            set<FeatureDistance>::const_iterator it = feature_distance_set.begin();
            if(ratio_test) {
                const set<FeatureDistance>::const_iterator it2 = std::next(it, 1);
                if(nearest_k == nearest_size &&
                   it2->distance * FEATURE_RATIO_TEST_THRESHOLD > it->distance) {
                    continue;
                }
                it = it2;
            }
            feature_pairs[p].insert(feature_pairs[p].end(), it, feature_distance_set.end());
        }
    }
    vector<FeatureDistance> feature_pairs_result;
    if(pair_count == PAIR_COUNT) {
        sort(feature_pairs[0].begin(), feature_pairs[0].end(), compareFeaturePair);
        sort(feature_pairs[1].begin(), feature_pairs[1].end(), compareFeaturePair);
        if(intersect) {
            set_intersection(feature_pairs[0].begin(), feature_pairs[0].end(),
                             feature_pairs[1].begin(), feature_pairs[1].end(),
                             std::inserter(feature_pairs_result, feature_pairs_result.begin()),
                             compareFeaturePair);
        } else {
            set_union(feature_pairs[0].begin(), feature_pairs[0].end(),
                      feature_pairs[1].begin(), feature_pairs[1].end(),
                      std::inserter(feature_pairs_result, feature_pairs_result.begin()),
                      compareFeaturePair);
        }
    } else {
        feature_pairs_result = std::move(feature_pairs[0]);
    }
    vector<double> distances;
    distances.reserve(feature_pairs_result.size());
    for(int i = 0; i < feature_pairs_result.size(); ++i) {
        distances.emplace_back(feature_pairs_result[i].distance);
    }
    double mean, std;
    Statistics::getMeanAndSTD(distances, mean, std);

    const double OUTLIER_THRESHOLD = (INLIER_TOLERANT_STD_DISTANCE * std) + mean;
    vector<pair<int, int> > initial_indices;
    initial_indices.reserve(feature_pairs_result.size());
    for(int i = 0; i < feature_pairs_result.size(); ++i) {
        if(feature_pairs_result[i].distance < OUTLIER_THRESHOLD) {
            initial_indices.emplace_back(feature_pairs_result[i].feature_index[0],
                                         feature_pairs_result[i].feature_index[1]);
        }
    }
    return initial_indices;
}