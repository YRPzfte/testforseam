#include "function.h"

void cal_overlap_edge(const Mat& source_img, const Mat& target_img, int& start_col, int& end_col) {
    uchar b, g, r;
    assert(source_img.rows == target_img.rows && source_img.cols == target_img.cols);
    end_col = source_img.cols - 1;
    while(start_col >= 0) {
        bool find = false;
        for (int row = 0 ; row < source_img.rows ; ++row) {
            b = source_img.ptr<uchar>(row)[end_col * 3];
			g = source_img.ptr<uchar>(row)[end_col * 3 + 1];
			r = source_img.ptr<uchar>(row)[end_col * 3 + 2];
			if (!(b == 0 && g == 0 && r == 0)) {
			    find = true;
			    break;
			}
        }
        if (find) break;
        end_col --;
    }
    start_col = 0;
    while (end_col < target_img.cols) {
        bool find = false;
        for (int row = 0 ; row < target_img.rows ; ++row) {
            b = target_img.ptr<uchar>(row)[start_col * 3];
            g = target_img.ptr<uchar>(row)[start_col * 3 + 1];
            r = target_img.ptr<uchar>(row)[start_col * 3 + 2];
            if (!(b == 0 && g == 0 && r == 0)) {
                find = true;
                break;
            }
        }
        if (find) break;
        start_col ++;
    }
}

double color_distance(const Mat& source_img, const Mat& target_img, int row, int col) {
    int source_b = source_img.ptr<uchar>(row)[col * 3];
    int source_g = source_img.ptr<uchar>(row)[col * 3 + 1];
    int source_r = source_img.ptr<uchar>(row)[col * 3 + 2];

    int target_b = target_img.ptr<uchar>(row)[col * 3];
    int target_g = target_img.ptr<uchar>(row)[col * 3 + 1];
    int target_r = target_img.ptr<uchar>(row)[col * 3 + 2];
    return sqrt((source_b - target_b) * (source_b - target_b) + (source_g - target_g) * (source_g - target_g) + (source_r - target_r) * (source_r - target_r));

}

vector<vector<double> > init_vector(int rows, int cols) {
    vector<vector<double> > result;
    for (int i = 0 ; i < rows ; ++i) {
        vector<double> vector_elem;
        for (int j = 0 ; j < cols ; ++j) {
            vector_elem.push_back(0);
        }
        result.push_back(vector_elem);
    }
    return result;
}

int getColor(const Mat& img, int row, int col, int pipe=0) {
    int value = img.ptr<uchar>(row)[col * 3 + pipe];
    return value;
}

void findBestLine(const Mat& source_img, const Mat& target_img, vector<int>& paths, int start_col, int end_col, int start_row, int end_row, int last_col) {
    if (start_row == 0 && end_row == 0) {
        start_row = 0;
        end_row = source_img.rows;
    }
    if (start_col == 0 && end_col == 0) {
        cal_overlap_edge(source_img, target_img, start_col, end_col);
    }

    vector<vector<double> > colors = init_vector(end_row - start_row, end_col - start_col);
    vector<vector<double> > geometrys = init_vector(end_row - start_row, end_col - start_col);
    vector<vector<double> > strengths = init_vector(end_row - start_row, end_col - start_col);
    vector<vector<double> > strengthPath = init_vector(end_row - start_row, end_col - start_col);

    //计算颜色差异
    for (int row = start_row ; row < end_row ; ++row) {
        for (int col = start_col ; col < end_col ; ++col) {
            colors[row - start_row][col - start_col] = color_distance(source_img, target_img, row, col);
        }
    }
    //计算几何差异
    double AxdaoB, AydaoB, BxdaoB, BydaoB;
    double AxdaoG, AydaoG, BxdaoG, BydaoG;
    double AxdaoR, AydaoR, BxdaoR, BydaoR;
    double Axdao, Aydao, Bxdao, Bydao;
    //第一行
    int rowBegin = start_row;
    for (int j = start_col ; j < end_col ; ++j) {
        //最左侧
        if (j == start_col) {
            Axdao = - getColor(source_img, rowBegin, j - 1) - 2 * getColor(source_img, rowBegin + 1, j - 1) + getColor(source_img, rowBegin, j + 1) + 2 * getColor(source_img, rowBegin + 1, j + 1);
            Aydao = 2 * getColor(source_img, rowBegin + 1, j - 1) + getColor(source_img, rowBegin + 1, j) + 2 * getColor(source_img, rowBegin + 1, j + 1);
            
            Bxdao = getColor(target_img, rowBegin, j + 1) + 2 * getColor(target_img, rowBegin + 1, j + 1);
            Bydao = getColor(target_img, rowBegin + 1, j) + 2 * getColor(target_img, rowBegin + 1, j + 1);
        } else if (j == end_col - 1) {
        //最右侧
            Axdao = -getColor(source_img, rowBegin, j - 1) - 2 * getColor(source_img, rowBegin + 1, j - 1);
            Aydao = 2 * getColor(source_img, rowBegin + 1, j - 1) + getColor(source_img, rowBegin + 1, j);
            
            Bxdao = -getColor(target_img, rowBegin, j - 1) - 2 * getColor(target_img, rowBegin + 1, j - 1) + getColor(target_img, rowBegin, j + 1) + 2 * getColor(target_img, rowBegin + 1, j + 1);
            Bydao = 2 * getColor(target_img, rowBegin +1, j - 1) + getColor(target_img, rowBegin + 1, j) + 2 * getColor(target_img, rowBegin + 1, j + 1);
        } else {
        //中间
            Axdao = - getColor(source_img, rowBegin, j - 1) - 2 * getColor(source_img, rowBegin + 1,j - 1) + getColor(source_img, rowBegin, j + 1) + 2 * getColor(source_img, rowBegin + 1, j + 1);
            Aydao = 2 * getColor(source_img, rowBegin + 1, j - 1) + getColor(source_img, rowBegin + 1, j) + 2 * getColor(source_img, rowBegin + 1,j + 1);
            
            Bxdao = -getColor(target_img, rowBegin, j - 1) - 2 * getColor(target_img, rowBegin + 1, j - 1) + getColor(target_img, rowBegin, j + 1) + 2 * getColor(target_img, rowBegin + 1, j + 1);
            Bydao = 2 * getColor(target_img, rowBegin +1, j - 1) + getColor(target_img, rowBegin + 1, j) + 2 * getColor(target_img, rowBegin + 1, j + 1);
        }
        geometrys[rowBegin - start_row][j - start_col] = (AxdaoB - Bxdao) * (AxdaoB - Bxdao) + (Aydao - Bydao) * (Aydao - Bydao);
    }
    //中间行
    for (int j = start_col ; j < end_col ; ++j) {
        for (int i = start_row + 1 ; i < end_row - 1; ++i)  {
            //最左侧
            if (j == start_col) {
                Axdao = -2 * getColor(source_img, i - 1, j - 1) - getColor(source_img, i, j - 1) - 2 * getColor(source_img, i + 1,j - 1) + 2 * getColor(source_img, i - 1, j - 1) + getColor(source_img, i, j + 1) + 2 * getColor(source_img, i + 1, j + 1);
                Aydao = -2 * getColor(source_img, i - 1, j - 1) + 2 * getColor(source_img, i + 1, j - 1) - getColor(source_img, i - 1, j) + getColor(source_img, i + 1, j) - 2 * getColor(source_img, i - 1, j + 1) + 2 * getColor(source_img, i + 1,j + 1);
    
                Bxdao = 2 * getColor(target_img, i - 1, j + 1) + getColor(target_img, i, j + 1) + 2 * getColor(target_img, i + 1, j + 1);
                Bydao = -getColor(target_img, i - 1, j) + getColor(target_img, i + 1, j) - 2 * getColor(target_img, i - 1, j + 1) + 2 * getColor(target_img, i + 1, j + 1);
            } else if (j == end_col - 1) {
            //最右侧
                Axdao = -2 * getColor(source_img, i - 1, j - 1) - getColor(source_img, i, j - 1) - 2 * getColor(source_img, i + 1, j - 1);
                Aydao = -2 * getColor(source_img, i - 1, j - 1) + 2 * getColor(source_img, i + 1, j - 1) - getColor(source_img, i - 1, j) + getColor(source_img, i + 1, j);
    
                Bxdao = -2 * getColor(target_img, i - 1, j - 1) - getColor(target_img, i, j - 1) - 2 * getColor(target_img, i + 1, j - 1) + 2 * getColor(target_img, i - 1, j + 1) + getColor(target_img, i, j + 1) + 2 * getColor(target_img, i + 1, j + 1);
                Bydao = -2 * getColor(target_img, i - 1, j - 1) + 2 * getColor(target_img, i + 1, j - 1) - getColor(target_img, i - 1, j) + getColor(target_img, i + 1, j) - 2 * getColor(target_img, i - 1, j + 1) + 2 * getColor(target_img, i + 1, j + 1);

            } else {
            //中间
                Axdao = -2 * getColor(source_img, i - 1, j - 1) - getColor(source_img, i, j - 1) - 2 * getColor(source_img, i + 1,j - 1) + 2 * getColor(source_img, i - 1, j - 1) + getColor(source_img, i, j + 1) + 2 * getColor(source_img, i + 1, j + 1);
                Aydao = -2 * getColor(source_img, i - 1, j - 1) + 2 * getColor(source_img, i + 1, j - 1) - getColor(source_img, i - 1, j) + getColor(source_img, i + 1, j) - 2 * getColor(source_img, i - 1, j + 1) + 2 * getColor(source_img, i + 1,j + 1);
                Bxdao = -2 * getColor(target_img, i - 1, j - 1) - getColor(target_img, i, j - 1) - 2 * getColor(target_img, i + 1, j - 1) + 2 * getColor(target_img, i - 1, j + 1) + getColor(target_img, i, j + 1) + 2 * getColor(target_img, i + 1, j + 1);
                Bydao = -2 * getColor(target_img, i - 1, j - 1) + 2 * getColor(target_img, i + 1, j - 1) - getColor(target_img, i - 1, j) + getColor(target_img, i + 1, j) - 2 * getColor(target_img, i - 1, j + 1) + 2 * getColor(target_img, i + 1, j + 1);
            }
            geometrys[i - start_row][j - start_col] = (AxdaoB - Bxdao) * (AxdaoB - Bxdao) + (Aydao - Bydao) * (Aydao - Bydao);
        }
    }
    //最后一行
    int rowEnd = end_row - 1;
    for (int j = start_col ; j < end_col ; ++j) {
        //最左侧
        if (j == start_col) {
            Axdao = -2 * getColor(source_img, rowEnd - 1, j - 1) - getColor(source_img, rowEnd, j - 1) + 2 * getColor(source_img, rowEnd - 1, j + 1) + getColor(source_img, rowEnd, j + 1);
            Aydao = -2 * getColor(source_img, rowEnd - 1, j - 1) - getColor(source_img, rowEnd - 1, j) - 2 * getColor(source_img, rowEnd - 1,j + 1);
            
            Bxdao = 2 * getColor(target_img, rowEnd - 1, j + 1) + getColor(target_img, rowEnd, j + 1);
            Bydao = -getColor(target_img, rowEnd - 1, j) - 2 * getColor(target_img, rowEnd - 1, j + 1);
        } else if (j == end_col - 1) {
        //最右侧
            Axdao = -2 * getColor(source_img, rowEnd - 1, j - 1) - getColor(source_img, rowEnd, j - 1);
            Aydao = -2 * getColor(source_img, rowEnd - 1, j - 1) - getColor(source_img, rowEnd - 1, j);
            
            Bxdao = -2 * getColor(target_img, rowEnd - 1, j - 1) - getColor(target_img, rowEnd, j - 1) + 2 * getColor(target_img, rowEnd - 1, j + 1) + getColor(target_img, rowEnd, j + 1);
            Bydao = -2 * getColor(target_img, rowEnd - 1, j - 1) - getColor(target_img, rowEnd - 1, j) - 2 * getColor(target_img, rowEnd - 1, j + 1);
        } else {
        //中间
            Axdao = -2 * getColor(source_img, rowEnd - 1, j - 1) - getColor(source_img, rowEnd, j - 1) + 2 * getColor(source_img, rowEnd - 1, j + 1) + getColor(source_img, rowEnd, j + 1);
            Aydao = -2 * getColor(source_img, rowEnd - 1, j - 1) - getColor(source_img, rowEnd - 1, j) - 2 * getColor(source_img, rowEnd - 1,j + 1);
            
            Bxdao = -2 * getColor(target_img, rowEnd - 1, j - 1) - getColor(target_img, rowEnd, j - 1) + 2 * getColor(target_img, rowEnd - 1, j + 1) + getColor(target_img, rowEnd, j + 1);
            Bydao = -2 * getColor(target_img, rowEnd - 1, j - 1) - getColor(target_img, rowEnd - 1, j) - 2 * getColor(target_img, rowEnd - 1, j + 1);
        }
        geometrys[rowEnd - start_row][j - start_col] = (AxdaoB - Bxdao) * (AxdaoB - Bxdao) + (Aydao - Bydao) * (Aydao - Bydao);
    }
    for (int i = 0 ; i < colors.size() ; ++i) {
        for (int j = 0 ; j < colors[i].size() ; ++j) {
            strengths[i][j] = colors[i][j] * colors[i][j] + geometrys[i][j];
        }
    }
    for (int i = 0 ; i < end_row - start_row ; ++i) {
        for (int j = 0 ; j < end_col - start_col ; ++j) {
            if (i == 0) {
                strengthPath[i][j] = strengths[i][j];
            } else {
                if (j == 0) {
                    strengthPath[i][j] = min(strengthPath[i - 1][j], strengthPath[i - 1][j + 1]) + strengths[i][j];
                } else if(j == end_col - start_col - 1) {
                    strengthPath[i][j] = min(strengthPath[i - 1][j - 1], strengthPath[i - 1][j]) + strengths[i][j];
                } else {
                    strengthPath[i][j] = min(min(strengthPath[i - 1][j - 1], strengthPath[i - 1][j]), strengthPath[i - 1][j + 1]) + strengths[i][j];
                }
            }
        }
    }
    int find_col = -1;
    for (int row = end_row - 1; row >= start_row + 1 ; --row) {
        if (row == end_row - 1) {
            if (last_col != -1) {
                find_col = last_col;
            } else {
                float min_strength_value = -1;
                for (int j = start_col ; j < end_col; ++j) {
                    if (min_strength_value == -1 || strengthPath[row - start_row][j - start_col] < min_strength_value) {
                        min_strength_value = strengthPath[row - start_row][j - start_col];
                        find_col = j;
                    }
                }
            }
            paths.insert(paths.begin(), find_col);
        }
        if (find_col == start_col) {
            if (strengthPath[row - start_row - 1][find_col - start_col] < strengthPath[row - start_row - 1][find_col + 1 - start_col]) {
                find_col = find_col;
            } else{
                find_col = find_col + 1;
            }
        } else if (find_col == end_col - 1) {
            if (strengthPath[row - start_row - 1][find_col - start_col] < strengthPath[row - start_row - 1][find_col - 1 - start_col]) {
                find_col = find_col;
            } else {
                find_col = find_col - 1;
            }
        } else {
            if (strengthPath[row - start_row - 1][find_col - start_col] < strengthPath[row - start_row - 1][find_col + 1 - start_col] && strengthPath[row - start_row - 1][find_col - start_col] < strengthPath[row - start_row - 1][find_col - 1 - start_col]) {
                find_col = find_col;
            } else {
                if (strengthPath[row - start_row - 1][find_col - 1 - start_col] < strengthPath[row - start_row - 1][find_col + 1 - start_col]) {
                    find_col = find_col - 1;
                } else {
                    find_col = find_col + 1;
                }
            }
        }
       paths.insert(paths.begin(), find_col);
    }
}

void getRGB(const Mat& img, uchar& r, uchar& g, uchar& b, int row, int col) {
    b = img.ptr<uchar>(row)[col * 3];
    g = img.ptr<uchar>(row)[col * 3 + 1];
    r = img.ptr<uchar>(row)[col * 3 + 2];
}

void fillContent(Mat& result, const Mat& source_img, const Mat& target_img, int row, int col) {
    for (int j = 0 ; j < col ; ++j) {
        result.ptr<uchar>(row)[j * 3] = source_img.ptr<uchar>(row)[j * 3];
        result.ptr<uchar>(row)[j * 3 + 1] = source_img.ptr<uchar>(row)[j * 3 + 1];
        result.ptr<uchar>(row)[j * 3 + 2] = source_img.ptr<uchar>(row)[j * 3 + 2];
    }
    for (int j = col; j < result.cols ; ++j) {
        result.ptr<uchar>(row)[j * 3] = target_img.ptr<uchar>(row)[j * 3];
        result.ptr<uchar>(row)[j * 3 + 1] = target_img.ptr<uchar>(row)[j * 3 + 1];
        result.ptr<uchar>(row)[j * 3 + 2] = target_img.ptr<uchar>(row)[j * 3 + 2];
    }
//显示接缝
//    result.ptr<uchar>(row)[col * 3] = 0;
//    result.ptr<uchar>(row)[col * 3 + 1] = 0;
//    result.ptr<uchar>(row)[col * 3 + 2] = 255;
}

//判断接缝处source_img和target_img是否均有内容
bool judgeSuit(const Mat& source_img, const Mat& target_img, int row, int col) {
    uchar source_b, source_g, source_r, target_b, target_g, target_r;
    getRGB(source_img, source_r, source_g, source_b, row, col);
    getRGB(target_img, target_r, target_g, target_b, row, col);
    return !((source_b == 0 && source_g == 0 && source_r == 0) || (target_b == 0 && target_g == 0 && target_r == 0));
}

Mat stitchingByBestLine(const vector<int>& paths, const Mat& source_img, const Mat& target_img) {
    Mat result = Mat::zeros(source_img.rows, source_img.cols, CV_8UC3);
    int row = -1, col;
    uchar source_b, source_g, source_r, target_b, target_g, target_r;
    do {
        row ++;
        getRGB(source_img, source_r, source_g, source_b, row, paths[row]);
        getRGB(target_img, target_r, target_g, target_b, row, paths[row]);
        if (row >= paths.size()) break;
    } while ((source_b == 0 && source_g == 0 && source_r == 0) || (target_b == 0 && target_g == 0 && target_r == 0));

    assert(row < paths.size());
    for (int i = 0 ; i < row ; ++i) {
        col = paths[i];
        fillContent(result, source_img, target_img, i, col);
    }
    int prev_col = paths[row];
    while (row < paths.size()) {
        if (!judgeSuit(source_img, target_img, row, paths[row])) {
            int next_row;
            for (next_row = row ; next_row < paths.size() ; ++next_row) {
                if (judgeSuit(source_img, target_img, next_row, paths[next_row])) break;
            }
            if (next_row == paths.size()) {
                for (int i = row ; i < next_row ; ++i) {
                    col = paths[i];
                    fillContent(result, source_img, target_img, i, col);
                }
            } else {
                col = prev_col;
                for (int i = row ; i < next_row ; ++i) {
                    if (col > paths[next_row]) {
                        if (judgeSuit(source_img, target_img, i, col - 1)) {
                            col --;
                        } else {
                            if (!judgeSuit(source_img, target_img, i, col) && judgeSuit(source_img, target_img, i, col + 1)) {
                                col ++;
                            }
                        }

                    } else {
                        if (judgeSuit(source_img, target_img, i, col + 1)) {
                            col ++;
                        } else {
                            if (!judgeSuit(source_img, target_img, i, col) && judgeSuit(source_img, target_img, i, col - 1)) {
                                col --;
                            }
                        }
                    }
                    fillContent(result, source_img, target_img, i, col);
                }
            }
            prev_col = paths[next_row];
            row = next_row;
        } else {
            col = paths[row];
            fillContent(result, source_img, target_img, row, col);
            prev_col = col;
            row ++;
        }
    }

    return result;
}

Mat blending_images(const Mat& source_img, const Mat& target_img, int overlap_width, int source_col, int target_col, vector<int>& paths) {

    Mat source_view = source_img(Rect(0, 0, source_col + overlap_width, source_img.rows));
    Mat target_view = target_img(Rect(target_col - overlap_width, 0, target_img.cols - target_col + overlap_width, target_img.rows));

    Mat source_result = Mat::zeros(source_view.rows, source_view.cols + target_view.cols - overlap_width, CV_8UC3);
    Mat source_imageROI = source_result(Rect(0, 0, source_view.cols, source_view.rows));
    source_view.copyTo(source_imageROI);

    Mat target_result = Mat::zeros(source_view.rows, source_view.cols + target_view.cols - overlap_width, CV_8UC3);
    Mat target_imageROI = target_result(Rect(source_view.cols - overlap_width, 0, target_view.cols, target_view.rows));
    target_view.copyTo(target_imageROI);
    if(paths.size() == 0) {
        findBestLine(source_result, target_result, paths, source_view.cols - overlap_width, source_view.cols);
    }
    Mat result = stitchingByBestLine(paths, source_result, target_result);
//    for (int row = 0 ; row < result.rows ; ++row) {
//        result.ptr<uchar>(row)[(source_col + overlap_width) * 3] = 0;
//        result.ptr<uchar>(row)[(source_col + overlap_width) * 3 + 1] = 0;
//        result.ptr<uchar>(row)[(source_col + overlap_width) * 3 + 2] = 255;
//
//        result.ptr<uchar>(row)[source_col * 3] = 255;
//        result.ptr<uchar>(row)[source_col * 3 + 1] = 0;
//        result.ptr<uchar>(row)[source_col * 3 + 2] = 0;
//    }
    return result;
}