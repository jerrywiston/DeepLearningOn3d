#include <cmath>
using namespace std;

float IOU(float center1[3], float size1[3], float center2[3], float size2[3]){
    float cube[3][2];
    float c1[2], c2[2];
    for(int i=0; i<3; ++i){
        c1[0] = center1[i] - size1[i]/2;
        c1[1] = center1[i] + size1[i]/2;
        c2[0] = center2[i] - size2[i]/2;
        c2[1] = center2[i] + size2[i]/2;
        cube[i][0] = (c1[0] > c2[0])? c1[0]:c2[0];
        cube[i][1] = (c1[1] < c2[1])? c1[1]:c2[1];
    }

    float v1 = size1[0]*size1[1]*size1[2];
    float v2 = size2[0]*size2[1]*size2[2];

    float Overlap = abs((cube[0][0]-cube[0][1])*
                        (cube[1][0]-cube[1][1])*
                        (cube[2][0]-cube[2][1]));
    float UnionPart = v1 + v2 - Overlap;

    return Overlap/UnionPart;
}
