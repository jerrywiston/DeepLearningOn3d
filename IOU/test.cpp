#include <iostream>
#include "IOU.hpp"

using namespace std;

int main(){
    float c1[3];
    float s1[3];
    float c2[3];
    float s2[3];

    c1[0] = 0; c1[1] = 0; c1[2] = 0;
    c2[0] = 1; c2[1] = 0; c2[2] = 0;

    s1[0] = 2; s1[1] = 2; s1[2] = 2;
    s2[0] = 2; s2[1] = 2; s2[2] = 2;

    cout << IOU(c1,s1,c2,s2) << endl;
    return 0;
}
