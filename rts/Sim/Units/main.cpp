


#ifdef _WIN32
//#define EIGEN_DONT_ALIGN_STATICALLY

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#endif

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <vector>
#include <list>
#include <cmath>
#include <Eigen/Dense>
#include "IkChain.h"

using namespace std;
using namespace Eigen;


IkChain mainArm;
IkChain secArm;

Vector3f goal;



void update() {
    
    goal += Vector3f(-3,0,-1);
    //goal = Vector3f(0, 0, 7);
    mainArm.solve(goal, 100);
    secArm.solve(goal, 100);
}




int main(int argc, char* argv[]) {


    vector<Segment> segs;
    Segment* new_seg = new Segment(1, BALLJOINT);
    segs.push_back(*new_seg);
    new_seg = new Segment(1, BALLJOINT);
	segs.push_back(*new_seg);
    new_seg = new Segment(1, BALLJOINT);
    segs.push_back(*new_seg);
    new_seg = new Segment(3.5, BALLJOINT);
    segs.push_back(*new_seg);
    new_seg = new Segment(2.5, BALLJOINT);
    segs.push_back(*new_seg);
    new_seg = new Segment(1, BALLJOINT);
    segs.push_back(*new_seg);

    vector<Segment> segs2;
    Segment* new_seg2 = new Segment(1, BALLJOINT);
    segs2.push_back(*new_seg2);
    new_seg2 = new Segment(1, BALLJOINT);
	segs2.push_back(*new_seg2);
    new_seg2 = new Segment(1, BALLJOINT);
    segs2.push_back(*new_seg2);
    new_seg2 = new Segment(3.5, BALLJOINT);
    segs2.push_back(*new_seg2);
    new_seg2 = new Segment(2.5, BALLJOINT);
    segs2.push_back(*new_seg2);
    new_seg2 = new Segment(1, BALLJOINT);
    segs2.push_back(*new_seg2);

    mainArm.set_segments(segs);
    secArm.set_segments(segs2);

    while (true)
    {
        update();
    }


    return 0;
}
