#include "Segment.h"

//Constructor
//We load into the Transformation Matrice the Identity Matrice


//Intialize the Segment - TODO at last Segment, hand over maxsize of segment
Segment::Segment(Vector3f nextStartPoint, JointType jt) {
	T = T.Identity();

	nextStartPoint[0]= pUnitPiece[0] - nextStartPoint[0];
	nextStartPoint[1]= pUnitPiece[1] - nextStartPoint[1];
	nextStartPoint[2]= pUnitPiece[2] - nextStartPoint[2];

    //distance to next point - aka magnitude
    mag = sqrtf(nextStartPoint[0] * nextStartPoint[0] +
                nextStartPoint[1] * nextStartPoint[1] + 
                nextStartPoint[2] * nextStartPoint[2]);
    joint = jt;
}

//Intialize the Segment - TODO at last Segment, hand over maxsize of segment
Segment::Segment(float magnitude, JointType jt) {
    T = T.Identity();
    //distance to next point - aka magnitude
    mag = magnitude;
    joint = jt;
}

//Destructor
Segment::~Segment() {

}

Vector3f Segment::get_end_point() {
    // start with vector going into the Z direction
    // transform into the rotation of the segment

    return T * Vector3f(0, 0, mag);
}

///Gets the X-Component-  Pitch
Vector3f Segment::get_right() {
    return T*Vector3f(1,0,0);
}

///Gets the Y-Component - Yaw

Vector3f Segment::get_up() {
    return T*Vector3f(0,1,0);
}


///Gets the Z-Component -Roll

Vector3f Segment::get_z() {
    return T*Vector3f(0,0,1);
}


///Get the Transformation Matrice
AngleAxisf Segment::get_T() {
    return T;
}

float Segment::get_mag() {
    return mag;
}

void Segment::save_last_transformation() {
    last_T = T;
}

void Segment::load_last_transformation() {
    T = last_T;
}

void Segment::save_transformation() {
    saved_T = T;
}

void Segment::load_transformation() {
    T = saved_T;
}

void Segment::apply_angle_change(float rad_change, Vector3f angle) {
    T = AngleAxisf(rad_change, angle) * T;
}

void Segment::transform(AngleAxisf t) {
    T = t * T;
}

void Segment::reset() {
    T = T.Identity();
}
