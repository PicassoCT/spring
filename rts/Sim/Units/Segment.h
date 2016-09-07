
/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#ifndef SEGMENT
#define SEGMENT

#include <vector>
#include "point3f.h"

typedef enum {
    BALLJOINT,
    MONOJOINT
} JointType;

class LocalModelPiece;

class Segment
{


private:
        // magnitude of the segment
        float mag;
        // transformation matrix (rotation) of the segment
        AngleAxisf T, saved_T, last_T;

        // save the angle when computing the changes
        Point3f saved_angle;

        // the type of joint the origin of the segment is
        // connected to
        JointType joint;

        Point3f nextStartPoint;

        //UnitOrigin Position always 0/0/0 in its own coord system
        Point3f pPieceBaseUnit;
        
    public:
        // constructors
        Segment();
        Segment(unsigned int pieceID, LocalModelPiece* lPiece, float magnitude, JointType jt);
      	Segment(unsigned int pieceID, LocalModelPiece* lPiece, Point3f nextStartPointOffset, JointType jt);
        ~Segment();

        Point3f velocity;
        Point3f angleLimits;

        //corresponding piece
        LocalModelPiece* piece;
        //yes, this is doubling (unchangeable) Information, but reducing pointeritis
        unsigned int pieceID;
 

        // returns end point in object space
        Point3f get_end_point();

        Point3f get_right();
        Point3f get_up();
        Point3f get_z();

        AngleAxisf get_T();
        float get_mag();

        void save_transformation();
        void load_transformation();

        void save_last_transformation();
        void load_last_transformation();

        void apply_angle_change(float rad_change, Point3f angle);

        // clear transformations
        void reset();
        // randomize transformation
        void randomize();

        // apply transformation
        void transform(AngleAxisf t);
};

#endif // Segment
