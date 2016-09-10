#include "Segment.h"
#include "Unit.h"
#include "Rendering/Models/3DModel.h"

///Vital Functors ////////////////////////////////////////////////////////////////
//Constructor
//We load into the Transformation Matrice the Identity Matrice
//Intialize the Segment - TODO at last Segment, hand over maxsize of segment
Segment::Segment() 
{
	T = T.Identity();
	//distance to next point - aka magnitude
	mag = 0;
	joint = BALLJOINT;
}


//Intialize the Segment - TODO at last Segment, hand over maxsize of segment
Segment::Segment(unsigned int pieceID, LocalModelPiece* lPiece,  Point3f pUnitNextPieceBasePoint, JointType jt) 
{
	T = T.Identity();
	this->pieceID= pieceID;
	piece= lPiece;

	float3 basePosition = piece->GetPosition();
	pUnitPieceBasePoint= Point3f(basePosition.x,basePosition.y,basePosition.z);

	//distance to next point - aka magnitude
	mag = distance(pUnitNextPieceBasePoint,pUnitPieceBasePoint);
	joint = jt;
	velocity= Point3f(0,0,0);
	this->print();
}

//Intialize the Segment - TODO at last Segment, hand over maxsize of segment
Segment::Segment(unsigned int pieceID, LocalModelPiece* lPiece, float magnitude, JointType jt) 
{
	T = T.Identity();
	this->pieceID= pieceID;
	piece= lPiece;

	float3 basePosition = piece->GetPosition();
	pUnitPieceBasePoint= Point3f(basePosition.x,basePosition.y,basePosition.z);
	
	//distance to next point - aka magnitude
	mag = magnitude;
	joint = jt;
	velocity= Point3f(0,0,0);
	this->print();
}

//Destructor
Segment::~Segment() 
{

}

///////////////////////////////////////////////////////////////////////////////////

//Memmbers

void Segment::print()
{
	std::cout<<"Segment "<<pieceID <<std::endl;
	std::cout<<"  {"<<std::endl;
	std::cout<<"   "<<"Base: P("<<pUnitPieceBasePoint<<")" <<std::endl;
	std::cout<<"   "<<"Length: "<< mag <<std::endl;
	std::cout<<"  }"<<std::endl;
}

float Segment::distance(Point3f a, Point3f b)
{
	a=a-b;
	return sqrtf(   a(0,0)*a(0,0) + 
					a(1,0)*a(1,0) +
					a(2,0)*a(2,0)); 
}


//Clamp Limited Joints - For a fixed Value, just specify Limits that are equal
 Point3f  Segment::clampJoint(Point3f Value)
 {

	Value(0,0) =  (Value(0,0) < jointUpLim(0,0))? Value(0,0) : jointUpLim(0,0);
	Value(1,0) =  (Value(1,0) < jointUpLim(1,0))? Value(1,0) : jointUpLim(1,0);
	Value(2,0) =  (Value(2,0) < jointUpLim(2,0))? Value(2,0) : jointUpLim(2,0);

	Value(0,0) =  (Value(0,0) < jointLowLim(0,0))? Value(0,0) : jointLowLim(0,0);
	Value(1,0) =  (Value(1,0) < jointLowLim(1,0))? Value(1,0) : jointLowLim(1,0);
	Value(2,0) =  (Value(2,0) < jointLowLim(2,0))? Value(2,0) : jointLowLim(2,0);

	return Value;
 }


Point3f Segment::get_end_point() {

	// start with vector going into the Z direction
	// transform into the rotation of the segment
	return (T * Point3f(0, 0, mag)) ;
}

///Gets the X-Component-  Pitch
Point3f Segment::get_right() 
{

	switch(joint){
	case BALLJOINT:
		return T * Point3f(1, 0, 0);
	case LIMJOINT:
		return clampJoint(T * Point3f(1, 0, 0));
	default:
		return  Point3f(0, 0, 0);
	}

}

///Gets the Y-Component - Yaw

Vector3f  Segment::get_up() 
{
	switch(joint){
		case BALLJOINT:
			return T * Vector3f (0, 1, 0);
		case LIMJOINT:
			return clampJoint(T * Vector3f (0, 1, 0));
		default:
			return  Vector3f (0, 0, 0);
	}
}


///Gets the Z-Component -Roll
Vector3f  Segment::get_z() {

	switch(joint)
	{
		case BALLJOINT:
			return T * Vector3f (0, 0, 1);
		case LIMJOINT:
			return clampJoint(T * Vector3f (0, 0, 1));
		default:
			return  Vector3f (0, 0, 0);
	}
}
   
 void Segment::setLimitJoint(   float limX, 
								float limUpX,
								float limY, 
								float limUpY,   
								float limZ,
								float limUpZ){
	joint= LIMJOINT;

	//Clamp the joint
	limX= (limX <= limUpX) ? limX : limUpX;
	limY= (limY <= limUpY) ? limY : limUpY;
	limZ= (limZ <= limUpZ) ? limZ : limUpZ;

	jointLowLim(0,0)=limX;
	jointLowLim(1,0)=limY;
	jointLowLim(2,0)=limZ;

	jointUpLim(0,0)=limUpX;
	jointUpLim(1,0)=limUpY;
	jointUpLim(2,0)=limUpZ;

 }


///Get the Transformation Matrice
AngleAxisf Segment::get_T() 
{
	return T;
}

float Segment::get_mag() 
{
	return mag;
}

void Segment::save_last_transformation() 
{
	last_T = T;
}

void Segment::load_last_transformation() 
{
	T = last_T;
}

void Segment::save_transformation() 
{
	saved_T = T;
}

void Segment::load_transformation() 
{
	T = saved_T;
}

void Segment::apply_angle_change(float rad_change, Vector3f  angle) 
{
	T = AngleAxisf(rad_change, angle) * T;
}

void Segment::transform(AngleAxisf t) 
{
	T = t * T;
}

void Segment::reset() 
{
	T = T.Identity();
}

Point3f Segment::get_rotation()
{
    Point3f worldVector(0,0,1);
    worldVector = T* worldVector;
    worldVector= worldVector.normalized();

    //now lets decompose this vector into radians
    std::cout<<"Transformation:"<<std::endl;
    std::cout<<worldVector(0,0)<<std::endl;
    std::cout<<worldVector(1,0)<<std::endl;
    std::cout<<worldVector(2,0)<<std::endl;

    

	float Yaw,Pitch, Roll;
    Roll = 0;

    Pitch = atan2(sqrt(pow(worldVector(0,0),2) + pow(worldVector(1,0),2)),
                            worldVector(2,0));
    Yaw = atan2 (worldVector(1,0),worldVector(0,0));

	return Point3f(Pitch,Yaw,Roll);
}