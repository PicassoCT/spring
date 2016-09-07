/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#ifndef IKCHAIN
#define IKCHAIN

#ifdef _WIN32
	//#define EIGEN_DONT_ALIGN_STATICALLY
	#define EIGEN_DONT_VECTORIZE
	#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#endif

#include <vector>
#include "Segment.h"
#include "point3f.h"

using namespace Eigen;

class LocalModelPiece;
class LocalModel;
class CUnit;

typedef enum {
    OVERRIDE,
    BLENDIN
} MotionBlend;


///Class Ikchain- represents a Inverse Kinmatik Chain
class IkChain
{

public:
	enum AnimType {ANone = -1, ATurn = 0, ASpin = 1, AMove = 2};

	///Constructors 
	IkChain();
	//Create the segments
	IkChain(int id, CUnit* unit, LocalModelPiece* startPiece, unsigned int startPieceID, unsigned int endPieceID);

	//Was the Goal altered
	bool GoalChanged=true;

	//Helper Function to inialize the Path recursive
	bool recPiecePathExplore(LocalModelPiece* parentLocalModel, unsigned int parentPiece, unsigned int endPieceNumber, int depth);
	bool initializePiecePath(LocalModelPiece* startPiece, unsigned int startPieceID, unsigned int endPieceID);
	
	//Checks wether a Piece is part of this chain
	bool isValidIKPiece(float pieceID);

	//IK is active or paused
	bool IKActive ;

	//Setter
	void SetActive (bool isActive);

	//Solves the Inverse Kinematik Chain for a new goal point
	void solve(float frames );

	//apply the resolved Kinematics to the actual Model
	void applyIkTransformation(MotionBlend motionBlendMethod);

	//Get the Next PieceNumber while building the chain
	int GetNextPieceNumber(float PieceNumber);


	//Creates a Jacobi Matrice
    Matrix<float,1,3> compute_jacovian_segment(int seg_num, Point3f goal_point, Point3f angle);

    // computes end_effector up to certain number of segments
    Point3f calculate_end_effector(int segment_num = -1);

	//unit this IKChain belongs too
	CUnit* unit;

	//Identifier of the Kinematik Chain
	float IkChainID;

	//The baseposition in WorldCoordinats
	Point3f base;

	// the goal Point also in World Coordinats
	Point3f goalPoint;

	//Vector containing the Segments
	std::vector <Segment> segments;

	//Size of Segment
	int segment_size ;


	//Destructor
	~IkChain();
private:

	//TODO find out what this one does
	Point3f calculateEndEffector(int Segment = -1);

	//Get the max Lenfgth of the IK Chain
	float getMaxLength();
};

#endif // IKCHAIN
