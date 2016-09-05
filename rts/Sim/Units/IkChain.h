/* This file is part of the Spring engine (GPL v2 or later), see LICENSE.html */

#ifndef IKCHAIN
#define IKCHAIN

#ifdef _WIN32
	//#define EIGEN_DONT_ALIGN_STATICALLY
	#define EIGEN_DONT_VECTORIZE
	#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#endif

#include <vector>
#include "point3f.h"
using namespace std;

#include "Segment.h"
#include "Rendering/Models/3DModel.h"



class CUnit;
class Segment;
class LocalModel;
///Class Ikchain- represents a Inverse Kinmatik Chain
class IkChain
{

public:
	///Constructors 

	//Create the segments
	IkChain(int id, CUnit* unit, LocalModelPiece* startPiece, float startPieceID, float endPieceID);

	//Helper Function to inialize the Path recursive
	bool recPiecePathExplore(LocalModelPiece* parentLocalModel, int parentPiece, int endPieceNumber, int depth);

	//IK is active or paused
	bool IKActive ;
	void SetActive (bool isActive);

	//Get the Next PieceNumber while building the chain
	int GetNextPieceNumber(float PieceNumber);

	void set_segments(std::vector<Segment> segments);

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

	//Solves the Inverse Kinematik Chain
	void solve(Point3f goal_point, int life_count);

	~IkChain();
private:

	//TODO find out what this one does
	Point3f calculateEndEffector(int Segment = -1);

	//Get the max Lenfgth of the IK Chain
	float getMaxLength();
};

#endif // IKCHAIN
