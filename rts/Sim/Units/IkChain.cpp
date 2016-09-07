
#include "IkChain.h"
#include "Unit.h"
//#include "point3f.h"
#include "Rendering/Models/3DModel.h"
#include "Sim/Units/Scripts/UnitScript.h"

// See end of source for member bindings
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


//checks wether a IK-Chain contains a piece 
bool IkChain::isValidIKPiece(float pieceID){
        if (segments.empty()) {return false;}

        for (auto seg = segments.begin(); seg !=  segments.end(); ++seg) 
        {
           if ((*seg).pieceID == pieceID) return true;
        }

return false;
}

void IkChain::SetActive(bool isActive)
{
    IKActive  = isActive;
}

//recursive explore the model and find the end of the IK-Chain
bool IkChain::recPiecePathExplore(LocalModelPiece* parentLocalModelPiece, int parentPiece, int endPieceNumber, int depth){
    //Get DecendantsNumber
   
      for (auto piece = parentLocalModelPiece->children.begin(); piece !=  parentLocalModelPiece->children.end(); ++piece) 
        {
            LocalModelPiece* lPiece = *piece;
            
            //we found the last piece of the kinematikChain
            if ((lPiece)->scriptPieceIndex == endPieceNumber){
                //lets gets size as a float
                float magnitude = 42.0f; //TODO Get the magnitude dude
                segments[depth]= *(new Segment( magnitude, BALLJOINT));
                segments[depth].piece= lPiece;
                segments[depth].pieceID= lPiece->scriptPieceIndex;
                segment_size= depth + 1;
                return true;
            }
            
            //if one of the pieces children is the endpiece backtrack
            if (recPiecePathExplore(lPiece, lPiece->scriptPieceIndex , endPieceNumber, depth+1 ) == true)
            {
                //Get the magnitude of the piece
                Point3f nextStartPointOffset= Point3f(parentLocalModelPiece->GetAbsolutePos().x,
                                        parentLocalModelPiece->GetAbsolutePos().y,
                                        parentLocalModelPiece->GetAbsolutePos().z) ;

                segments[depth]= *(new Segment( nextStartPointOffset, BALLJOINT));
                segments[depth].piece= lPiece;
                segments[depth].pieceID= lPiece->scriptPieceIndex;
                
                return true;        
            }

        }
       
return false;
}


bool IkChain::initializePiecePath(LocalModelPiece* startPiece, int startPieceID, int endPieceID){
    //Check for 
     if (startPiece  == NULL || startPieceID < 1 || endPieceID < 1 ) return false;
    //TODO check wether start and Endpiece exist

    IKActive= true;

    return recPiecePathExplore(startPiece, startPieceID, endPieceID, 0);
}


IkChain::IkChain(int id, CUnit* unit, LocalModelPiece* startPiece, float startPieceID, float endPieceID )
{
    this->unit= unit;

    initializePiecePath(startPiece, (int) startPieceID, (int) endPieceID);

    IkChainID = id;
}

IkChain::~IkChain()
{
	//Release the Pieces
}
//////////////////////////////////////////////////////////////////////
// Template for the pseudo Inverse
//////////////////////////////////////////////////////////////////////
template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon =
std::numeric_limits<double>::epsilon())
{
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU |
    Eigen::ComputeThinV);

    double tolerance =  epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
    return svd.matrixV() *  (svd.singularValues().array().abs() >
    tolerance).select(svd.singularValues().array().inverse(),
    0).matrix().asDiagonal() * svd.matrixU().adjoint();
}
//////////////////////////////////////////////////////////////////////
float IkChain::getMaxLength(){
	float totalDistance=0;

    for(std::vector<Segment>::iterator seg = segments.begin(); seg != segments.end(); ++seg)
    {
        totalDistance += seg->get_mag();
    }
		//for (auto seg = segments.cbegin(); seg != segments.cend(); ++seg) {
		//totalDistance += seg.get_mag();
		//}
return NULL;
};

//TODO need max Speed per Segment and a diffrent Joint Type with Limited Rotation
void IkChain::solve(float frames) {
    // prev and curr are for use of halving
    // last is making sure the iteration gets a better solution than the last iteration,
    // otherwise revert changes
    float prev_err, curr_err, last_err = 9999;
    Point3f current_point;
    int max_iterations = 200;
    int count = 0;
    float err_margin = 0.01;

    Point3f goal_point = goalPoint;

    goal_point -= base;
    if (goal_point.norm() > getMaxLength()) {
        goal_point = goal_point.normalized() * getMaxLength();
    }

    current_point = calculate_end_effector();

    // save the first err
    prev_err = (goal_point - current_point).norm();
    curr_err = prev_err;
    last_err = curr_err;

    // while the current point is close enough, stop iterating
    while (curr_err > err_margin) {
        // calculate the difference between the goal_point and current_point
        Point3f dP = goal_point - current_point;

        // create the jacovian
        int segment_size = segments.size();

        // build the transpose matrix (easier for eigen matrix construction)
        MatrixXf jac_t(3*segment_size, 3);
        for(int i=0; i<3*segment_size; i+=3) {
            Matrix<float, 1, 3> row_theta = compute_jacovian_segment(i/3, goal_point, (segments[i/3]).get_right());
            Matrix<float, 1, 3> row_phi = compute_jacovian_segment(i/3, goal_point, (segments[i/3]).get_up());
            Matrix<float, 1, 3> row_z = compute_jacovian_segment(i/3, goal_point, (segments[i/3]).get_z());

            jac_t(i, 0) = row_theta(0, 0);
            jac_t(i, 1) = row_theta(0, 1);
            jac_t(i, 2) = row_theta(0, 2);

            jac_t(i+1, 0) = row_phi(0, 0);
            jac_t(i+1, 1) = row_phi(0, 1);
            jac_t(i+1, 2) = row_phi(0, 2);

            jac_t(i+2, 0) = row_z(0, 0);
            jac_t(i+2, 1) = row_z(0, 1);
            jac_t(i+2, 2) = row_z(0, 2);
        }
        // compute the final jacovian
        MatrixXf jac(3, 3*segment_size);
        jac = jac_t.transpose();

        Matrix<float, Dynamic, Dynamic> pseudo_ijac;
        MatrixXf pinv_jac(3*segment_size, 3);
        pinv_jac = pseudoInverse(jac);

        Matrix<float, Dynamic, 1> changes = pinv_jac * dP;



        for(int i=0; i<3*segment_size; i+=3) {
            // save the current transformation on the segments
            segments[i/3].save_transformation();

            // apply the change to the theta angle
            segments[i/3].apply_angle_change(changes[i], segments[i/3].get_right());
            // apply the change to the phi angle
            segments[i/3].apply_angle_change(changes[i+1], segments[i/3].get_up());
            // apply the change to the z angle
            segments[i/3].apply_angle_change(changes[i+2], segments[i/3].get_z());
        }

        // compute current_point after making changes
        current_point = calculate_end_effector();

        //cout << "current_point: " << vectorString(current_point) << endl;
        //cout << "goal_point: " << vectorString(goal_point) << endl;

        prev_err = curr_err;
        curr_err = (goal_point - current_point).norm();

        int halving_count = 0;

        // make sure we aren't iterating past the solution
        while (curr_err > last_err) {
            // undo changes
            for(int i=0; i<segment_size; i++) {
                // unapply the change to the saved angle
                segments[i].load_transformation();
            }
            current_point = calculate_end_effector();
            changes *= 0.5;
            // reapply halved changes
            for(int i=0; i<3*segment_size; i+=3) {
                // save the current transformation on the segments
                segments[i/3].save_transformation();

                // apply the change to the theta angle
                segments[i/3].apply_angle_change(changes[i], segments[i/3].get_right());
                // apply the change to the phi angle
                segments[i/3].apply_angle_change(changes[i+1], segments[i/3].get_up());
                // apply the change to the z angle
                segments[i/3].apply_angle_change(changes[i+2], segments[i/3].get_z());
            }

            // compute the end_effector and measure error
            current_point = calculate_end_effector();
            prev_err = curr_err;
            curr_err = (goal_point - current_point).norm();

            //cout << "|half| curr err: " << curr_err << " || prev err: " << prev_err << endl;
            halving_count++;
            if (halving_count > 100)
                break;
        }

        if (curr_err > last_err) {
            // undo changes
            for(int i=0; i<segment_size; i++) {
                // unapply the change to the saved angle
                segments[i].load_last_transformation();
            }
            current_point = calculate_end_effector();
            curr_err = (goal_point - current_point).norm();
            //cout << "curr iteration not better than last, reverting" << endl;
            //cout << "curr err: " << curr_err << " || last err: " << last_err << endl;
            break;
        }
        for(int i=0; i<segment_size; i++) {
            // unapply the change to the saved angle
            segments[i].save_last_transformation();
        }
        //cout << "curr err: " << curr_err << " || last err: " << last_err << endl;
        last_err = curr_err;
        //cout << "last_err is now : " << last_err << endl;


        // make sure we don't infinite loop
        count++;
        if (count > max_iterations) {
            break;
        }
    }

    applyIkTransformation(OVERRIDE);

    /*
    // if we haven't gotten to a nice solution
    if (curr_err > err_margin) {
        // kill off infinitely recursive solutions
        if (life_count <= 0) {
            return;
        }

        // try to solve it again
        solve(goal_point, life_count-1);
    } else {
    */
}

void IkChain::applyIkTransformation(MotionBlend motionBlendMethod){
    //Get the Unitscript for the Unit that holds the segment




    //itterate over the pieces
    for(int i=0; i<segment_size; i++) {
            unit->script->AddAnim(   CUnitScript::ATurn,
                                    (int)segments[i].pieceID,
                                    0.0f ,
                                    (segments[i].velocity)[0],
                                    (segments[i].get_right())[0], //TODO jointclamp this
                                    0.0f
                                    );

            unit->script->AddAnim( CUnitScript::ATurn,
                                    (int)segments[i].pieceID,
                                    1.0f,
                                    (segments[i].velocity)[1],
                                    (segments[i].get_up())[1], //TODO jointclamp this
                                    0.0f
                                    );

            unit->script->AddAnim(  CUnitScript::ATurn,
                                    (int)segments[i].pieceID,
                                    2.0f,
                                    (segments[i].velocity)[2],
                                    (segments[i].get_z())[2], //TODO jointclamp this
                                    0.0f
                                    );
        }

    //apply transformation to the instance geometry



}


// computes end_effector up to certain number of segments
Point3f IkChain::calculate_end_effector(int segment_num /* = -1 */) {
    Point3f ret;

    int segment_num_to_calc = segment_num;
    // if default value, compute total end effector
    if (segment_num == -1) {
        segment_num_to_calc = segments.size() - 1;
    }
    // else don't mess with it

    // start with base
    ret = base;
    for(int i=0; i<=segment_num_to_calc; i++) {
        // add each segments end point vector to the base
        ret += segments[i].get_end_point();
    }
    // return calculated end effector
    return ret;
}


//Returns a Jacovian Segment a row of 3 Elements
Matrix<float, 1, 3>  IkChain::compute_jacovian_segment(int seg_num, Point3f goalPoint, Point3f angle) 
{
    Segment s = segments[seg_num];
    // mini is the amount of angle you go in the direction for numerical calculation
    float mini = 0.0005;

    Point3f transformed_goal = goalPoint;
    for(int i=segments.size()-1; i>seg_num; i--) {
        // transform the goal point to relevence to this segment
        // by removing all the transformations the segments afterwards
        // apply on the current segment
        transformed_goal -= segments[i].get_end_point();
    }

    Point3f my_end_effector = calculate_end_effector(seg_num);

    // transform them both to the origin
    if (seg_num-1 >= 0) {
        my_end_effector -= calculate_end_effector(seg_num-1);
        transformed_goal -= calculate_end_effector(seg_num-1);
    }

    // original end_effector
    Point3f original_ee = calculate_end_effector();

    // angle input is the one you rotate around
    // remove all the rotations from the previous segments by applying them
    AngleAxisf t = AngleAxisf(mini, angle);

    // transform the segment by some delta(theta)
    s.transform(t);
    // new end_effector
    Point3f new_ee = calculate_end_effector();
    // reverse the transformation afterwards
    s.transform(t.inverse());

        // difference between the end_effectors
    // since mini is very small, it's an approximation of
    // the derivative when divided by mini
    Point3f diff = new_ee - original_ee;

    // return the row of dx/dtheta, dy/dtheta, dz/dtheta
    Matrix<float, 1, 3> ret;
    ret << diff[0]/mini, diff[1]/mini, diff[2]/mini;
    return ret;
}
// computes end_effector up to certain number of segments
Point3f IkChain::calculateEndEffector(int segment_num /* = -1 */) {
    Point3f ret;

    int segment_num_to_calc = segment_num;
    // if default value, compute total end effector
    if (segment_num == -1) {
        segment_num_to_calc = segments.size() - 1;
    }
    // else don't mess with it

    // start with base
    ret = base;
    for(int i=0; i<=segment_num_to_calc; i++) {
        // add each segments end point vector to the base
        ret += segments[i].get_end_point();
    }
    // return calculated end effector
    return ret;
}

void IkChain::set_segments(std::vector<Segment> segments){

};




/******************************************************************************/
