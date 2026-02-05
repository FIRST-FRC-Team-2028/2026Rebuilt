package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
//import edu.wpi.first.math.Matrix;
//import edu.wpi.first.math.Nat;
//import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;

/**Provide 2D vector arithmetic methods to treat Pose2D objects like vectors 
     * <p> ie, addition and subtraction are component operations - yes all three, including the angles
     * <p> But scalar multiplication, dot product, cross product,
     * <p> unit vector, norm 
     * <p> do not apply to the angles*/
public class PoseVec2d 
    //extends Vector
    {
    double[] x = new double[3];
    //Vector vMe;
    /** Construct an entity that acts like a vector from a Pose2d 
     * <p> ie, addition and subtraction are component operations
     * <p> scalar multiplication, dot product, cross product,
     * <p> unit vector, norm
    */
    public PoseVec2d(Pose2d me){
        //Matrix mat = new Matrix(<N3>, <N1>);
        //oduble[0]=me.getX();
        //super(mat);
        //super(<N1><Double> 3);
        x[0]=me.getX();
        x[1]=me.getY();
        x[2]=me.getRotation().getRadians();
    }

    /** Construct new PoseVec2d from componentscomponents
     * <p> Please provide angle in radians
     * @param x
     * @param y
     * @param t
     */
    public PoseVec2d(double x, double y, double t){
        this.x[0]=x;
        this.x[1]=y;
        this.x[2]=t;
    }

    /**Construct Pose2d from this "vector" */
    public Pose2d toPose2d() {
        return new Pose2d(x[0],x[1],new Rotation2d(x[2]));
    }

    /** return X component of this "vector" */
    public double X(){return x[0];}
    /** return Y component of this "vector" */
    public double Y(){return x[1];}
    /** return angle component of this "vector" */
    public double T(){return x[2];}

    /**Add components of addend to this vector and return the resulting new vector */
    public PoseVec2d plus(PoseVec2d addend) {
        return new PoseVec2d(x[0]+addend.X(),
                             x[1]+addend.Y(),
                             x[2]+addend.T()
                            );
    }

    /**subtract components of addend from this vector and return the resulting new vector */
    public PoseVec2d minus(PoseVec2d addend) {
        return new PoseVec2d(x[0]-addend.X(),
                             x[1]-addend.Y(),
                             x[2]-addend.T()
                            );
    }

    /**Compute dot product with
     * @param dotee
     */
    public double dot(PoseVec2d dotee) {
        return x[0]*dotee.X() + x[0]*dotee.Y();
    }

    /**Compute second norm  */
    public  double norm() {
        //return x[0]*x[0] + x[1]*x[1];
        return this.dot(this);
     }

     /**
      * Apply multiplier only to X and Y components, not angle T: use original
     * @param mult
     * @return a new PoseVec2d
     */
    public PoseVec2d scalarProd(double mult) {
        return new PoseVec2d(x[0]*mult,
                             x[1]*mult,
                             x[2]
                            );
    }

    /**Generate vector of length 1 in the same direction*/
    public PoseVec2d unit() {
        return scalarProd(1./norm());
    }

    /** Compute inner product */
    public double cross(PoseVec2d posee) {
        return x[0]*posee.Y() - x[1]*posee.X();
    }

    /** transform (not done yet)
    public PoseVec2d transform(Matrix<N3,N3> trans){
        PoseVec2d res = new PoseVec2d(this.dot(trans.extractColumnVector(0)),
                                      this.dot(trans.extractColumnVector(1)),
                                      this.dot(trans.extractColumnVector(2)));
    } */

    /* example of use 
     *  dest = start + (total dist - range) * ||(hub - whereIam)||
     * 
    void goTorange() {
        PoseVec2d diff = new PoseVec2d(AprilTag.getPose2d()).minus(whereIam);
        double dist = diff.norm() - desiredRange;
        PoseVec2d whereToGo = whereIam.plus(diff.unit().scalarProd(dist));  // this doesn't change the heading
    }
    */
}
