package frc.robot;

//import edu.wpi.first.apriltag.AprilTag;
//import edu.wpi.first.math.Matrix;
//import edu.wpi.first.math.Matrix;
//import edu.wpi.first.math.Nat;
//import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.numbers.N3;

/**Provide 2D vector arithmetic methods to treat Pose2D objects like vectors 
     * <p> ie, addition and subtraction are component operations - yes all three, including the angles
     * <p> But scalar multiplication, dot product, cross product,
     * <p> unit vector, norm 
     * <p> do not apply to the angles*/
public class VPose2d 
    //extends Vector
    {
    double[] x = new double[3];
    //Vector vMe;
    /**  From a Pose2d, construct an entity that acts like a vector
     * <p> ie, addition and subtraction are component operations.
     * <p> scalar multiplication, dot product, cross product,
     * <p> unit vector, norm apply to the location parts
    */
    public VPose2d(Pose2d me){
        x[0]=me.getX();
        x[1]=me.getY();
        x[2]=me.getRotation().getRadians();
    }

    /** Construct new VPose2d from componentscomponents
     * <p> Please provide angle in radians
     * @param x  location, meters to be consistent with WPI
     * @param y  location, ditto
     * @param t  heading angle, radians - ditto
     */
    public VPose2d(double x, double y, double t){
        this.x[0]=x;
        this.x[1]=y;
        this.x[2]=t;
    }

    /**Construct Pose2d from this "vector" */
    public Pose2d toPose2d() {
        return new Pose2d(x[0],x[1],new Rotation2d(x[2]));
    }
    /**Construct Pose2d from this "vector" 
     @param rot  rotation to overwrite, radians
    */
    public Pose2d toPose2d(double rot) {
        return new Pose2d(x[0],x[1],new Rotation2d(rot));
    }

    /** return X component of this "vector" */
    public double X(){return x[0];}
    /** return Y component of this "vector" */
    public double Y(){return x[1];}
    /** return angle component of this "vector" */
    public double T(){return x[2];}

    /**Add components of addend to this vector and return the resulting new vector */
    public VPose2d plus(VPose2d addend) {
        return new VPose2d(x[0]+addend.X(),
                             x[1]+addend.Y(),
                             x[2]+addend.T()
                            );
    }

    /**subtract components of addend from this vector and return the resulting new vector */
    public VPose2d minus(VPose2d addend) {
        return new VPose2d(x[0]-addend.X(),
                             x[1]-addend.Y(),
                             x[2]-addend.T()
                            );
    }

    /**Compute dot product with
     * @param dotee
     */
    public double dot(VPose2d dotee) {
        return x[0]*dotee.X() + x[1]*dotee.Y();
    }

    /**Compute second norm  */
    public  double norm() {
        //return x[0]*x[0] + x[1]*x[1];
        return  Math.sqrt(this.dot(this)) ;
     }

     /** Apply multiplier only to X and Y components, not angle T: use original
     * @param mult applied to x and y; rot
     * @return a new VPose2d
     */
    public VPose2d scalarProd(double mult) {
        return scalarProd(mult, 1.);
    }
    public VPose2d scalarProd(double mult, double multr) {
        return new VPose2d(x[0]*mult,
                             x[1]*mult,
                             x[2] *multr  // just in case you might want
                            );
    }

    /**Generate vector of length 1 in the same direction*/
    public VPose2d unit() {
        return scalarProd(1./norm());
    }

    /** Compute inner product */
    public double cross(VPose2d posee) {
        return x[0]*posee.Y() - x[1]*posee.X();
    }

    /** transform (not done yet)
    public VPose2d transform(Matrix<N3,N3> trans){
        VPose2d res = new VPose2d(this.dot(trans.extractColumnVector(0)),
                                      this.dot(trans.extractColumnVector(1)),
                                      this.dot(trans.extractColumnVector(2)));
    } */

    /* example of use 
     *  dest = start + (total dist - range) * ||(hub - whereIam)||
     * 
    void goTorange() {
        VPose2d diff = new VPose2d(AprilTag.getPose2d()).minus(whereIam);
        double dist = diff.norm() - desiredRange;
        VPose2d whereToGo = whereIam.plus(diff.unit().scalarProd(dist));  // this doesn't change the heading
    }
    */
}
