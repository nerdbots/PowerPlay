package teamcode.RobotUtilities.Odometry;

//import opencv.core.Point;
import teamcode.RobotUtilities.Odometry.CurvePoint;
import teamcode.RobotUtilities.core.PointPP;
public class CurvePoint1 {
    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;
    public double pointLength;
    public double slowDownTurnRadians;
    public double slowDownTurnAmount;



    public CurvePoint1(double x, double y, double moveSpeed, double turnSpeed, double followDistance,
                      double slowDownTurnRadians, double slowDownTurnAmount){
        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
        this.slowDownTurnRadians = slowDownTurnRadians;
        this.slowDownTurnAmount = slowDownTurnAmount;
    }

    public CurvePoint1(CurvePoint thisPoint){
        x = thisPoint.x;
        y = thisPoint.y;
        moveSpeed = thisPoint.moveSpeed;
        turnSpeed = thisPoint.turnSpeed;
        followDistance = thisPoint.followDistance;
        slowDownTurnRadians = thisPoint.slowDownTurnRadians;
        slowDownTurnAmount = thisPoint.slowDownTurnAmount;
        pointLength = thisPoint.pointLength;
    }

    public PointPP toPoint(){

        return new PointPP(x, y);
    }

    public void setPoint(PointPP point){
        x = point.x;
        y = point.y;

    }

}