package teamcode.RobotUtilities;

import teamcode.RobotUtilities.core.PointPP;

public enum ArmShoulderPositions {

    INTAKE(10,1,0.28,0.72),
    LEVEL1(900,1,0.28,0.72),
    LEVEL2(1507,1,0.53,0.47),
    LEVEL3(2010,1,0.7,0.3),
    S4(310,1,0.28,0.72),
    S3(240,1,0.63,0.37),
    S2(150,1,0.63,0.37),
    S1(75,1,0.63,0.37),

    HOME(100,1,0.15,0.85),
    SHARED_HUB(1000, 1, 0.28,0.72);

    private  final int armTarget;
    private final double maxPower;
    private final double leftWristServoPosition;
    private final double rightWristServoPosition;


    private ArmShoulderPositions(int armTarget, double maxPower, double leftWristServoPosition, double rightWristServoPosition) {
        this.armTarget = armTarget;
        this.maxPower = maxPower;
        this.leftWristServoPosition = leftWristServoPosition;
        this.rightWristServoPosition = rightWristServoPosition;

    }

    public int getArmTarget() {
        return this.armTarget;
    }

    public double getMaxPower() {
        return this.maxPower;
    }

    public double getLeftWristServoPosition() {
        return this.leftWristServoPosition;
    }
    public double getRightWristServoPosition() {
        return this.rightWristServoPosition;
    }

    public static class CurvePoint {
        public double x;
        public double y;
        public double moveSpeed;
        public double turnSpeed;
        public double followDistance;
        public double pointLength;
        public double slowDownTurnRadians;
        public double slowDownTurnAmount;



        public CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance,
                          double slowDownTurnRadians, double slowDownTurnAmount){
            this.x = x;
            this.y = y;
            this.moveSpeed = moveSpeed;
            this.turnSpeed = turnSpeed;
            this.followDistance = followDistance;
            this.slowDownTurnRadians = slowDownTurnRadians;
            this.slowDownTurnAmount = slowDownTurnAmount;
        }

        public CurvePoint(CurvePoint thisPoint){
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
}