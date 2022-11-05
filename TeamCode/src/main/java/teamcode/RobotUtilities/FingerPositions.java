package teamcode.RobotUtilities;

public  enum FingerPositions {

    ENTER_INTAKE(0.53,0.55), //Not using anymore for PowerPlay
    INTAKE_READY(0.2,0.8), //Opens the claw
    GRAB(0.62,0.38),
    RELEASE(0.584,0.416); //Not using anymore for PowerPlay

    private final double leftFingerPosition;
    private final double rightFingerPosition;

    private FingerPositions(double leftFingerPosition, double rightFingerPosition){
        this.leftFingerPosition = leftFingerPosition;
        this.rightFingerPosition = rightFingerPosition;
    }

    public double getLeftFingerPosition() {
        return this.leftFingerPosition;
    }
    public double getRightFingerPosition() {
        return this.rightFingerPosition;
    }


}