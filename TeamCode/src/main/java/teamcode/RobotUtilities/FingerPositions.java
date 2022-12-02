package teamcode.RobotUtilities;

public  enum FingerPositions {

    ENTER_INTAKE(0.53,0.55),
    INTAKE_READY(0.25,0.75),
    GRAB(0.7,0.3),
    RELEASE(0.58,0.42);

    public final double leftFingerPosition;
    public final double rightFingerPosition;

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