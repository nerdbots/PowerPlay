package teamcode.RobotUtilities;

public  enum FingerPositions {

    ENTER_INTAKE(0.53,0.55),
    INTAKE_READY(0.2,0.8),
    GRAB(0.62,0.38),
    RELEASE(0.58,0.42);

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