package teamcode.RobotUtilities;

public  enum FingerPositions {

    ENTER_INTAKE(0.53,0.55),
    INTAKE_READY(0.57,0.51),
    GRAB(0.5,0.65),
    RELEASE(0.6,0.5);

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