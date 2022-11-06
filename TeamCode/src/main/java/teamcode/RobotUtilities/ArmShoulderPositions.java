package teamcode.RobotUtilities;

public enum ArmShoulderPositions {

    INTAKE(10,1,0.28,0.72),
    LEVEL1(900,1,0.28,0.72),
    LEVEL2(1507,1,0.53,0.47),
    LEVEL3(2010,1,0.7,0.3),
    TSE_DROP(600,1,0.62,0.38),
    GROUND_PICKUP(1000,1,0.63,0.37),
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

}