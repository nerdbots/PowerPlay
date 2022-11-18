package teamcode.Auton;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import teamcode.RobotUtilities.Odometry.CurvePoint;
import teamcode.RobotUtilities.Odometry.OdometryGlobalCoordinatePositionNERD;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

//import opencv.teamcode.RobotUtilities.core.PointPP;
import teamcode.RobotUtilities.core.PointPP;
import teamcode.RobotUtilities.*;

import java.util.ArrayList;

public class PurePursuitRobotMovement6_Turn_MultiThread {

    private boolean debugFlag=false;

    //We need an opmode to get the hardware map etc.

    private LinearOpMode opmode;

    private HardwareMap hardwareMap;

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;

    public DcMotor frontEncoder;
    public DcMotor rightEncoder;
    public DcMotor leftEncoder;
    public DcMotor backEncoder;

    private DcMotor duckyDiskMotor;
    private DcMotor intakeMotor;

    //    private Servo leftArmServo;
//    private Servo rightArmServo;
    //Finger Servos
    private Servo leftGrab;
    private Servo rightGrab;

    private int debugIncrement = 0;
    //    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime elapsedTime = new ElapsedTime();
    private ElapsedTime Timer = new ElapsedTime();

    private BNO055IMU imu = null;   // Gyro device

    Orientation angles;
    Acceleration gravity;

    Orientation lastAngles = new Orientation();

    private double robotAngleToField = 0;
    private double robotAngleToTarget = 0;
    private double motorAngleToTarget = 0;
    private double zPIDAngle = 0;


    double robotXMultiThread = 0;
    double robotYMultiThread = 0;


    double distanceToTarget = 10;
    double prevDistanceToTarget = 20;
    double distanceToTargetAngle = 0;
    double distanceToEndPoint = 10;



    double deltaTime = 0.024;
    double startTime = 0;
    double oldTime = 0;
    double loopTime = 0;
    double loopTimeW = 0;
    double currentTime = 0;
    double parkTimer = 0;


    double xPower = 0;
    double yPower = 0;
    double zPower = 0;
    double zPowerStart = -0.25;
    double zPowerIncrease = 0.075;
    public double angleStart = 0;
    double robotFaceAngle = 0;
    double angleIncrement = 0;

    boolean useZPID = false;

    double zSpeedTargetAngle = 0;

    double robotTargetSpeed = 0.5;
    double robotTargetAngle = 0;
    double robotTurnSpeed = 0;

    double robotSpeed = 0;
    double decelRate =0;
    double robotTargetSpeedIPS = 0;
    double robotTargetSpeedPID = 0;

    double frontLeftMotorPower = 0;
    double frontRightMotorPower = 0;
    double rearLeftMotorPower = 0;
    double rearRightMotorPower = 0;
    double maxPowerEndPP = 0;


    //For ArmPID

    double propErrorArm = 0;
    double intErrorArm = 0;
    double loopTimeArm = 0;
    double prevDerErrorArm = 0;
    double derErrorArm = 0;
    double angletoleranceArm = 0;
    double motorPowerArm = 0;
    double currentTimeArm = 0;
    double oldTimeArm = 0;
    double deltaTimeArm = 0;
    double startTimeArm = 0;
    public static double armKp = 0.01;//0.005
    public static double armKi = 0.00;
    public static double armKd = 0.00;//0.00005
    public static double maxPowerArm = 0.75;

    private ElapsedTime armElapsedTime=new ElapsedTime();

    public static double HOME_MAX_POWER = 0.2; //Updated 11_09 was 0.2

    //    volatile double armHoldStartTime = 0.0;
    double armHoldStartTime = 0.0;

    private ElapsedTime clawReleaseDelayTime=new ElapsedTime();


    //


    //For ArmsPID Only 11_08

    double propErrorArmOnly = 0;
    double intErrorArmOnly = 0;
    double loopTimeArmOnly = 0;
    double prevDerErrorArmOnly = 0;
    double derErrorArmOnly = 0;
    double angletoleranceArmOnly = 0;
    double motorPowerArmOnly = 0;
    double currentTimeArmOnly = 0;
    double oldTimeArmOnly = 0;
    double deltaTimeArmOnly = 0;
    double startTimeArmOnly = 0;
    public static double armKpOnly = 0.01;//0.01
    public static double armKiOnly = 0.0;
    public static double armKdOnly = 0.0;
    public static double maxPowerArmOnly = 0.75;

    public static double HOME_MAX_POWER_ARMS_ONLY = 0.2;


    //For TURN PID

    boolean onTarget = false;
    public static double turnPIDKp = 0.01;
    public static double turnPIDKi = 0;
    public static double turnPIDKd = 0.002;

    public static double turnPIDpropError = 0;
    public static double turnPIDintError = 0;
    public static double turnPIDderError = 0;
    public static double turnPIDprevDerError = 0;
    public static double turnPIDangletolerance = 5.0;
    double turnPIDcurrentTime = 0;
    double turnPIDdeltaTime = 0;
    double turnPIDstartTime = 0;
    double turnPIDoldTime = 0;
    double turnPIDloopTime = 0;

    double turnPIDmotorPower = 0;

    double motorPowerOutput = 0;

    double turnPIDtimer = 0.0;


    //for Jusnoor's code
    double prevTickTime = 0;
    int prevLeft = 0, prevRight = 0, prevLeftB = 0, prevRightB = 0;
    private final double wheelDiameter = 3.54331; // For omni wheels we are using
    private final double wheelMountAngle = 45.0; //For current drivetrain
    private final double GEAR_RATIO = 20.0 / 15.0;  // Gear ratio
    private final double ticksPerRotation = 540.0; //For omni wheels we are using
    public double [] Velocities = new double[5];
    double maxVelocity = 0.0;
    double maxAcceleration = 0.0;

    final double COUNTS_PER_INCH = 194.044;

    //teamcode.RobotUtilities.Odometry

    OdometryGlobalCoordinatePositionNERD globalPositionUpdate ;
    Thread positionThread;

    public void startOdometryThread(){
        globalPositionUpdate = new OdometryGlobalCoordinatePositionNERD(leftEncoder, rightEncoder, backEncoder, imu, COUNTS_PER_INCH, 50);
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

    }

    public void stopOdometryThread(){

        try {
            positionThread.stop();
        }catch (Exception e){
            //Nothing to do
        }

//        globalPositionUpdate.stop();

    }

    /**
     * Constructor to create NerdBOT object
     * <p>
     * Creates a new NerdBOT object and assigns the hardwareMap provided by caller
     *
     * @param opmode Hardware Map provided by the calling OpMode.
     *               NerdBOT takes an opmode object so that it can get the hardwareMap.     *
     */

    public PurePursuitRobotMovement6_Turn_MultiThread (LinearOpMode opmode) {
        this.opmode = opmode;
        this.hardwareMap = opmode.hardwareMap;
    }

    //Function to initialize hardware components.

    public void setDebug(boolean debugFlag){
        this.debugFlag=debugFlag;
    }


    public void initializeHardware(){

        //Initialize Motors

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        this.frontLeftMotor = this.hardwareMap.get(DcMotor.class, "Front_Left_Motor");
        this.frontRightMotor = this.hardwareMap.get(DcMotor.class, "Front_Right_Motor");
        this.rearLeftMotor = this.hardwareMap.get(DcMotor.class, "Rear_Left_Motor");
        this.rearRightMotor = this.hardwareMap.get(DcMotor.class, "Rear_Right_Motor");

        this.frontEncoder = this.hardwareMap.get(DcMotor.class, "leftArmMotor");
        this.rightEncoder = this.hardwareMap.get(DcMotor.class, "rightArmMotor");
        this.leftEncoder = this.hardwareMap.get(DcMotor.class, "Ducky_Disk");
        this.backEncoder = this.hardwareMap.get(DcMotor.class, "Intake");




//        this.wobbleServo = hardwareMap.get(Servo.class, "wobble_Goal_Servo");
//        this.indexingServo = hardwareMap.get(Servo.class, "indexingServo");

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        this.imu = this.hardwareMap.get(BNO055IMU.class, "imu");
        this.imu.initialize(parameters);
        this.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        resetAngle();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

//        this.frontEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.backEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        this.frontEncoder.setVelocityPIDFCoefficients(200, 0.1, 0, 16);

        this.frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.frontEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.backEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.frontEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
//        this.rightEncoder.setDirection(DcMotor.Direction.REVERSE);
//        this.leftEncoder.setDirection(DcMotor.Direction.REVERSE);
//        this.backEncoder.setDirection(DcMotor.Direction.REVERSE);

        this.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
//        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        leftGrab = hardwareMap.get(Servo.class, "leftGrab");
        rightGrab = hardwareMap.get(Servo.class, "rightGrab");

        //Positions to get in the intake. This is initial position we will be at the beginning.

//        leftArmServo.setPosition(0.3);
//        rightArmServo.setPosition(0.7);
        leftGrab.setPosition(FingerPositions.GRAB.getLeftFingerPosition());
        rightGrab.setPosition(FingerPositions.GRAB.getRightFingerPosition());

        robotXMultiThread = 0;
        robotYMultiThread = 0;

        prevDistanceToTarget = 20;

        //Initializes motors (obvi)
//        this.wobbleMotor = hardwareMap.get(DcMotor.class, "Left");
//        this.wobbleServo = hardwareMap.get(Servo.class, "wobble_Goal_Servo");


    }


    private void resetAngle() {


        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        robotAngleToField = 0;
    }

    //Function to get the angle of the Gyro sensor
    private double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        robotAngleToField += deltaAngle;

        lastAngles = angles;

        return robotAngleToField;
    }

    public double [] getVelocityForCurrentLoop() {
        int leftCurrent = frontLeftMotor.getCurrentPosition();
        int rightCurrent = frontRightMotor.getCurrentPosition();
        int leftBCurrent = rearLeftMotor.getCurrentPosition();
        int rightBCurrent = rearRightMotor.getCurrentPosition();

        double tTime = elapsedTime.seconds();
        double deltaTickTime = tTime - prevTickTime;
        prevTickTime = tTime;

        double leftInches = ticksToInches((int) (leftCurrent - prevLeft), wheelDiameter, wheelMountAngle);
        double rightInches = ticksToInches((int) (rightCurrent - prevRight), wheelDiameter, wheelMountAngle);
        double leftBInches = ticksToInches((int) (leftBCurrent - prevLeftB), wheelDiameter, wheelMountAngle);
        double rightBInches = ticksToInches((int) (rightBCurrent - prevRightB), wheelDiameter, wheelMountAngle);
        double frontLeftVelocity = leftInches / deltaTickTime;
        double frontRightVelocity = rightInches / deltaTickTime;
        double rearLeftVelocity = leftBInches / deltaTickTime;
        double rearRightVelocity = rightBInches / deltaTickTime;
        double avgVelocity = (frontLeftVelocity + frontRightVelocity + rearLeftVelocity + rearRightVelocity) / 4;
        double currentAcceleration = avgVelocity / deltaTickTime;
        if (currentAcceleration > maxAcceleration) {
            maxAcceleration = currentAcceleration;
        }
        if (avgVelocity > maxVelocity) {
            maxVelocity = avgVelocity;
        }
        int leftDeltaTicks = leftCurrent - prevLeft;
        int rightDeltaTicks = rightCurrent - prevRight;
        int leftBDeltaTicks = leftBCurrent - prevLeftB;
        int rightBDeltaTicks = rightBCurrent - prevRightB;
        double avgDeltaTicks = (leftDeltaTicks + rightDeltaTicks + leftBDeltaTicks + rightBDeltaTicks) / 4;
        prevLeft = leftCurrent;
        prevRight = rightCurrent;
        prevLeftB = leftBCurrent;
        prevRightB = rightBCurrent;
        Velocities[0] = frontLeftVelocity;
        Velocities[1] = frontRightVelocity;
        Velocities[2] = rearLeftVelocity;
        Velocities[3] = rearRightVelocity;
        Velocities[4] = deltaTickTime; // added delta time to be used in PID

        return Velocities;  //inches per second
    }

    public double ticksToInches(int ticks, double wheelDiameter, double wheelMountAngle) {
        double circum = wheelDiameter * Math.PI;
        double numberofWheelRotations = (double) ticks / ticksPerRotation;
        double wheelDistanceToTravel = numberofWheelRotations * circum;
//        double straightDistanceToTravel = wheelDistanceToTravel; // (Math.cos(Math.toRadians(wheelMountAngle)) * GEAR_RATIO);
        return wheelDistanceToTravel;
    }

    private double powerToSpeed (double motorPower){
        double wheelSpeedRPS = motorPower * 6000 / 60 / 19.2; //convert motor power to wheel rotations per second, 6000 rpm max motor speed, 60 seconds in a minute.
        double wheelSpeedIPS = wheelSpeedRPS * wheelDiameter * Math.PI; //
        return wheelSpeedIPS;

    }

    private double speedToPower (double wheelSpeed){
        double motorPower = wheelSpeed * 60 * 19.2 / 6000 / wheelDiameter / Math.PI;
//        double wheelSpeedRPS = motorPower * 6000 / 60 / 19.2; //convert motor power to wheel rotations per second, 6000 rpm max motor speed, 60 seconds in a minute.
//        double wheelSpeedIPS = wheelSpeedRPS * wheelDiameter * Math.PI; //
        return motorPower;

    }

    boolean distanceTargetReached(double distanceToTarget, double parkRadius){

        boolean onDistanceTarget = false;

        if (distanceToTarget < parkRadius){
            onDistanceTarget = true;
        }


        return  onDistanceTarget;
    }

    boolean distanceTargetReachedPark (double distanceToTarget, double deltaTime, double parkDistance, double parkTime){

        boolean onDistanceTarget = false;

        if (distanceToTarget < parkDistance){
            parkTimer += deltaTime;
        }

        if (distanceToTarget < parkDistance && parkTimer > parkTime){
            onDistanceTarget = true;
        }

        return  onDistanceTarget;
    }

    public void followCurve(ArrayList<CurvePoint> allPoints, double zPowerFF, double distanceToPark, double parkAngleTarget, double parkRadius){

        startTime = elapsedTime.seconds();
        oldTime = startTime;
        distanceToEndPoint = 10;
        NerdPID_PurePursuit.resetIntError();
        zPowerStart = zPowerFF;
        zPowerIncrease = 0.075;
        angleStart = getAngle() + 90;
        angleIncrement = ((parkAngleTarget - angleStart) / (distanceToEndPoint - distanceToPark));
        robotFaceAngle = angleStart;
//11_15
//        OdometryGlobalCoordinatePositionNERD globalPositionUpdate = new OdometryGlobalCoordinatePositionNERD(leftEncoder, rightEncoder, backEncoder, imu, COUNTS_PER_INCH, 75);
//        Thread positionThread = new Thread(globalPositionUpdate);
//        positionThread.start();

        while (this.opmode.opModeIsActive() && !this.opmode.isStopRequested() && !distanceTargetReached(distanceToEndPoint, parkRadius)) {

            robotXMultiThread = globalPositionUpdate.returnXCoordinate();
            robotYMultiThread = globalPositionUpdate.returnYCoordinate();

            CurvePoint followMe = getFollowPointPath(allPoints, new PointPP(robotXMultiThread, robotYMultiThread),
                    allPoints.get(0).followDistance);

            CurvePoint endPoint = getEndPoint(allPoints, new PointPP(robotXMultiThread, robotYMultiThread),
                    allPoints.get(0).followDistance);

            CurvePoint startPath = getStartPath(allPoints, new PointPP(robotXMultiThread, robotYMultiThread),
                    allPoints.get(0).followDistance);

            distanceToEndPoint = Math.hypot(endPoint.x - robotXMultiThread, endPoint.y - robotYMultiThread);

            if(distanceToEndPoint < distanceToPark){
                goToPositionEndPP(endPoint.x, endPoint.y, 1.0, parkAngleTarget, 0.2, distanceToPark, new PointPP(robotXMultiThread, robotYMultiThread));
            }
            else {
                goToPositionPP(followMe.x, followMe.y, followMe.moveSpeed, zPowerFF, followMe.turnSpeed, parkAngleTarget, distanceToPark, startPath.x, startPath.y, endPoint.x, endPoint.y, new PointPP(robotXMultiThread, robotYMultiThread));
            }


        }
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
//11_15
//        globalPositionUpdate.stop();
    }



    public void followCurveArm(ArrayList<CurvePoint> allPoints, double zPowerFF, double distanceToPark, double parkAngleTarget,
                               double parkRadius, ArmShoulderPositions initialShoulderPosition, ArmShoulderPositions targetShoulderPosition,
                               FingerPositions initialFingerPosition, FingerPositions endFingerPosition,
                               double armDelay, double armHoldPositionTime, String motor, double power){

        resetArmVariables();

        startTime = elapsedTime.seconds();
        oldTime = startTime;
        distanceToEndPoint = 10;
        NerdPID_PurePursuit.resetIntError();
        zPowerStart = zPowerFF;
        zPowerIncrease = 0.075;
        angleStart = getAngle() + 90;
        angleIncrement = ((parkAngleTarget - angleStart) / (distanceToEndPoint - distanceToPark));
        robotFaceAngle = angleStart;

        //ARM 11_08

        startTimeArm = armElapsedTime.seconds();
        oldTimeArm = startTimeArm;




        boolean finalArmTargetReached = false;
        double armDelayTimer = 0.0;
        if(armDelay >0)
            armDelayTimer = armElapsedTime.seconds();


        armHoldStartTime = 0.0;

        leftGrab.setPosition(initialFingerPosition.getLeftFingerPosition());
        rightGrab.setPosition(initialFingerPosition.getRightFingerPosition());


        while (this.opmode.opModeIsActive() && !this.opmode.isStopRequested() &&
                !distanceTargetReached(distanceToEndPoint, parkRadius) &&
                !finalArmTargetReached && !isArmHoldTimeReached(armHoldPositionTime) ) {

            //ARM_11_08

            RobotLog.d("NOVI - followCurveArm %d" ,++debugIncrement);

            currentTimeArm = armElapsedTime.seconds();
            loopTimeArm = currentTimeArm - oldTimeArm;
            oldTimeArm = currentTimeArm;
            deltaTimeArm = currentTimeArm - startTimeArm;

//
            robotXMultiThread = globalPositionUpdate.returnXCoordinate();
            robotYMultiThread = globalPositionUpdate.returnYCoordinate();

            CurvePoint followMe = getFollowPointPath(allPoints, new PointPP(robotXMultiThread, robotYMultiThread),
                    allPoints.get(0).followDistance);
            RobotLog.d("NOVI - followCurveArm %d" ,++debugIncrement);


            CurvePoint endPoint = getEndPoint(allPoints, new PointPP(robotXMultiThread, robotYMultiThread),
                    allPoints.get(0).followDistance);

            RobotLog.d("NOVI - followCurveArm %d" ,++debugIncrement);


            CurvePoint startPath = getStartPath(allPoints, new PointPP(robotXMultiThread, robotYMultiThread),
                    allPoints.get(0).followDistance);

            RobotLog.d("NOVI - followCurveArm %d" ,++debugIncrement);


            distanceToEndPoint = Math.hypot(endPoint.x - robotXMultiThread, endPoint.y - robotYMultiThread);

            RobotLog.d("NOVI - followCurveArm %d" ,++debugIncrement);

            if(distanceToEndPoint < distanceToPark){
                goToPositionEndPP(endPoint.x, endPoint.y, 1.0, parkAngleTarget, 0.2, distanceToPark, new PointPP(robotXMultiThread, robotYMultiThread));
                RobotLog.d("NOVI - followCurveArm %d" ,++debugIncrement);

            }
            else {
                goToPositionPP(followMe.x, followMe.y, followMe.moveSpeed, zPowerFF, followMe.turnSpeed, parkAngleTarget, distanceToPark, startPath.x, startPath.y, endPoint.x, endPoint.y, new PointPP(robotXMultiThread, robotYMultiThread));
                RobotLog.d("NOVI - followCurveArm %d" ,++debugIncrement);

            }

            //Skip the ARM portion if a delay is requested.
            if(armDelay > 0){
                if((armElapsedTime.seconds() - armDelayTimer) < armDelay) continue;
            }

            //ARM Start
            double armPidOutput = 0.0;
            double armMotorsign = 1.0;
            double armMotorPower = 0.0;



            armPidOutput = armPID(targetShoulderPosition.getArmTarget(), frontEncoder.getCurrentPosition()*-1);

            RobotLog.d("NERD FRONT ONE #### FollowCurveArm - target %d, currentArmPosition %d, armPIDOut %f", targetShoulderPosition.getArmTarget(), frontEncoder.getCurrentPosition(),armPidOutput);

            RobotLog.d("NOVI - followCurveArm %d" ,++debugIncrement);

            armMotorsign = Math.signum(armPidOutput);

            if (Math.abs(armPidOutput) > targetShoulderPosition.getMaxPower()) {
                armMotorPower = armMotorsign * targetShoulderPosition.getMaxPower();
            } else {
                armMotorPower = armPidOutput;
            }

            RobotLog.d("NOVI - followCurveArm %d" ,++debugIncrement);

            RobotLog.d("NERDBLUEAUTON Motor powers %f, LooptimeARM %f", armMotorPower, loopTimeArm);
            frontEncoder.setPower(-armMotorPower);
            rightEncoder.setPower(-armMotorPower);
        }
        if((distanceTargetReached(distanceToEndPoint,parkRadius) && isArmTargetReached(targetShoulderPosition,frontEncoder.getCurrentPosition()))){
            finalArmTargetReached = true;
            if(armHoldPositionTime > 0)
                armHoldStartTime = armElapsedTime.seconds();
            RobotLog.d("NERD_11_08  Setting Finger Positions#### FollowCurveArm - Arm Hold Timer Started originalArmTargetPosition %d, currentArmTargetPosition %d, frontEncoder.getCurrentPosition %d",
                    targetShoulderPosition.getArmTarget(),targetShoulderPosition.getArmTarget(),frontEncoder.getCurrentPosition() );

            RobotLog.d("NOVI - followCurveArm %d" ,++debugIncrement);


            leftGrab.setPosition(endFingerPosition.getLeftFingerPosition());
            rightGrab.setPosition(endFingerPosition.getRightFingerPosition());

            RobotLog.d("NOVI - followCurveArm %d" ,++debugIncrement);



        }

        //ARM End

        //ARM 11_08

        RobotLog.d("NOVI - followCurveArm %d" ,++debugIncrement);

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);

    }


    public void setFingerPositions(FingerPositions targetFingerPosition)
    {
        this.leftGrab.setPosition(targetFingerPosition.getLeftFingerPosition());
        this.rightGrab.setPosition(targetFingerPosition.getRightFingerPosition());
    }

    public void setFingerPositions(FingerPositions targetFingerPosition, double seconds)
    {
        ElapsedTime clawTime = new ElapsedTime();
        while (this.opmode.opModeIsActive() && clawTime.seconds() < seconds
        ) {
            this.leftGrab.setPosition(targetFingerPosition.getLeftFingerPosition());
            this.rightGrab.setPosition(targetFingerPosition.getRightFingerPosition());
        }
    }

    public boolean isArmHoldTimeReached(double armHoldTime){
        boolean armHoldTimeReached = false;
        if(armHoldStartTime > 0){
            if((armElapsedTime.seconds() - armHoldStartTime) >= armHoldTime){
                armHoldTimeReached=true;
                //ARM 11_08
                RobotLog.d("NERD #### Resetting armHoldStartTime ");
                armHoldStartTime = 0;
            }
        }
        return armHoldTimeReached;
    }
    public boolean isArmTargetReached(ArmShoulderPositions targetPosition,  int currentPosition){
        boolean targetReached = false;

        RobotLog.d("NERD #### isArmTargetReached Target %d, current %d", targetPosition.getArmTarget(), currentPosition);
        if(Math.abs((targetPosition.getArmTarget() - Math.abs(currentPosition)) )<= 30){

            targetReached = true;
        }

        return targetReached;

    }

    public boolean isArmTargetReached(int targetPosition,  int currentPosition){
        boolean targetReached = false;

        RobotLog.d("NERD #### isArmTargetReached Target %d, current %d", targetPosition, currentPosition);
        if(Math.abs((targetPosition - Math.abs(currentPosition)) )<= 50){

            targetReached = true;
        }

//        clawReleaseDelayTime.reset();
        RobotLog.d("NERD #### targetReached %b", targetReached);
        return targetReached;

    }



    private CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, PointPP robotLocation, double followRadius){
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for(int i = 0; i < pathPoints.size() - 1; i++){
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endline = pathPoints.get(i + 1);

//            double[] robotPositionXYV = findDisplacementOptical();

            ArrayList<PointPP> intersections = MathFunctions.lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endline.toPoint());

            double closestAngle = 100000000;

            double closestDistance = 1000000;


            for(PointPP thisIntersection : intersections){
//                double angle = Math.atan2(thisIntersection.y - robotPositionXYV[5], thisIntersection.x - robotPositionXYV[4]);
//                //Need to check which angle is the same as worldAngle_rad...currently using getAngle() converted to radians
//                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle - (robotPositionXYV[6] * Math.PI / 180)));
//
//                if(deltaAngle < closestAngle){
//                    closestAngle = deltaAngle;
//                    followMe.setPoint(thisIntersection);
                double distance = Math.hypot(thisIntersection.x - robotLocation.x, thisIntersection.y - robotLocation.y);

                if(distance < closestDistance){
                    closestDistance = distance;
                    followMe.setPoint(thisIntersection);
                }
            }
        }


        return followMe;
    }

    private CurvePoint getEndPoint(ArrayList<CurvePoint> pathPoints, PointPP robotLocation, double followRadius){

        int endPathCount = pathPoints.size() - 2;
        CurvePoint endPoint = new CurvePoint(pathPoints.get(endPathCount));

        return endPoint;
    }

    private CurvePoint getStartPath(ArrayList<CurvePoint> pathPoints, PointPP robotLocation, double followRadius){

        CurvePoint startPath = new CurvePoint(pathPoints.get(0));

        return startPath;
    }


    public void goToPositionPP(double x, double y, double movementSpeed, double zPowerFeedForward, double turnSpeed, double parkAngleTarget, double parkDistance,
                               double robotPositionXStart, double robotPositionYStart, double endPointX, double endPointY, PointPP robotLocationMT){

        currentTime = elapsedTime.seconds();
        loopTime = currentTime - oldTime;
        oldTime = currentTime;

        distanceToTarget = Math.hypot(x - robotLocationMT.x, y - robotLocationMT.y);

        double absoluteAngleToTarget = Math.atan2(y - robotLocationMT.y, x - robotLocationMT.x) * 180 / Math.PI;

        robotTargetAngle = absoluteAngleToTarget;

        motorAngleToTarget = MathFunctions.AngleWrapDeg((robotTargetAngle - 45) - getAngle());

        xPower = Math.cos(motorAngleToTarget * 3.14 / 180) * movementSpeed;
        yPower = Math.sin(motorAngleToTarget * 3.14 / 180) * movementSpeed;

        double relativeTurnAngle = MathFunctions.AngleWrapDeg(robotTargetAngle - (getAngle() + 90));

        double distanceFromStart = Math.hypot(robotLocationMT.x - robotPositionXStart, robotLocationMT.y - robotPositionYStart);
        double distanceAtStart = Math.hypot(endPointX - robotPositionXStart, endPointY - robotPositionYStart);
        angleIncrement = (parkAngleTarget - angleStart) / (distanceAtStart - parkDistance - 10);

        if (angleStart < parkAngleTarget){
            robotFaceAngle = Range.clip(angleIncrement * distanceFromStart + angleStart, angleStart, parkAngleTarget);
        }else if (angleStart > parkAngleTarget){
            robotFaceAngle = Range.clip(angleIncrement * distanceFromStart + angleStart, parkAngleTarget, angleStart);
        }

        zPIDAngle = 90 + getAngle();
        robotTurnSpeed = NerdPID_PurePursuit.zPowerDrive(robotFaceAngle, zPIDAngle, loopTime);

        double robotTurnSpeedFF = Range.clip((zPowerStart + zPowerIncrease), -0.5, 0);
        zPowerStart = robotTurnSpeedFF;


        zPower = Range.clip((robotTurnSpeed + zPowerStart), -0.3, 0.3);

        frontLeftMotorPower = -xPower + zPower;
        rearRightMotorPower = xPower + zPower;
        frontRightMotorPower = yPower + zPower;
        rearLeftMotorPower = -yPower + zPower;

        double frontLeftMotorTarget = powerToSpeed(frontLeftMotorPower);
        double rearRightMotorTarget = powerToSpeed(rearRightMotorPower);
        double frontRightMotorTarget = powerToSpeed(frontRightMotorPower);
        double rearLeftMotorTarget = powerToSpeed(rearLeftMotorPower);

        getVelocityForCurrentLoop();

        double frontLeftMotorSpeed = Velocities[0];
        double rearRightMotorSpeed = Velocities[3];
        double frontRightMotorSpeed = Velocities[1];
        double rearLeftMotorSpeed = Velocities[2];
        double deltaTime = Velocities[4];

        double [] motorSpeedCommand = NerdVelocityFollowing.velocityFollowing(frontLeftMotorTarget, rearRightMotorTarget,
                frontRightMotorTarget, rearLeftMotorTarget, frontLeftMotorSpeed, rearRightMotorSpeed, frontRightMotorSpeed,
                rearLeftMotorSpeed, deltaTime);

        frontLeftMotor.setPower(motorSpeedCommand[0]);
        rearRightMotor.setPower(motorSpeedCommand[3]);
        frontRightMotor.setPower(motorSpeedCommand[1]);
        rearLeftMotor.setPower(motorSpeedCommand[2]);

        if (debugFlag) {
            RobotLog.d("goToPositionPP - runTime %f, deltaTime %f, robotLocationX %f, robotLocationY %f, robotFaceAngle %f, robotAngle %f",
                    currentTime, deltaTime, robotLocationMT.x, robotLocationMT.y, robotFaceAngle, zPIDAngle);
        }


    }

    public void goToPositionEndPP(double x, double y, double movementSpeed, double targetAngleForPark, double turnSpeed, double distanceToEndPoint, PointPP robotLocationMT){

        distanceToTargetAngle = getAngle() + 90;

        currentTime = elapsedTime.seconds();
        loopTime = currentTime - oldTime;
        oldTime = currentTime;

        distanceToTarget = Math.hypot(x - robotLocationMT.x, y - robotLocationMT.y);

        double absoluteAngleToTarget = Math.atan2(y - robotLocationMT.y, x - robotLocationMT.x) * 180 / Math.PI;

        if (distanceToEndPoint > 20) {
            if (distanceToTarget < distanceToEndPoint && distanceToTarget > (distanceToEndPoint / 2)) {
                robotTargetSpeed = Range.clip(movementSpeed * (distanceToTarget - (distanceToEndPoint / 2)) / 30, 0.4, 1);
                prevDistanceToTarget = distanceToEndPoint / 2;
            } else if (distanceToTarget < (distanceToEndPoint / 2)) {
                robotTargetSpeedPID = NerdPID_PurePursuit.movementSpeedPID(distanceToTarget, prevDistanceToTarget, loopTime);
                prevDistanceToTarget = distanceToTarget;
                robotTargetSpeed = robotTargetSpeedPID;
            }
        }else{
            robotTargetSpeedPID = NerdPID_PurePursuit.shortParkPID(distanceToTarget, prevDistanceToTarget, loopTime);
            prevDistanceToTarget = distanceToTarget;
            robotTargetSpeed = robotTargetSpeedPID;
        }

        robotTargetAngle = absoluteAngleToTarget;

        robotAngleToTarget = MathFunctions.AngleWrapDeg(robotTargetAngle - getAngle());
        motorAngleToTarget = MathFunctions.AngleWrapDeg((robotTargetAngle - 45) - getAngle());

        xPower = Math.cos(motorAngleToTarget * 3.14 / 180) * robotTargetSpeed;
        yPower = Math.sin(motorAngleToTarget * 3.14 / 180) * robotTargetSpeed;



//            double relativeTurnAngle = MathFunctions.AngleWrapDeg(robotAngleToTarget - 180 + preferredAngle);
        double relativeTurnAngle = MathFunctions.AngleWrapDeg(targetAngleForPark - 90 - getAngle());
        double targetParkAngle = targetAngleForPark;
        zPIDAngle = 90 + getAngle();
        zPower = Range.clip(NerdPID_PurePursuit.zPowerPark(targetParkAngle, zPIDAngle, loopTime), -0.3, 0.3);

        frontLeftMotorPower = -xPower + zPower;
        rearRightMotorPower = xPower + zPower;
        frontRightMotorPower = yPower + zPower;
        rearLeftMotorPower = -yPower + zPower;

        if(Math.abs(frontLeftMotorPower) > Math.abs(frontRightMotorPower) && Math.abs(frontLeftMotorPower) > Math.abs(rearLeftMotorPower) && Math.abs(frontLeftMotorPower) > Math.abs(rearRightMotorPower)){
            maxPowerEndPP = Math.abs(frontLeftMotorPower);
        }
        else if (Math.abs(frontRightMotorPower) > Math.abs(frontLeftMotorPower) && Math.abs(frontRightMotorPower) > Math.abs(rearLeftMotorPower) && Math.abs(frontRightMotorPower) > Math.abs(rearRightMotorPower)){
            maxPowerEndPP = Math.abs(frontRightMotorPower);
        }
        else if(Math.abs(rearRightMotorPower) > Math.abs(frontRightMotorPower) && Math.abs(rearRightMotorPower) > Math.abs(rearLeftMotorPower) && Math.abs(rearRightMotorPower) > Math.abs(frontLeftMotorPower)){
            maxPowerEndPP = Math.abs(rearRightMotorPower);
        }
        if(Math.abs(rearLeftMotorPower) > Math.abs(frontRightMotorPower) && Math.abs(rearLeftMotorPower) > Math.abs(frontLeftMotorPower) && Math.abs(rearLeftMotorPower) > Math.abs(rearRightMotorPower)) {
            maxPowerEndPP = Math.abs(rearLeftMotorPower);
        }

        if(maxPowerEndPP > 1) {
            frontLeftMotorPower /= maxPowerEndPP;
            frontRightMotorPower /= maxPowerEndPP;
            rearLeftMotorPower /= maxPowerEndPP;
            rearRightMotorPower /= maxPowerEndPP;
        }


        double frontLeftMotorTarget = powerToSpeed(frontLeftMotorPower);
        double rearRightMotorTarget = powerToSpeed(rearRightMotorPower);
        double frontRightMotorTarget = powerToSpeed(frontRightMotorPower);
        double rearLeftMotorTarget = powerToSpeed(rearLeftMotorPower);

        getVelocityForCurrentLoop();

        double frontLeftMotorSpeed = Velocities[0];
        double rearRightMotorSpeed = Velocities[3];
        double frontRightMotorSpeed = Velocities[1];
        double rearLeftMotorSpeed = Velocities[2];
        double deltaTime = Velocities[4];

        double [] motorSpeedCommand = NerdVelocityFollowing.velocityFollowing(frontLeftMotorTarget, rearRightMotorTarget,
                frontRightMotorTarget, rearLeftMotorTarget, frontLeftMotorSpeed, rearRightMotorSpeed, frontRightMotorSpeed,
                rearLeftMotorSpeed, deltaTime);

        frontLeftMotor.setPower(motorSpeedCommand[0]);
        rearRightMotor.setPower(motorSpeedCommand[3]);
        frontRightMotor.setPower(motorSpeedCommand[1]);
        rearLeftMotor.setPower(motorSpeedCommand[2]);


        // channels to record to debug optical encoder field centric driving
        if (debugFlag) {
            RobotLog.d("goToPositionEndPP - runTime %f, deltaTime %f, robotLocationX %f, robotLocationY %f, targetParkAngle %f, robotAngle %f",
                    currentTime, deltaTime, robotLocationMT.x, robotLocationMT.y, targetParkAngle, zPIDAngle);
        }

    }

    public void runMotor(String motor, double power, double timeInSeconds){

        double motorStarTime = elapsedTime.seconds();
        double motorRunTime = 0.0;
        DcMotor currentMotor = null;

        while (this.opmode.opModeIsActive() && motorRunTime <= timeInSeconds ){
            if (motor.equals("duckyDisc")){
                currentMotor = this.leftEncoder;
                runDriveMotors(0.2);
            }
            else if (motor.equals("intake")){
                currentMotor = this.backEncoder;
            }

            currentMotor.setPower(power);

            motorRunTime = elapsedTime.seconds() - motorStarTime;
        }
        currentMotor.setPower(0.0);

    }

    public void runMotor(String motor, double power){

        if (motor.equals("duckyDisc")){
            this.leftEncoder.setPower(power);
        }
        else if (motor.equals("intake")) {
            this.backEncoder.setPower(power);
        }


    }

    public void runDriveMotors(double power){

        frontLeftMotor.setPower(power);
        rearRightMotor.setPower(-power);
        frontRightMotor.setPower(power);
        rearLeftMotor.setPower(-power);

    }



    private double armPID(double targetValue, double currentValue) {


        propErrorArm = (targetValue - currentValue);

        intErrorArm += (targetValue - currentValue) * loopTimeArm; //11_08

        derErrorArm = ((targetValue - currentValue) - prevDerErrorArm) / loopTimeArm; //11_08
        prevDerErrorArm = targetValue - currentValue;

        if (Math.abs(targetValue - currentValue) < angletoleranceArm) {
//                    onTarget = true;
//                    runTest = false;
            motorPowerArm = propErrorArm * armKp + intErrorArm * armKi;
            intErrorArm = 0;
        } else {
            motorPowerArm = propErrorArm * armKp + intErrorArm * armKi + derErrorArm * armKd;
        }
//        if (Math.abs(targetValue - currentValue) < 0.5) {
//            motor1.setPower(0);
//       }
        double motorPowersin = Math.signum(motorPowerArm);
        if (Math.abs(motorPowerArm) > maxPowerArm) {
            motorPowerArm = maxPowerArm * motorPowersin;

        }
        return motorPowerArm;
    }



    private double armPIDOnly(double targetValue, double currentValue) {


        propErrorArmOnly = (targetValue - currentValue);

        intErrorArmOnly += (targetValue - currentValue) * loopTimeArmOnly;

        derErrorArmOnly = ((targetValue - currentValue) - prevDerErrorArmOnly) / loopTimeArmOnly;
        prevDerErrorArmOnly = targetValue - currentValue;

        if (Math.abs(targetValue - currentValue) < angletoleranceArmOnly) {
//                    onTarget = true;
//                    runTest = false;
            motorPowerArmOnly = propErrorArmOnly * armKpOnly + intErrorArmOnly * armKiOnly;
            intErrorArmOnly = 0;
        } else {
            motorPowerArmOnly = propErrorArmOnly * armKpOnly + intErrorArmOnly * armKiOnly + derErrorArmOnly * armKdOnly;
        }
//        if (Math.abs(targetValue - currentValue) < 0.5) {
//            motor1.setPower(0);
//       }
        double motorPowersin = Math.signum(motorPowerArmOnly);
        if (Math.abs(motorPowerArmOnly) > maxPowerArmOnly) {
            motorPowerArmOnly = maxPowerArmOnly * motorPowersin;

        }
        return motorPowerArmOnly;
    }


    public void moveArmsOnly(ArmShoulderPositions armTargetPosition, int change, FingerPositions fingerTargetPosition) {
        ElapsedTime clawRealsetime = new ElapsedTime();
        int newArmTargetPosition = armTargetPosition.getArmTarget() - change;
        startTimeArmOnly = elapsedTime.seconds();
        oldTimeArmOnly = startTimeArmOnly;
        ElapsedTime armTimeout = new ElapsedTime();
        boolean armTargetReached = false;
        boolean clawReleased=false;
        boolean clawTimerStarted = false;
        while (this.opmode.opModeIsActive()  ) {

            RobotLog.d("NOVI - moveArmsOnly %d" ,++debugIncrement);


            if(!armTargetReached){
                armTargetReached= isArmTargetReached(newArmTargetPosition, frontEncoder.getCurrentPosition());
            }

            RobotLog.d("NERD Arm Target Reached %b",armTargetReached );
            RobotLog.d("NOVI - moveArmsOnly %d" ,++debugIncrement);
            if(armTargetReached && !clawTimerStarted)   {
                clawReleaseDelayTime.reset();
                clawTimerStarted = true;
                RobotLog.d("NERD Claw Timer started %f",clawReleaseDelayTime.seconds() );
                RobotLog.d("NOVI - moveArmsOnly %d" ,++debugIncrement);

            }
            if (!armTargetReached) {
                currentTimeArmOnly = elapsedTime.seconds();
                loopTimeArmOnly = currentTimeArmOnly - oldTimeArmOnly;
                oldTimeArmOnly = currentTimeArmOnly;
                deltaTimeArmOnly = currentTimeArmOnly - startTimeArmOnly;

                RobotLog.d("NOVI - moveArmsOnly %d" ,++debugIncrement);

                //ARM Start
                double armPidOutput = 0.0;
                double armMotorsign = 1.0;
                double armMotorPower = 0.0;

                armPidOutput = armPID(newArmTargetPosition, frontEncoder.getCurrentPosition() * -1); //11_08 check
                armMotorsign = Math.signum(armPidOutput);

                RobotLog.d("NOVI - moveArmsOnly %d" ,++debugIncrement);
                if (Math.abs(armPidOutput) > armTargetPosition.getMaxPower()) {
                    armMotorPower = armMotorsign * armTargetPosition.getMaxPower();
                } else {
                    armMotorPower = armPidOutput;
                }

                RobotLog.d("Arm Motor Power %f", armMotorPower );
                RobotLog.d("NOVI - moveArmsOnly %d" ,++debugIncrement);

                leftEncoder.setPower(-armMotorPower);
                rightEncoder.setPower(-armMotorPower); //11_08 check

                RobotLog.d("NOVI - moveArmsOnly %d" ,++debugIncrement);
//            leftArmServo.setPosition(armTargetPosition.getLeftWristServoPosition());
//            rightArmServo.setPosition(armTargetPosition.getRightWristServoPosition());

            }


            RobotLog.d("Before arm target reached - NERDClawDelayTime %f", clawReleaseDelayTime.seconds());

            RobotLog.d("NOVI - moveArmsOnly %d" ,++debugIncrement);
            if (armTargetReached )
                RobotLog.d("NOVI - moveArmsOnly %d" ,++debugIncrement);
            if(clawReleaseDelayTime.seconds()  < 0.5) {
                RobotLog.d("NOVI - moveArmsOnly %d" ,++debugIncrement);
                leftGrab.setPosition(fingerTargetPosition.getLeftFingerPosition());
                rightGrab.setPosition(fingerTargetPosition.getRightFingerPosition());
                RobotLog.d("After NERDClawDelayTime %f", clawReleaseDelayTime.seconds());
            }else{
                RobotLog.d("NOVI - moveArmsOnly %d" ,++debugIncrement);
                break;
            }

            if(armTimeout.seconds() > 1) {
                RobotLog.d("NOVI - moveArmsOnly %d" ,++debugIncrement);
                armTargetReached = true;
            }
        }

    }


    public void turnRobot (double targetAngle){
        turnPIDloopTime = 0.0;
        turnPIDtimer = elapsedTime.seconds();
        while (this.opmode.opModeIsActive() && !this.opmode.isStopRequested() && !turnPIDAngleTargetReached(targetAngle)) {

            //First calculate motor speeds for linear (x, y) motion


            turnPIDcurrentTime = elapsedTime.seconds();
            turnPIDloopTime = turnPIDcurrentTime - turnPIDoldTime;
            turnPIDoldTime = turnPIDcurrentTime;
            turnPIDdeltaTime = turnPIDcurrentTime - turnPIDstartTime;


            motorPowerOutput = turnPID(targetAngle);

            rearRightMotor.setPower(motorPowerOutput);
            frontLeftMotor.setPower(motorPowerOutput);
            frontRightMotor.setPower(motorPowerOutput);
            rearLeftMotor.setPower(motorPowerOutput);
        }
        rearRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
    }

    private boolean turnPIDAngleTargetReached(double targetAngleR){
        boolean angleTargetReached = false;
        if ((Math.abs(targetAngleR - 90 - getAngle()) < turnPIDangletolerance) || ((elapsedTime.seconds() - turnPIDtimer) >= 1.0)){
            angleTargetReached = true;
        }
        return angleTargetReached;
    }

    public double turnPID(double turnPIDrobotTargetAngle) {

        double turnPIDAngle = getAngle() + 90;

        turnPIDpropError = MathFunctions.AngleWrapDeg(turnPIDrobotTargetAngle - turnPIDAngle);

        if (Math.abs(turnPIDpropError) < 1) {
//                    Kp = 0.0940;
//                } else if (Math.abs(propError) >= 1 && Math.abs(propError) < 5) {
//                    Kp = 0.0480;
//                } else if (Math.abs(propError) >= 5 && Math.abs(propError) < 10) {
//                    Kp = 0.0240;
//                } else if (Math.abs(propError) >= 10 && Math.abs(propError) < 20) {
//                    Kp = 0.0120;
//                } else if (Math.abs(propError) >= 20 && Math.abs(propError) < 45) {
//                    Kp = 0.0085;
//                    Kd = 0.001;
//                } else if (Math.abs(propError) >= 45 && Math.abs(propError) < 90) {
//                    Kp = 0.0070;
//                } else if (Math.abs(propError) >= 90 && Math.abs(propError) < 180) {
//                    Kp = 0.0070;
        }
        turnPIDintError += turnPIDpropError * turnPIDloopTime;

        turnPIDderError = ((turnPIDpropError) - turnPIDprevDerError) / turnPIDloopTime;
        turnPIDprevDerError = turnPIDpropError;

        if (Math.abs(turnPIDrobotTargetAngle - turnPIDAngle) < turnPIDangletolerance) {
//                    onTarget = true;
//                    runTest = false;
            turnPIDmotorPower = turnPIDpropError * turnPIDKp + turnPIDintError * turnPIDKi;
            turnPIDintError = 0;
        }
        else    {
            turnPIDmotorPower = Range.clip(turnPIDpropError * turnPIDKp + turnPIDintError * turnPIDKi + turnPIDderError * turnPIDKd, -0.3, 0.3);
        }


        return turnPIDmotorPower;

    }

    public void resetITerm() {
        NerdVelocityFollowing.resetI();
    }

    public void resetTimers() {
        elapsedTime.reset();
        Timer.reset();
    }

    public void printI() {
        opmode.telemetry.addData("I gain", NerdVelocityFollowing.FLTotalError);
        opmode.telemetry.addData("I gain", NerdVelocityFollowing.FRTotalError);
        opmode.telemetry.addData("I gain", NerdVelocityFollowing.RLTotalError);
        opmode.telemetry.addData("I gain", NerdVelocityFollowing.RRTotalError);
        opmode.telemetry.update();
    }

    public void resetArmVariables(){

        propErrorArm = 0;
        intErrorArm = 0;
        loopTimeArm = 0;
        prevDerErrorArm = 0;
        derErrorArm = 0;
        angletoleranceArm = 0;
        motorPowerArm = 0;
        currentTimeArm = 0;
        oldTimeArm = 0;
        deltaTimeArm = 0;
        startTimeArm = 0;
    }


}



