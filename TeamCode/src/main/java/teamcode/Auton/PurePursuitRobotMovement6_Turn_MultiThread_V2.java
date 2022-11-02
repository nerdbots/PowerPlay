package teamcode.Auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import teamcode.RobotUtilities.ArmShoulderPositions;
import teamcode.RobotUtilities.FingerPositions;

import java.util.ArrayList;

import teamcode.RobotUtilities.Odometry.OdometryGlobalCoordinatePositionNERD;
import teamcode.RobotUtilities.core.PointPP;
import teamcode.RobotUtilities.Odometry.CurvePoint;
import teamcode.RobotUtilities.MathFunctions;
import teamcode.RobotUtilities.MathFunctions2;
import teamcode.RobotUtilities.NerdPID_PurePursuit;
import teamcode.RobotUtilities.NerdVelocityFollowing;

public class PurePursuitRobotMovement6_Turn_MultiThread_V2 {

    private boolean debugFlag=false;

    //We need an opmode to get the hardware map etc.

    private LinearOpMode opmode;

    private HardwareMap hardwareMap;

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;

    private DcMotor frontEncoder;
    private DcMotor rightEncoder;
    private DcMotor leftEncoder;
    private DcMotor backEncoder;

    private DcMotor duckyDiskMotor;
    private DcMotor intakeMotor;

//    private Servo leftArmServo;
//    private Servo rightArmServo;
    //Finger Servos
    private Servo leftGrab;
    private Servo rightGrab;

//    ColorSensor colorSensor;
    RevBlinkinLedDriver blinkinLedDriver;
    ElapsedTime IntakeTimer = new ElapsedTime();
    ElapsedTime IntakeTimer2 = new ElapsedTime();
    private boolean isBlockIn = false;

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

    //For CurvePoint
    double robotVectorAngle;


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
    public static double armKp = 0.005;//0.005
    public static double armKi = 0.00;
    public static double armKd = 0.00025;//0.00005
    public static double maxPowerArm = 0.4;

    private ElapsedTime armElapsedTime=new ElapsedTime();

    public static double HOME_MAX_POWER = 0.2; //Updated 11_09 was 0.2

    //    volatile double armHoldStartTime = 0.0;
    double armHoldStartTime = 0.0;

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
    public static double armKpOnly = 0.005;//0.01
    public static double armKiOnly = 0.0;
    public static double armKdOnly = 0.0002;
    public static double maxPowerArmOnly = 0.4;

    public static double HOME_MAX_POWER_ARMS_ONLY = 0.2;


    //For TURN PID

    boolean onTarget = false;
    public static double turnPIDKp = 0.015; //0.015
    public static double turnPIDKi = 0.0006;
    public static double turnPIDKd = 0.001; //0.001 //0.002

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


    //Pure Pursuit Path Counting Functions
    int pL; //Path Location indicator for pure pursuit
    int pLf; //Next path point indicator for pure pursuit
    boolean distanceReached = false;
    boolean distanceStarted = false;
    double pathSegment1Length = 0;
    double distanceToPoint = 0;
    double distanceToPointOld = 0;
    double distanceFromPoint = 0;
    double thisIntersection1XDisp = 0;
    double thisIntersection1YDisp = 0;
    double thisIntersection1XOld = 0;
    double thisIntersection1YOld = 0;
    double pathSegment1Mag = 0;
    double distanceFromPointPath = 0;



    //Wizards.exe odometry
    final double COUNTS_PER_INCH = 194.044;

    //Odometry

    OdometryGlobalCoordinatePositionNERD globalPositionUpdate ;
    Thread positionThread;

    public void startOdometryThread(){
        globalPositionUpdate = new OdometryGlobalCoordinatePositionNERD(leftEncoder, rightEncoder, backEncoder, imu, COUNTS_PER_INCH, 75);
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

    }

    public void stopOdometryThread(){

        try {
            positionThread.stop();
        }catch (Exception e){
            //Nothing to do
        }
    }

    /**
     * Constructor to create NerdBOT object
     * <p>
     * Creates a new NerdBOT object and assigns the hardwareMap provided by caller
     *
     * @param opmode Hardware Map provided by the calling OpMode.
     *               NerdBOT takes an opmode object so that it can get the hardwareMap.     *
     */

    public PurePursuitRobotMovement6_Turn_MultiThread_V2(LinearOpMode opmode) {
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


//        this.colorSensor = this.hardwareMap.get(ColorSensor.class, "colorSensor");
//        this.blinkinLedDriver = this.hardwareMap.get(RevBlinkinLedDriver.class, "led");



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
        this.rightEncoder.setDirection(DcMotor.Direction.REVERSE);
        this.leftEncoder.setDirection(DcMotor.Direction.REVERSE);
        this.backEncoder.setDirection(DcMotor.Direction.REVERSE);

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
        leftGrab.setPosition(0.53);
        rightGrab.setPosition(0.55);

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
        pL = 0;
        distanceFromPointPath = 0;


        while (this.opmode.opModeIsActive() && !this.opmode.isStopRequested() && !distanceTargetReached(distanceToEndPoint, parkRadius)) {

            robotXMultiThread = globalPositionUpdate.returnXCoordinate();
            robotYMultiThread = globalPositionUpdate.returnYCoordinate();



            CurvePoint followMe = getFollowPointPath(allPoints, new PointPP(robotXMultiThread, robotYMultiThread),
                    allPoints.get(0).followDistance, pL);

            CurvePoint endPoint = getEndPoint(allPoints, new PointPP(robotXMultiThread, robotYMultiThread),
                    allPoints.get(0).followDistance);

            CurvePoint startPath = getStartPath(allPoints, new PointPP(robotXMultiThread, robotYMultiThread),
                    allPoints.get(0).followDistance);

            distanceToEndPoint = Math.hypot(endPoint.x - robotXMultiThread, endPoint.y - robotYMultiThread);

            CurvePoint startSegment1 = allPoints.get(pL);
            CurvePoint endSegment1 = allPoints.get(pL + 1);

            ArrayList<PointPP> perpendicularIntersection1 = MathFunctions2.pathDistance(new PointPP(robotXMultiThread, robotYMultiThread), startSegment1.toPoint(), endSegment1.toPoint(), followMe.toPoint());

            for(PointPP thisIntersection1 : perpendicularIntersection1){
                pathSegment1Length = Math.sqrt(((endSegment1.x - startSegment1.x) * (endSegment1.x - startSegment1.x)) + (endSegment1.y - startSegment1.y) * (endSegment1.y - startSegment1.y));
                distanceToPoint = Math.hypot((endSegment1.x - thisIntersection1.x), (endSegment1.y - thisIntersection1.y));
//                distanceFromPoint = Math.hypot((startSegment1.x - thisIntersection1.x), (startSegment1.y - thisIntersection1.y));
//
//                thisIntersection1XDisp = thisIntersection1.x - thisIntersection1XOld;
//                thisIntersection1YDisp = thisIntersection1.y - thisIntersection1YOld;
//                thisIntersection1XOld = thisIntersection1.x;
//                thisIntersection1YOld = thisIntersection1.y;
//                pathSegment1Mag = Math.hypot(thisIntersection1XDisp, thisIntersection1YDisp);
//                distanceFromPointPath += pathSegment1Mag;

//                if (distanceToPoint < (pathSegment1Length / 5) || distanceFromPoint > pathSegment1Length || distanceFromPointPath > (pathSegment1Length * 0.9))
                if (distanceToPoint < (pathSegment1Length / 3) || distanceFromPoint > pathSegment1Length){
                    distanceReached = true;
                    pL = pL + 1;
                    distanceFromPointPath = 0;
                }else{
                    distanceReached = false;
                }
                distanceToPointOld = distanceToPoint;
            }

            if(distanceToEndPoint < distanceToPark){
                goToPositionEndPP(endPoint.x, endPoint.y, 1.0, parkAngleTarget, 0.2, distanceToPark, new PointPP(robotXMultiThread, robotYMultiThread));
            }
            else {
                goToPositionPP(followMe.x, followMe.y, followMe.moveSpeed, zPowerFF, followMe.turnSpeed, parkAngleTarget, distanceToPark, startSegment1.x, startSegment1.y, endSegment1.x, endSegment1.y,
                        new PointPP(robotXMultiThread, robotYMultiThread), startSegment1.slowDownTurnAmount, endSegment1.slowDownTurnAmount, pathSegment1Length, distanceToPoint);
            }

            if (debugFlag) {
                RobotLog.d("followCurve - runTime %f, deltaTime %f, pathSegment1Length %f, distanceToPoint %f, distanceFromPoint %f, distanceFromPointPath %f, pL %d",
                        currentTime, deltaTime, pathSegment1Length, distanceToPoint, distanceFromPoint, distanceFromPointPath, pL);
            }


        }
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);

    }


//
//    public void followCurveArm(ArrayList<CurvePoint> allPoints, double zPowerFF, double distanceToPark, double parkAngleTarget,
//                               double parkRadius, ArmShoulderPositions initialShoulderPosition, ArmShoulderPositions targetShoulderPosition,
//                               FingerPositions targetFingerPosition, FingerPositions endFingerPosition,
//                               double armDelay, double armHoldPositionTime, String motor, double power){
//
//        resetArmVariables();
//
//        startTime = elapsedTime.seconds();
//        oldTime = startTime;
//        distanceToEndPoint = 10;
//        NerdPID_PurePursuit.resetIntError();
//        zPowerStart = zPowerFF;
//        zPowerIncrease = 0.075;
//        angleStart = getAngle() + 90;
//        angleIncrement = ((parkAngleTarget - angleStart) / (distanceToEndPoint - distanceToPark));
//        robotFaceAngle = angleStart;
//
//        //ARM 11_08
//
//        startTimeArm = armElapsedTime.seconds();
//        oldTimeArm = startTimeArm;
//
//
//        ArmShoulderPositions originalArmTargetPosition = targetShoulderPosition;
//        ArmShoulderPositions intermediateArmTargetPosition = ArmShoulderPositions.HOME;
//        ArmShoulderPositions currentArmTargetPosition;
//        ArmShoulderPositions previousArmPosition = initialShoulderPosition;
//
//        RobotLog.d("originalArmTargetPosition %d, intermediateArmTargetPosition %d, targetShoulderPosition %d, previousArmPosition %d",
//                originalArmTargetPosition.getArmTarget(), intermediateArmTargetPosition.getArmTarget(),targetShoulderPosition.getArmTarget(),previousArmPosition.getArmTarget());
//
//
//        boolean finalArmTargetReached = false;
//        double armDelayTimer = 0.0;
//        if(armDelay >0)
//            armDelayTimer = armElapsedTime.seconds();
//
//        if(initialShoulderPosition != targetShoulderPosition) {
//            currentArmTargetPosition = intermediateArmTargetPosition;
//        }else{
//            currentArmTargetPosition = targetShoulderPosition;
//        }
//
//        armHoldStartTime = 0.0;
//
//        leftGrab.setPosition(targetFingerPosition.getLeftFingerPosition());
//        rightGrab.setPosition(targetFingerPosition.getRightFingerPosition());
////11_15
////        OdometryGlobalCoordinatePositionNERD globalPositionUpdate = new OdometryGlobalCoordinatePositionNERD(leftEncoder, rightEncoder, backEncoder, imu, COUNTS_PER_INCH, 75);
////        Thread positionThread = new Thread(globalPositionUpdate);
////        positionThread.start();
//
//        while (this.opmode.opModeIsActive() && !this.opmode.isStopRequested() &&
//                !distanceTargetReached(distanceToEndPoint, parkRadius) &&
//                !finalArmTargetReached && !isArmHoldTimeReached(armHoldPositionTime) ) {
//
//            //ARM_11_08
//
//            currentTimeArm = armElapsedTime.seconds();
//            loopTimeArm = currentTimeArm - oldTimeArm;
//            oldTimeArm = currentTimeArm;
//            deltaTimeArm = currentTimeArm - startTimeArm;
//
////            double[] robotPositionXYV = findDisplacementOptical();
////
//            robotXMultiThread = globalPositionUpdate.returnXCoordinate();
//            robotYMultiThread = globalPositionUpdate.returnYCoordinate();
//
//            CurvePoint followMe = getFollowPointPath(allPoints, new PointPP(robotXMultiThread, robotYMultiThread),
//                    allPoints.get(0).followDistance);
//
//            CurvePoint endPoint = getEndPoint(allPoints, new PointPP(robotXMultiThread, robotYMultiThread),
//                    allPoints.get(0).followDistance);
//
//            CurvePoint startPath = getStartPath(allPoints, new PointPP(robotXMultiThread, robotYMultiThread),
//                    allPoints.get(0).followDistance);
//
//            distanceToEndPoint = Math.hypot(endPoint.x - robotXMultiThread, endPoint.y - robotYMultiThread);
//
//            if(distanceToEndPoint < distanceToPark){
//                goToPositionEndPP(endPoint.x, endPoint.y, 1.0, parkAngleTarget, 0.2, distanceToPark, new PointPP(robotXMultiThread, robotYMultiThread));
//            }
//            else {
//                goToPositionPP(followMe.x, followMe.y, followMe.moveSpeed, zPowerFF, followMe.turnSpeed, parkAngleTarget, distanceToPark, startPath.x, startPath.y, endPoint.x, endPoint.y, new PointPP(robotXMultiThread, robotYMultiThread));
//            }
//
//            //Skip the ARM portion if a delay is requested.
//            if(armDelay > 0){
//                if((armElapsedTime.seconds() - armDelayTimer) < armDelay) continue;
//            }
//
//            //ARM Start
//            double armPidOutput = 0.0;
//            double armMotorsign = 1.0;
//            double armMotorPower = 0.0;
//
//            RobotLog.d("NERD FRONT ONE #### FollowCurveArm - currentArmTargetPosition %d", frontEncoder.getCurrentPosition());
//
//
//            armPidOutput = armPID(currentArmTargetPosition.getArmTarget(), frontEncoder.getCurrentPosition() * -1);
//            armMotorsign = Math.signum(armPidOutput);
//            if((currentArmTargetPosition == ArmShoulderPositions.HOME && previousArmPosition == ArmShoulderPositions.INTAKE) ||
//                    (currentArmTargetPosition == ArmShoulderPositions.INTAKE && previousArmPosition == ArmShoulderPositions.HOME)) {
//                RobotLog.d(" NERD INSIDE setting HOME_MAX_POWER ");
//                if (Math.abs(armPidOutput) > HOME_MAX_POWER) {
//                    armMotorPower = armMotorsign * HOME_MAX_POWER;
//                } else {
//                    armMotorPower = armPidOutput;
//                }
//            }
//            else {
//                if (Math.abs(armPidOutput) > currentArmTargetPosition.getMaxPower()) {
//                    armMotorPower = armMotorsign * currentArmTargetPosition.getMaxPower();
//                } else {
//                    armMotorPower = armPidOutput;
//                }
//            }
//            RobotLog.d("NERDBLUEAUTON Motor powers %f, LooptimeARM %f", armMotorPower, loopTimeArm);
//            frontEncoder.setPower(armMotorPower);
//            rightEncoder.setPower(-armMotorPower);
//            leftArmServo.setPosition(currentArmTargetPosition.getLeftWristServoPosition());
//            rightArmServo.setPosition(currentArmTargetPosition.getRightWristServoPosition());
//
//            RobotLog.d("originalArmTargetPosition %d, intermediateArmTargetPosition %d, targetShoulderPosition %d, previousArmPosition %d",
//                    originalArmTargetPosition.getArmTarget(), intermediateArmTargetPosition.getArmTarget(),targetShoulderPosition.getArmTarget(),previousArmPosition.getArmTarget());
//
//            if(initialShoulderPosition != targetShoulderPosition){
//                if(currentArmTargetPosition == intermediateArmTargetPosition){
//
//                    if(isArmTargetReached(currentArmTargetPosition, frontEncoder.getCurrentPosition())){
//                        RobotLog.d("NERD_11_08 #### FollowCurveArm - ArmTarget Reached SWAPPED, originalArmTargetPosition %d, currentArmTargetPosition %d, frontEncoder.getCurrentPosition %d",
//                                originalArmTargetPosition.getArmTarget(),currentArmTargetPosition.getArmTarget(),frontEncoder.getCurrentPosition() );
//                        previousArmPosition = currentArmTargetPosition;
//                        currentArmTargetPosition = originalArmTargetPosition;
//                    }
//
//                    RobotLog.d("NERD_11_08 #### FollowCurveArm - Did NOT SWAP originalArmTargetPosition %d, currentArmTargetPosition %d, frontEncoder.getCurrentPosition %d",
//                            originalArmTargetPosition.getArmTarget(),currentArmTargetPosition.getArmTarget(),frontEncoder.getCurrentPosition() );
//
//                }
//            }
//            if((distanceTargetReached(distanceToEndPoint,parkRadius) && isArmTargetReached(originalArmTargetPosition,frontEncoder.getCurrentPosition()))){
//                finalArmTargetReached = true;
//                if(armHoldPositionTime > 0)
//                    armHoldStartTime = armElapsedTime.seconds();
//                RobotLog.d("NERD_11_08 #### FollowCurveArm - Arm Hold Timer Started originalArmTargetPosition %d, currentArmTargetPosition %d, frontEncoder.getCurrentPosition %d",
//                        originalArmTargetPosition.getArmTarget(),currentArmTargetPosition.getArmTarget(),frontEncoder.getCurrentPosition() );
//                if(originalArmTargetPosition == ArmShoulderPositions.INTAKE){
//                    leftGrab.setPosition(FingerPositions.INTAKE_READY.getLeftFingerPosition());
//                    rightGrab.setPosition(FingerPositions.INTAKE_READY.getRightFingerPosition());
//                }else {
//                    leftGrab.setPosition(endFingerPosition.getLeftFingerPosition());
//                    rightGrab.setPosition(endFingerPosition.getRightFingerPosition());
//                }
//
//
//            }
//
//            //ARM End
//
//            //ARM 11_08
//
//            if(motor.equals("intake")) {
//                //   if(finalArmTargetReached == true && originalArmTargetPosition == ArmShoulderPositions.INTAKE)
//                runMotor("intake", power);
//            }
//            else if (motor.equals("duckyDisc"))
//                runMotor("duckyDisc", power);
//
//        }
//        frontLeftMotor.setPower(0);
//        frontRightMotor.setPower(0);
//        rearLeftMotor.setPower(0);
//        rearRightMotor.setPower(0);
//        runMotor("intake",0);
//        runMotor("duckyDisc", 0);
////11_15
////        globalPositionUpdate.stop();
//
//    }



    public void followCurveArm_V2(ArrayList<CurvePoint> allPoints, double zPowerFF, double distanceToPark, double parkAngleTarget,
                                  double parkRadius, ArmShoulderPositions initialShoulderPosition, ArmShoulderPositions targetShoulderPosition,
                                  FingerPositions targetFingerPosition, FingerPositions endFingerPosition,
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
        pL = 0;
        distanceFromPointPath = 0;

        //ARM 11_08

        startTimeArm = armElapsedTime.seconds();
        oldTimeArm = startTimeArm;

        ElapsedTime armElapsedTimeForDelay=new ElapsedTime();


        ArmShoulderPositions originalArmTargetPosition = targetShoulderPosition;
        ArmShoulderPositions intermediateArmTargetPosition = ArmShoulderPositions.HOME;
        ArmShoulderPositions currentArmTargetPosition;
        ArmShoulderPositions previousArmPosition = initialShoulderPosition;

        RobotLog.d("originalArmTargetPosition %d, intermediateArmTargetPosition %d, targetShoulderPosition %d, previousArmPosition %d",
                originalArmTargetPosition.getArmTarget(), intermediateArmTargetPosition.getArmTarget(),targetShoulderPosition.getArmTarget(),previousArmPosition.getArmTarget());


        boolean finalArmTargetReached = false;
        double armDelayStartTime = 0.0;
        if(armDelay >0) {
            armDelayStartTime = armElapsedTimeForDelay.seconds();
        }
        boolean armDelayCompleted = false;

        //If we do not want to move the arm, no need to go home. Else, we always go to HOME position first.
        if(initialShoulderPosition == targetShoulderPosition) {
            currentArmTargetPosition = initialShoulderPosition;
        }else{
            currentArmTargetPosition = intermediateArmTargetPosition;
        }


        armHoldStartTime = 0.0;

        leftGrab.setPosition(targetFingerPosition.getLeftFingerPosition());
        rightGrab.setPosition(targetFingerPosition.getRightFingerPosition());

        ElapsedTime intakeDelayTimer = new ElapsedTime();



        while (this.opmode.opModeIsActive() && !this.opmode.isStopRequested() &&
                !distanceTargetReached(distanceToEndPoint, parkRadius) &&
                !finalArmTargetReached && !isArmHoldTimeReached(armHoldPositionTime) ) {

            //ARM_11_08

            currentTimeArm = armElapsedTime.seconds();
            loopTimeArm = currentTimeArm - oldTimeArm;
            oldTimeArm = currentTimeArm;
            deltaTimeArm = currentTimeArm - startTimeArm;

//            double[] robotPositionXYV = findDisplacementOptical();
//
            robotXMultiThread = globalPositionUpdate.returnXCoordinate();
            robotYMultiThread = globalPositionUpdate.returnYCoordinate();

            CurvePoint followMe = getFollowPointPath(allPoints, new PointPP(robotXMultiThread, robotYMultiThread),
                    allPoints.get(0).followDistance, pL);

            CurvePoint endPoint = getEndPoint(allPoints, new PointPP(robotXMultiThread, robotYMultiThread),
                    allPoints.get(0).followDistance);

            CurvePoint startPath = getStartPath(allPoints, new PointPP(robotXMultiThread, robotYMultiThread),
                    allPoints.get(0).followDistance);

            distanceToEndPoint = Math.hypot(endPoint.x - robotXMultiThread, endPoint.y - robotYMultiThread);

            if (debugFlag) {
                RobotLog.d("distanceToEndPoint- runTime %f, deltaTime %f, distanceToEndPoint %f, robotXMultiThread %f, robotYMultiThread %f, endPointx %f, pL %d, endPointy %f, distanceToPark %f",
                        currentTime, deltaTime, distanceToEndPoint, robotXMultiThread, robotYMultiThread, endPoint.x, pL, endPoint.y, distanceToPark);
            }

            CurvePoint startSegment1 = allPoints.get(pL);
            CurvePoint endSegment1 = allPoints.get(pL + 1);

            ArrayList<PointPP> perpendicularIntersection1 = MathFunctions2.pathDistance(new PointPP(robotXMultiThread, robotYMultiThread), startSegment1.toPoint(), endSegment1.toPoint(), followMe.toPoint());

            for(PointPP thisIntersection1 : perpendicularIntersection1){
                pathSegment1Length = Math.sqrt(((endSegment1.x - startSegment1.x) * (endSegment1.x - startSegment1.x)) + (endSegment1.y - startSegment1.y) * (endSegment1.y - startSegment1.y));
                distanceToPoint = Math.hypot((endSegment1.x - thisIntersection1.x), (endSegment1.y - thisIntersection1.y));
//                distanceFromPoint = Math.hypot((startSegment1.x - thisIntersection1.x), (startSegment1.y - thisIntersection1.y));
//
//                thisIntersection1XDisp = thisIntersection1.x - thisIntersection1XOld;
//                thisIntersection1YDisp = thisIntersection1.y - thisIntersection1YOld;
//                thisIntersection1XOld = thisIntersection1.x;
//                thisIntersection1YOld = thisIntersection1.y;
//                pathSegment1Mag = Math.hypot(thisIntersection1XDisp, thisIntersection1YDisp);
//                distanceFromPointPath += pathSegment1Mag;

//                if (distanceToPoint < (pathSegment1Length / 5) || distanceFromPoint > pathSegment1Length || distanceFromPointPath > (pathSegment1Length * 0.9))
                if (distanceToPoint < (pathSegment1Length / 3) || distanceFromPoint > pathSegment1Length){
                    distanceReached = true;
                    pL = pL + 1;
                    distanceFromPointPath = 0;
                }else{
                    distanceReached = false;
                }
                distanceToPointOld = distanceToPoint;
            }

            if (debugFlag) {
                RobotLog.d("followCurveArm_V2 - runTime %f, deltaTime %f, pathSegment1Length %f, distanceToPoint %f, distanceFromPoint %f, distanceFromPointPath %f, pL %d, distanceToEndPark %f, distanceToPark %f",
                        currentTime, deltaTime, pathSegment1Length, distanceToPoint, distanceFromPoint, distanceFromPointPath, pL, distanceToEndPoint, distanceToPark);
            }

            if(distanceToEndPoint < distanceToPark){
                goToPositionEndPP(endPoint.x, endPoint.y, 1.0, parkAngleTarget, 0.2, distanceToPark, new PointPP(robotXMultiThread, robotYMultiThread));
            }
            else {
                goToPositionPP(followMe.x, followMe.y, followMe.moveSpeed, zPowerFF, followMe.turnSpeed, parkAngleTarget, distanceToPark, startSegment1.x, startSegment1.y, endSegment1.x, endSegment1.y,
                        new PointPP(robotXMultiThread, robotYMultiThread), startSegment1.slowDownTurnAmount, endSegment1.slowDownTurnAmount, pathSegment1Length, distanceToPoint);
            }

            //If there is a delay requested, we stay at the initial position until the requested time.
            //After that we go to HOME position.

            if(armDelay > 0){
                if(armElapsedTimeForDelay.seconds() - armDelayStartTime <= armDelay){
                    currentArmTargetPosition = initialShoulderPosition;
                }else{
                    if(!armDelayCompleted) {
                        currentArmTargetPosition = intermediateArmTargetPosition;
                        armDelayCompleted = true;
                    }
                }
            }

            //ARM Start
            double armPidOutput = 0.0;
            double armMotorsign = 1.0;
            double armMotorPower = 0.0;

            RobotLog.d("NERD FRONT ONE #### FollowCurveArm - currentArmTargetPosition %d", frontEncoder.getCurrentPosition());

            //We calculate PID for currentARmTargetPosition.
            //From HOME to INTAKE and INTAKE to HOME, we use less power.
            armPidOutput = armPID(currentArmTargetPosition.getArmTarget(), frontEncoder.getCurrentPosition() * -1);
            armMotorsign = Math.signum(armPidOutput);
            if((currentArmTargetPosition == ArmShoulderPositions.HOME && previousArmPosition == ArmShoulderPositions.INTAKE) ||
                    (currentArmTargetPosition == ArmShoulderPositions.INTAKE && previousArmPosition == ArmShoulderPositions.HOME)) {
                RobotLog.d(" NERD INSIDE setting HOME_MAX_POWER ");
                if (Math.abs(armPidOutput) > HOME_MAX_POWER) {
                    armMotorPower = armMotorsign * HOME_MAX_POWER;
                } else {
                    armMotorPower = armPidOutput;
                }
            }
            else {
                if (Math.abs(armPidOutput) > currentArmTargetPosition.getMaxPower()) {
                    armMotorPower = armMotorsign * currentArmTargetPosition.getMaxPower();
                } else {
                    armMotorPower = armPidOutput;
                }
            }
            frontEncoder.setPower(armMotorPower);
            rightEncoder.setPower(-armMotorPower);
//            leftArmServo.setPosition(currentArmTargetPosition.getLeftWristServoPosition());
//            rightArmServo.setPosition(currentArmTargetPosition.getRightWristServoPosition());


            // If we reach intermediate arm position (HOME), we swap to final arm position.
            if(initialShoulderPosition != targetShoulderPosition){
                if(currentArmTargetPosition == intermediateArmTargetPosition){
                    if(isArmTargetReached(currentArmTargetPosition, frontEncoder.getCurrentPosition())){
                        RobotLog.d("NERD_11_08 #### FollowCurveArm - ArmTarget Reached SWAPPED, originalArmTargetPosition %d, currentArmTargetPosition %d, frontEncoder.getCurrentPosition %d",
                                originalArmTargetPosition.getArmTarget(),currentArmTargetPosition.getArmTarget(),frontEncoder.getCurrentPosition() );
                        previousArmPosition = currentArmTargetPosition;
                        currentArmTargetPosition = originalArmTargetPosition;
                    }

                    RobotLog.d("NERD_11_08 #### FollowCurveArm - Did NOT SWAP originalArmTargetPosition %d, currentArmTargetPosition %d, frontEncoder.getCurrentPosition %d",
                            originalArmTargetPosition.getArmTarget(),currentArmTargetPosition.getArmTarget(),frontEncoder.getCurrentPosition() );

                }
            }
            if((distanceTargetReached(distanceToEndPoint,parkRadius) && isArmTargetReached(originalArmTargetPosition,frontEncoder.getCurrentPosition()))){
                finalArmTargetReached = true;
                if(armHoldPositionTime > 0)
                    armHoldStartTime = armElapsedTime.seconds();
                RobotLog.d("NERD_11_08 #### FollowCurveArm - Arm Hold Timer Started originalArmTargetPosition %d, currentArmTargetPosition %d, frontEncoder.getCurrentPosition %d",
                        originalArmTargetPosition.getArmTarget(),currentArmTargetPosition.getArmTarget(),frontEncoder.getCurrentPosition() );
                if(originalArmTargetPosition == ArmShoulderPositions.INTAKE){
                    leftGrab.setPosition(FingerPositions.INTAKE_READY.getLeftFingerPosition());
                    rightGrab.setPosition(FingerPositions.INTAKE_READY.getRightFingerPosition());
                }else {
                    leftGrab.setPosition(endFingerPosition.getLeftFingerPosition());
                    rightGrab.setPosition(endFingerPosition.getRightFingerPosition());
                }


            }

            //ARM End


            if(motor.equals("intake")) {
                //   if(finalArmTargetReached == true && originalArmTargetPosition == ArmShoulderPositions.INTAKE)
//                intakeDelayTimer.reset();
                if(intakeDelayTimer.seconds()> 1)
                    runMotor("intake", power);
            }
            else if (motor.equals("duckyDisc"))
                runMotor("duckyDisc", power);

        }
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
        runMotor("intake",0);
        runMotor("duckyDisc", 0);


    }


    public void setFingerPositions(FingerPositions targetFingerPosition)
    {
        this.leftGrab.setPosition(targetFingerPosition.getLeftFingerPosition());
        this.rightGrab.setPosition(targetFingerPosition.getRightFingerPosition());
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
        if(Math.abs((targetPosition.getArmTarget() - Math.abs(currentPosition)) )<= 20){

            targetReached = true;
        }

        return targetReached;

    }


    private CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, PointPP robotLocation, double followRadius, int pLf){
        CurvePoint followMe = new CurvePoint(pathPoints.get(pLf));

        for(int i = 0; i < pathPoints.size() - 1; i++){
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endline = pathPoints.get(i + 1);

//            double[] robotPositionXYV = findDisplacementOptical();

            ArrayList<PointPP> intersections = MathFunctions2.lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endline.toPoint());

            double closestAngle = 100000000;

//            double closestDistance = 1000000;


            for(PointPP thisIntersection : intersections){
                double angle = Math.atan2(thisIntersection.y - robotLocation.y, thisIntersection.x - robotLocation.x);
                robotVectorAngle = globalPositionUpdate.returnVectorByOdo();
                double deltaAngle = Math.abs(MathFunctions.AngleWrapDeg((angle * 180 / Math.PI) - robotVectorAngle));
                //Need to check which angle is the same as worldAngle_rad...currently using getAngle() converted to radians
//                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle - (robotPositionXYV[6] * Math.PI / 180)));

                if(deltaAngle < closestAngle + 10){
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
//                double distance = Math.hypot(thisIntersection.x - robotLocation.x, thisIntersection.y - robotLocation.y);
//
//                if(distance < closestDistance){
//                    closestDistance = distance;
//                    followMe.setPoint(thisIntersection);
                }
            }
            if (debugFlag) {
                RobotLog.d("CurvePoint - runTime %f, deltaTime %f, closestAngle %f, robotVectorAngle %f, followMe.x %f, followMe.y %f",
                        currentTime, deltaTime, closestAngle, robotVectorAngle, followMe.x, followMe.y);
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
                               double robotPositionXStart, double robotPositionYStart, double endPointX, double endPointY, PointPP robotLocationMT,
                               double segmentStartAngle, double segmentEndAngle, double segmentLength, double distToPoint){

        currentTime = elapsedTime.seconds();
        loopTime = currentTime - oldTime;
        oldTime = currentTime;

        distanceToTarget = Math.hypot(x - robotLocationMT.x, y - robotLocationMT.y);

        double absoluteAngleToTarget = Math.atan2(y - robotLocationMT.y, x - robotLocationMT.x) * 180 / Math.PI;

        robotTargetAngle = absoluteAngleToTarget;

        motorAngleToTarget = MathFunctions.AngleWrapDeg((robotTargetAngle - 45) - getAngle());

        xPower = Math.cos(motorAngleToTarget * 3.14 / 180) * movementSpeed;
        yPower = Math.sin(motorAngleToTarget * 3.14 / 180) * movementSpeed;

//        double relativeTurnAngle = MathFunctions.AngleWrapDeg(robotTargetAngle - (getAngle() + 90));
//        double distanceFromStart = Math.hypot(robotLocationMT.x - robotPositionXStart, robotLocationMT.y - robotPositionYStart);
//        double distanceAtStart = Math.hypot(endPointX - robotPositionXStart, endPointY - robotPositionYStart);
        angleIncrement = 3 * (segmentEndAngle - segmentStartAngle);

        if (distToPoint > segmentLength * 0.3){
            if (segmentStartAngle < segmentEndAngle){
                robotFaceAngle = Range.clip(angleIncrement * (1 - (distToPoint / segmentLength)) + segmentStartAngle, segmentStartAngle, segmentEndAngle);
            }else if (segmentStartAngle > segmentEndAngle){
                robotFaceAngle = Range.clip(angleIncrement * (1 - (distToPoint / segmentLength)) + segmentStartAngle, segmentEndAngle, segmentStartAngle);
            }
        }else if (distToPoint < segmentLength * 0.3){
            robotFaceAngle = segmentEndAngle;
        }

        zPIDAngle = 90 + getAngle();
//        robotTurnSpeed = NerdPID_PurePursuit.zPowerDrive(robotFaceAngle, zPIDAngle, loopTime);
        robotTurnSpeed = NerdPID_PurePursuit.zPowerDrive(robotFaceAngle, zPIDAngle, loopTime);
        double robotTurnSpeedFF = Range.clip((zPowerStart + zPowerIncrease), -0.5, 0);
        zPowerStart = robotTurnSpeedFF;


        zPower = Range.clip((robotTurnSpeed + zPowerStart), -0.5, 0.5);

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
            RobotLog.d("goToPositionPP - runTime %f, deltaTime %f, robotLocationX %f, robotLocationY %f, segmentStartAngle %f, segmentEndAngle %f, segmentLength %f, distToPoint %f, robotFaceAngle %f, robotAngle %f, pL %d, pLf %d, parkDistance %f",
                    currentTime, deltaTime, robotLocationMT.x, robotLocationMT.y, segmentStartAngle, segmentEndAngle, segmentLength, distToPoint, robotFaceAngle, zPIDAngle, pL, pLf, parkDistance);
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

//    public void AutonBlockIntake(){
//        isBlockIn = false;
//        IntakeTimer.reset();
//        IntakeTimer2.reset();
//        while(this.opmode.opModeIsActive() && isBlockIn == false && !this.opmode.isStopRequested()) {
//            IntakeTimer.reset();
//            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//            while(!(colorSensor.alpha() > 200) && IntakeTimer.seconds() <= 2) {
//                rearRightMotor.setPower(0.35);
//                frontLeftMotor.setPower(-0.35);
//                rearLeftMotor.setPower(-0.35);
//                frontRightMotor.setPower(0.35);
//                backEncoder.setPower(-1);
//                this.opmode.telemetry.addData("timer 1", IntakeTimer.seconds());
//                this.opmode.telemetry.update();
//
//            }
//            if(colorSensor.alpha() > 200) {
//                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);}
//            IntakeTimer2.reset();
//            while (IntakeTimer2.seconds() < 0.75) {
//                rearRightMotor.setPower(-0.35);
//                frontLeftMotor.setPower(0.35);
//                rearLeftMotor.setPower(0.35);
//                frontRightMotor.setPower(-0.35);
//                backEncoder.setPower(0);
//                this.opmode.telemetry.addData("timer 2", IntakeTimer2.seconds());
//                this.opmode.telemetry.update();
//
//            }
//            rearRightMotor.setPower(0);
//            frontLeftMotor.setPower(0);
//            rearLeftMotor.setPower(0);
//            frontRightMotor.setPower(0);
//            if(colorSensor.alpha() > 200) {
//                backEncoder.setPower(0.5);
//            }
//            if(colorSensor.alpha() > 200)  { isBlockIn = true; }
//
//        }
//
//
//
//    }
//
//    public boolean checkIfBlockIsIn(){
//        boolean blockIsIn = false;
//        if(colorSensor.alpha() > 200) blockIsIn = true;
//        return  blockIsIn;
//    }


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
        runDriveMotors(0);

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


    public void moveArmsOnly(ArmShoulderPositions armTargetPosition, FingerPositions fingerTargetPosition){

        startTimeArmOnly = elapsedTime.seconds();
        oldTimeArmOnly = startTimeArmOnly;

        while (this.opmode.opModeIsActive() && !isArmTargetReached(armTargetPosition,frontEncoder.getCurrentPosition())){
            currentTimeArmOnly = elapsedTime.seconds();
            loopTimeArmOnly = currentTimeArmOnly - oldTimeArmOnly;
            oldTimeArmOnly = currentTimeArmOnly;
            deltaTimeArmOnly = currentTimeArmOnly - startTimeArmOnly;


            //ARM Start
            double armPidOutput = 0.0;
            double armMotorsign = 1.0;
            double armMotorPower = 0.0;

            armPidOutput = armPID(armTargetPosition.getArmTarget(), frontEncoder.getCurrentPosition() * -1); //11_08 check
            armMotorsign = Math.signum(armPidOutput);
            if(armTargetPosition.equals(ArmShoulderPositions.HOME) || armTargetPosition.equals(ArmShoulderPositions.INTAKE)) {
                if (Math.abs(armPidOutput) > HOME_MAX_POWER_ARMS_ONLY) {
                    armMotorPower = armMotorsign * HOME_MAX_POWER_ARMS_ONLY;
                } else {
                    armMotorPower = armPidOutput;
                }
            }
            else {
                if (Math.abs(armPidOutput) > armTargetPosition.getMaxPower()) {
                    armMotorPower = armMotorsign * armTargetPosition.getMaxPower();
                } else {
                    armMotorPower = armPidOutput;
                }
            }

            leftEncoder.setPower(armMotorPower);
            rightEncoder.setPower(-armMotorPower); //11_08 check
            leftGrab.setPosition(fingerTargetPosition.getLeftFingerPosition());
            rightGrab.setPosition(fingerTargetPosition.getRightFingerPosition());
//            leftArmServo.setPosition(armTargetPosition.getLeftWristServoPosition());
//            rightArmServo.setPosition(armTargetPosition.getRightWristServoPosition());

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
            turnPIDmotorPower = Range.clip(turnPIDpropError * turnPIDKp + turnPIDintError * turnPIDKi + turnPIDderError * turnPIDKd, -0.9, 0.9);
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
//
//    public void resetColorSensor(){
//        colorSensor.resetDeviceConfigurationForOpMode();
//    }


}


