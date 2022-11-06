package teamcode.Auton;

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

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

//import opencv.core.PointPP;
import teamcode.RobotUtilities.core.PointPP;
import teamcode.RobotUtilities.ArmShoulderPositions;
import teamcode.RobotUtilities.FingerPositions;

import java.util.ArrayList;

public class PurePursuitRobotMovement6_Turn {

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

    private Servo leftArmServo;
    private Servo rightArmServo;
    //Finger Servos
    private Servo leftGrab;
    private Servo rightGrab;

    public static final double indexerHomePos = 1.0;
    public static final double indexerPushedPos = 0.45;

//    private DcMotor wobbleMotor;
//    Servo wobbleServo;

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

    private double xPosition = 0;
    private double yPosition = 0;
    private double displacementX = 0;
    private double displacementY = 0;

    double xPositionOpt = 0;
    double yPositionOpt = 0;


    private double xPositionOpticalInit = 0;
    private double yPositionOpticalInit = 0;
    private double displacementXOptical = 0;
    private double displacementYOptical = 0;

    double frontDispNoRotTotOpt = 0;
    double leftDispNoRotTotOpt = 0;
    double rightDispNoRotTotOpt = 0;
    double rearDispNoRotTotOpt = 0;

    double frontPositionOptical = 0;
    double rightPositionOptical = 0;
    double leftPositionOptical = 0;
    double backPositionOptical = 0;
    double [] robotPositionXYOptical;
//    double robotPositionXStart = 0;
//    double robotPositionYStart = 0;

    double omniDriveFactorOpt = 0;

    double frontDisplacementOld = 0;
    double leftDisplacementOld = 0;
    double rightDisplacementOld = 0;
    double rearDisplacementOld = 0;

    double robotRot = 0;
    double robotRotNew = 0;
    double robotRotOld = 0;
    double robotRotDisplacement = 0;
    double robotVectorByOdo = 0;
    double rfDisplacement = 0;
    double rfDisplacementNew = 0;
    double rfDisplacementOld = 0;
    double lfDisplacement = 0;
    double lfDisplacementNew = 0;
    double lfDisplacementOld = 0;
    double rrDisplacement = 0;
    double rrDisplacementNew = 0;
    double rrDisplacementOld = 0;
    double lrDisplacement = 0;
    double lrDisplacementNew = 0;
    double lrDisplacementOld = 0;
    double rfDispNoRot = 0;
    double lfDispNoRot = 0;
    double rrDispNoRot = 0;
    double lrDispNoRot = 0;
    double rfDispNoRotTot = 0;
    double lfDispNoRotTot = 0;
    double rrDispNoRotTot = 0;
    double lrDispNoRotTot = 0;
    double robotVectorMag = 0;
    double omniDriveAngle = 0;
    double omniDriveFactor = 0;


    double robotRotNewOpt = 0;
    double robotRotOldOpt = 0;
    double robotRotOpt = 0;
    //double robotRotDisplacementOptF = 0;
    double robotXdisplacementOpt = 0;
    double robotYdisplacementOpt = 0;
    double robotVectorByOdoOpt = 0;
    double robotVectorMagOpt = 0;
    double robotFieldAngleOpt = 0;
    double robotXdisplacementOptTot = 0;
    double robotYdisplacementOptTot = 0;


    double robotFieldPositionX = 0;
    double robotFieldPositionY = 0;
    double robotFieldAngle = 0;
    double [] robotPositionXY;
    double robotXdisplacement = 0;
    double robotYdisplacement = 0;

    double distanceToTarget = 10;
    double prevDistanceToTarget = 20;
    double distanceToTargetAngle = 0;
    double distanceToEndPoint = 10;

//    static final double DISTANCE_THRESHOLD = 2;

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
    public static double armKp = 0.005;//0.01
    public static double armKi = 0.00;
    public static double armKd = 0.00005;
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
    public static double turnPIDKp = 0.0075;
    public static double turnPIDKi = 0.0006;
    public static double turnPIDKd = 0.001;

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



    double shooterVeloc = 1000;


    /**
     * Constructor to create NerdBOT object
     * <p>
     * Creates a new NerdBOT object and assigns the hardwareMap provided by caller
     *
     * @param opmode Hardware Map provided by the calling OpMode.
     *               NerdBOT takes an opmode object so that it can get the hardwareMap.     *
     */

    public PurePursuitRobotMovement6_Turn(LinearOpMode opmode) {
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
        this.rightEncoder.setDirection(DcMotor.Direction.REVERSE);
        this.leftEncoder.setDirection(DcMotor.Direction.REVERSE);
        this.backEncoder.setDirection(DcMotor.Direction.REVERSE);

        xPosition = 0;
        yPosition = 0;

        displacementX = 0;
        displacementY = 0;

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

        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        leftGrab = hardwareMap.get(Servo.class, "leftGrab");
        rightGrab = hardwareMap.get(Servo.class, "rightGrab");

        //Positions to get in the intake. This is initial position we will be at the beginning.

        leftArmServo.setPosition(0.3);
        rightArmServo.setPosition(0.7);
        leftGrab.setPosition(0.53);
        rightGrab.setPosition(0.55);


        xPositionOpticalInit = 0;
        yPositionOpticalInit = 0;

        xPositionOpt = 0;
        yPositionOpt = 0;

        displacementXOptical = 0;
        displacementYOptical = 0;

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

    public void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed, double parkDistance, double parkRadius){
//        startTime = runtime;

        distanceToTarget = 10;
        distanceToTargetAngle = getAngle() + 90;

//        double [] robotPositionXYOpticalInit = findDisplacementOptical();
//        xPositionOpticalInit = robotPositionXYOpticalInit[4];
//        yPositionOpticalInit = robotPositionXYOpticalInit[5];

        while (this.opmode.opModeIsActive() && !this.opmode.isStopRequested() && !distanceTargetReached(distanceToTarget, parkRadius)) {

            //First calculate motor speeds for linear (x, y) motion

            currentTime = elapsedTime.seconds();
            loopTime = currentTime - oldTime;
            oldTime = currentTime;
            deltaTime = currentTime - startTime;

            robotPositionXY = findDisplacement(xPosition, yPosition, robotVectorByOdo);

            robotPositionXYOptical = findDisplacementOptical();

            //distanceToTarget = Math.hypot(x - robotPositionXY[0], y - robotPositionXY[1]);
            distanceToTarget = Math.hypot(x - robotPositionXYOptical[4], y - robotPositionXYOptical[5]);

            double absoluteAngleToTarget = Math.atan2(y - robotPositionXYOptical[5], x - robotPositionXYOptical[4]) * 180 / Math.PI;

            if (distanceToTarget < parkDistance && distanceToTarget > (parkDistance / 2)){
                robotTargetSpeed = movementSpeed * (distanceToTarget - (parkDistance / 2)) / 30;
                prevDistanceToTarget = parkDistance / 2;
            }else if (distanceToTarget < (parkDistance / 2)){
                robotTargetSpeedPID = NerdPID_PurePursuit.goToPositionParkPID(distanceToTarget, prevDistanceToTarget, loopTime);
                prevDistanceToTarget = distanceToTarget;
                robotTargetSpeed = robotTargetSpeedPID;
            }else{
                robotTargetSpeed = movementSpeed;
            }


            robotTargetAngle = absoluteAngleToTarget;

//            robotAngleToTarget = MathFunctions.AngleWrapDeg(robotTargetAngle - getAngle());
            motorAngleToTarget = MathFunctions.AngleWrapDeg((robotTargetAngle - 45) - getAngle());

            xPower = Math.cos(motorAngleToTarget * 3.14 / 180) * robotTargetSpeed;
            yPower = Math.sin(motorAngleToTarget * 3.14 / 180) * robotTargetSpeed;



//            double relativeTurnAngle = MathFunctions.AngleWrapDeg(robotAngleToTarget - 180 + preferredAngle);
            double relativeTurnAngle = MathFunctions.AngleWrapDeg(robotTargetAngle - (getAngle() + 90));

            if (Math.abs(preferredAngle - (getAngle() + 90)) > 35){
                useZPID = false;
            }
            else if (Math.abs(preferredAngle - (getAngle() + 90)) < 10){
                useZPID = true;
            }

            if (useZPID && distanceToTarget > parkDistance){
                zPIDAngle = 90 + getAngle();
                robotTurnSpeed = NerdPID_PurePursuit.zPowerDrive(preferredAngle, zPIDAngle, loopTime);
            } else if (useZPID && distanceToTarget < parkDistance){
                zPIDAngle = 90 + getAngle();
                robotTurnSpeed = NerdPID_PurePursuit.zPowerPark(preferredAngle, zPIDAngle, loopTime);
            } else if (!useZPID){
                robotTurnSpeed = Range.clip((preferredAngle - (getAngle() + 90)) / 30, -1, 1) * turnSpeed;
            }

            zPower = Range.clip(Math.abs(relativeTurnAngle / 30) * robotTurnSpeed, -0.3, 0.3);


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




//            if (debugFlag) {
//                RobotLog.d("goToPosition - timeSinceStart %f, robotTargetAngle %f, xPower %f, yPower %f , zPower %f, frontLeftMotorPower %f, rearRightMotorPower %f , frontRightMotorPower %f, rearLeftMotorPower %f, frontLeftMotorTicks %f, rearRightMotorTicks %f , frontRightMotorTicks %f , rearLeftMotorTicks %f, xPosition %f, yPosition %f, robotRot %f, robotRotDisplacement %f, robotAngleToTarget %f, robotVectorByOdoF %f, robotVectorByOdoR %f, frontVectorMag %f, rearVectorMag %f",
//                        deltaTime, robotTargetAngle, xPower, yPower, zPower, frontLeftMotorPower, rearRightMotorPower, frontRightMotorPower, rearLeftMotorPower, lfDisplacement, rrDisplacement, rfDisplacement, lrDisplacement, xPosition, yPosition, robotRot, robotRotDisplacement, robotAngleToTarget, robotVectorByOdoF, robotVectorByOdoR, frontVectorMag, rearVectorMag);
//            }

            // channels to record for velocity following
//            if (debugFlag) {
//                RobotLog.d("goToPosition - deltaTime %f, frontLeftMotorTarget %f, frontLeftMotorSpeed %f, frontRightMotorTarget %f, frontRightMotorSpeed %f, rearLeftMotorTarget %f, rearLeftMotorSpeed %f, rearRightMotorTarget %f, rearRightMotorSpeed %f, frontLeftMotorPower %f, frontRightMotorPower %f, rearLeftMotorPower %f, rearRightMotorPower %f, robotTargetAngle %f, robotAngleToTarget %f, zPIDAngle %f, relativeTurnAngle %f, xPosition %f, yPosition %f, frontOpticalEncoder %f, rightOpticalEncoder %f, leftOpticalEncoder %f, backOpticalEncoder %f, xPositionOpt %f, yPositionOpt %f, omniDriveFactorOpt %f",
//                        deltaTime, frontLeftMotorTarget, frontLeftMotorSpeed, frontRightMotorTarget, frontRightMotorSpeed, rearLeftMotorTarget, rearLeftMotorSpeed, rearRightMotorTarget, rearRightMotorSpeed, motorSpeedCommand [0], motorSpeedCommand [1], motorSpeedCommand [2], motorSpeedCommand [3], robotTargetAngle, robotAngleToTarget, zPIDAngle, relativeTurnAngle, robotPositionXY[0], robotPositionXY[1], robotPositionXYOptical[0], robotPositionXYOptical [1], robotPositionXYOptical [2], robotPositionXYOptical[3], robotPositionXYOptical[4], robotPositionXYOptical[5], omniDriveFactorOpt);
//            }

            // channels to record to debug optical encoder field centric driving
            if (debugFlag) {
                RobotLog.d("goToPosition - runTime %f, deltaTime %f, robotXdisplacementOpt %f, robotYdisplacementOpt %f, robotVectorByOdoOpt %f, robotVectorByOdo %f, robotVectorMagOpt %f, robotVectorMag %f, robotFieldAngleOpt %f, robotFieldAngle %f, relativeTurnAngle %f, robotAngleToTarget %f, robotTargetAngle %f, xPositionOpt %f, xPosition %f, yPositionOpt %f, yPosition %f, omniDriveFactorOpt %f, omniDriveFactor %f, distanceToTarget %f, robotTargetSpeed %f, robotTurnSpeed %f, zSpeedTargetAngle %f, zPIDAngle %f, distanceToTargetAngle %f",
                        currentTime, loopTime, robotXdisplacementOpt, robotYdisplacementOpt, robotVectorByOdoOpt, robotVectorByOdo, robotVectorMagOpt, robotVectorMag, robotFieldAngleOpt, robotFieldAngle, relativeTurnAngle, robotAngleToTarget, robotTargetAngle, xPositionOpt, xPosition, yPositionOpt, yPosition, omniDriveFactorOpt, omniDriveFactor, distanceToTarget, robotTargetSpeed, robotTurnSpeed, zSpeedTargetAngle, zPIDAngle, distanceToTargetAngle);

            }


        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);

    }

    private double [] findDisplacement(double displacementX, double displacementY, double robotVector){

        robotRotNew = getAngle();
        robotRot = robotRotNew - robotRotOld;
        robotRotOld = robotRotNew;

        //robot rotation expressed in motor ticks (robot angle, ticks per rev, robot diameter, degrees per rev, wheel diameter
        robotRotDisplacement = robotRot * 540 * (20.25 / (360 * 3.543));

        rfDisplacementNew = frontRightMotor.getCurrentPosition();
        lfDisplacementNew = frontLeftMotor.getCurrentPosition();
        lrDisplacementNew = rearLeftMotor.getCurrentPosition();
        rrDisplacementNew = rearRightMotor.getCurrentPosition();

        rfDisplacement = rfDisplacementNew - rfDisplacementOld;
        lfDisplacement = lfDisplacementNew - lfDisplacementOld;
        lrDisplacement = lrDisplacementNew - lrDisplacementOld;
        rrDisplacement = rrDisplacementNew - rrDisplacementOld;

        rfDisplacementOld = rfDisplacementNew;
        lfDisplacementOld = lfDisplacementNew;
        lrDisplacementOld = lrDisplacementNew;
        rrDisplacementOld = rrDisplacementNew;

        lfDispNoRot = lfDisplacement - robotRotDisplacement;
        rrDispNoRot = rrDisplacement - robotRotDisplacement;
        rfDispNoRot = rfDisplacement - robotRotDisplacement;
        lrDispNoRot = lrDisplacement - robotRotDisplacement;

        lfDispNoRotTot += robotRotDisplacement;
        rrDispNoRotTot += robotRotDisplacement;
        rfDispNoRotTot += robotRotDisplacement;
        lrDispNoRotTot += robotRotDisplacement;

        omniDriveAngle = motorAngleToTarget;

        if (Math.abs(Math.cos(omniDriveAngle * Math.PI / 180)) > 0.707) {
            omniDriveFactor = Math.abs(Math.cos(omniDriveAngle * Math.PI / 180));
        }
        else if (Math.abs(Math.sin(omniDriveAngle * Math.PI / 180)) > 0.707) {
            omniDriveFactor = Math.abs(Math.sin(omniDriveAngle * Math.PI / 180));
        }
        else {
            omniDriveFactor = 1.0;
        }


        //calculate X displacement, and convert ticks to inches (3.543 = wheel diameter inches, 560 = ticks per wheel rot), and account for robot angle
        //robotXdisplacement = ((-rfDispNoRot - lfDispNoRot + rrDispNoRot + lrDispNoRot) / 4) * ((3.543 * Math.PI) / 540) / omniDriveFactor;
        //robotXdisplacement = (((rrDispNoRot - lfDispNoRot) / 2) + ((lrDispNoRot + rfDispNoRotTot) / 2)) * ((3.543 * Math.PI) / 540) / omniDriveFactor;
        robotXdisplacement = ((rrDispNoRot - lfDispNoRot) / 2) * ((3.543 * Math.PI) / 540);
        //calculate Y displacement, and convert ticks to inches (3.543 = wheel diameter inches, 560 = ticks per wheel rot), and account for robot angle
        //robotYdisplacement = ((rfDispNoRot - lfDispNoRot + rrDispNoRot - lrDispNoRot) / 4) * ((3.543 * Math.PI) / 540) / omniDriveFactor;
        robotYdisplacement = ((-lrDispNoRot + rfDispNoRot) / 2) * ((3.543 * Math.PI) / 540);


        robotVectorByOdo = 45 + Math.atan2(robotYdisplacement, robotXdisplacement) * 180 / Math.PI;

        robotVectorMag = Math.sqrt((robotXdisplacement * robotXdisplacement) + (robotYdisplacement * robotYdisplacement));

        robotFieldAngle = (robotVectorByOdo + getAngle());

        robotFieldPositionX = robotVectorMag * Math.cos(robotFieldAngle * Math.PI / 180);  //field position in inches
        robotFieldPositionY = robotVectorMag * Math.sin(robotFieldAngle * Math.PI / 180);  //field position in inches

        xPosition += robotFieldPositionX;
        yPosition += robotFieldPositionY;


        double [] displacement = {xPosition, yPosition, robotVectorByOdo};
        return displacement;

    }

    private double [] findDisplacementOptical(){

        //First, determine the robot z movement, so encoder ticks caused by z movement can be removed from x, y movement
        robotRotNewOpt = getAngle();
        robotRotOpt = robotRotNewOpt - robotRotOldOpt;
        robotRotOldOpt = robotRotNewOpt;

        //robot rotation (each loop) expressed in motor ticks (robot angle, ticks per degree robot rotation...determined through testing for each encoder wheel).
        //double robotRotDisplacementOptFront = robotRotOpt * 0; //21.390 ticks per degree of robot rotation
        double robotRotDisplacementOptRight = robotRotOpt * 44.542; //21.085each wheel is mounted slightly different on the bot
        double robotRotDisplacementOptLeft = robotRotOpt * 40.914; //21.318
        double robotRotDisplacementOptBack = robotRotOpt * 11.861; //21.093

        //measure encoder position
        //frontPositionOptical = frontEncoder.getCurrentPosition();
        frontPositionOptical = 0;
        rightPositionOptical = rightEncoder.getCurrentPosition();
        leftPositionOptical = leftEncoder.getCurrentPosition();
        backPositionOptical = backEncoder.getCurrentPosition();

        //encoder ticks for each sensor, each loop
        //double frontDisplacement = frontPositionOptical - frontDisplacementOld;
        double leftDisplacement = leftPositionOptical - leftDisplacementOld;
        double rightDisplacement = rightPositionOptical - rightDisplacementOld;
        double rearDisplacement = backPositionOptical - rearDisplacementOld;

        frontDisplacementOld = frontPositionOptical;
        leftDisplacementOld = leftPositionOptical;
        rightDisplacementOld = rightPositionOptical;
        rearDisplacementOld = backPositionOptical;

        //Now, remove the ticks caused by z movement from the total encoder count, each loop
        //double frontDispNoRot = frontDisplacement - robotRotDisplacementOptFront;
        double leftDispNoRot = leftDisplacement - robotRotDisplacementOptRight;
        double rightDispNoRot = rightDisplacement - robotRotDisplacementOptLeft;
        double rearDispNoRot = rearDisplacement - robotRotDisplacementOptBack;

        //This section was created for debugging
        //frontDispNoRotTotOpt += frontDispNoRot;
        leftDispNoRotTotOpt += leftDispNoRot;
        rightDispNoRotTotOpt += rightDispNoRot;
        rearDispNoRotTotOpt += rearDispNoRot;

        //The optical encoders are mounted inline to the robot preferred direction, so we can just use robot angle to target...difference between robot target angle and gyro angle
//        omniDriveAngle = robotAngleToTarget + 45;

        //Determine the omni-Drive wheel effective gear ratio
        if (Math.abs(Math.cos(robotAngleToTarget * Math.PI / 180)) > 0.707) {
            omniDriveFactorOpt = Math.abs(Math.cos(robotAngleToTarget * Math.PI / 180));
        }
        else if (Math.abs(Math.sin(robotAngleToTarget * Math.PI / 180)) > 0.707) {
            omniDriveFactorOpt = Math.abs(Math.sin(robotAngleToTarget * Math.PI / 180));
        }
        else {
            omniDriveFactorOpt = 1.0;
        }

        //calculate X displacement, and convert ticks to inches (2.362 = wheel diameter inches, 1440 = ticks per wheel rot), and account for robot angle and omni wheel effect
//        robotXdisplacementOpt = ((-frontDispNoRot + rearDispNoRot) / 2) * ((2.362 * Math.PI) / 8192);
        robotXdisplacementOpt = rearDispNoRot * ((2.362 * 1.01 * Math.PI) / 8192); //added 2% error factor to improve accuracy.  Maybe the wheels are bigger than 60mm
//        robotXdisplacementOpt = ((-frontDispNoRot + rearDispNoRot) / 2) * ((2.362 * Math.PI) / 1440) / omniDriveFactorOpt;
        //robotXdisplacementOptTot = ((rightDispNoRotTotOpt + leftDispNoRotTotOpt) / 2 + (-frontDispNoRotTotOpt + rearDispNoRotTotOpt) / 2) * ((2.362 * 1.01 * Math.PI) / 8192);
        //calculate Y displacement, and convert ticks to inches (2.362 = wheel diameter inches, 1440 = ticks per wheel rot), and account for robot angle and omni wheel effect
        robotYdisplacementOpt = ((rightDispNoRot - leftDispNoRot) / 2) * ((2.362 * 1.01 * Math.PI) / 8192);
//        robotYdisplacementOpt = ((rightDispNoRot - leftDispNoRot) / 2) * ((2.362 * Math.PI) / 1440) / omniDriveFactorOpt;
        //robotYdisplacementOptTot = (((rightDispNoRotTotOpt - leftDispNoRotTotOpt) / 2) + ((frontDispNoRotTotOpt + rearDispNoRotTotOpt) / 2)) * ((2.362 * 1.01 * Math.PI) / 8192);

        //Using inverse kinematics, calculate the robot driving direction, from the encoder measurements
        robotVectorByOdoOpt = Math.atan2(robotYdisplacementOpt, robotXdisplacementOpt) * 180 / Math.PI;

        //Now that we know the robot driving direction, calculate the driving distance, each loop
        robotVectorMagOpt = Math.sqrt((robotXdisplacementOpt * robotXdisplacementOpt) + (robotYdisplacementOpt * robotYdisplacementOpt));

        //The calculated robot vector is the direction in field centric.  Adding the robot gyro angle was an error.
        robotFieldAngleOpt = (robotVectorByOdoOpt + getAngle());
        //robotFieldAngleOpt = robotVectorByOdoOpt;

        //Now we know the driving direction and distance for each loop, use forward kinematics calculation to determine x, y movement, each loop
        double robotFieldPositionXOpt = robotVectorMagOpt * Math.cos(robotFieldAngleOpt * Math.PI / 180);  //field position in inches
        double robotFieldPositionYOpt = robotVectorMagOpt * Math.sin(robotFieldAngleOpt * Math.PI / 180);  //field position in inches

        //Add each x, y loop calculation, to track the robot location on the field
        xPositionOpt += robotFieldPositionXOpt;
        yPositionOpt += robotFieldPositionYOpt;

        if (debugFlag) {
            RobotLog.d("findDisplacementOptical - runTime %f, deltaTime %f, leftDispNoRot %f, rightDispNoRot %f, rearDispNoRot %f, xPositionOptical %f, yPositionOptical %f, xPower %f, yPower %f, zPower %f, robotRotNewOpt %f, robotVectorByOdoOpt %f, robotFieldAngleOpt %f",
                    currentTime, loopTime, leftDispNoRot, rightDispNoRot, rearDispNoRot, xPositionOpt, yPositionOpt, xPower, yPower, zPower, robotRotNewOpt, robotVectorByOdoOpt, robotFieldAngleOpt);

        }

        //Store the encoder positions and x, y locations in an array and return the values
        double [] positionOptical = {frontPositionOptical, rightPositionOptical, leftPositionOptical, backPositionOptical, xPositionOpt, yPositionOpt, robotVectorByOdoOpt};
        return positionOptical;


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

    public void followCurve(ArrayList<ArmShoulderPositions.CurvePoint> allPoints, double zPowerFF, double distanceToPark, double parkAngleTarget, double parkRadius){

        startTime = elapsedTime.seconds();
        oldTime = startTime;
        distanceToEndPoint = 10;
        NerdPID_PurePursuit.resetIntError();
        zPowerStart = zPowerFF;
        zPowerIncrease = 0.075;
        angleStart = getAngle() + 90;
        angleIncrement = ((parkAngleTarget - angleStart) / (distanceToEndPoint - distanceToPark));
        robotFaceAngle = angleStart;

        while (this.opmode.opModeIsActive() && !this.opmode.isStopRequested() && !distanceTargetReached(distanceToEndPoint, parkRadius)) {

//            for (int i = 0; i < allPoints.size() - 1; i++) {
//                ComputerDebugging.sendLine(new FloatPoint(allPoints.get(i).x, allPoints.get(i).y),
//                        new FloatPoint(allPoints.get(i + 1).x, allPoints.get(i + 1).y));
//            }

            double[] robotPositionXYV = findDisplacementOptical();

            ArmShoulderPositions.CurvePoint followMe = getFollowPointPath(allPoints, new PointPP(robotPositionXYV[4], robotPositionXYV[5]),
                    allPoints.get(0).followDistance);

            ArmShoulderPositions.CurvePoint endPoint = getEndPoint(allPoints, new PointPP(robotPositionXYV[4], robotPositionXYV[5]),
                    allPoints.get(0).followDistance);

            ArmShoulderPositions.CurvePoint startPath = getStartPath(allPoints, new PointPP(robotPositionXYV[4], robotPositionXYV[5]),
                    allPoints.get(0).followDistance);

//            ComputerDebugging.sendKeyPoint(new FloatPoint(followMe.x, followMe.y));

            distanceToEndPoint = Math.hypot(endPoint.x - robotPositionXYV[4], endPoint.y - robotPositionXYV[5]);

            if(distanceToEndPoint < distanceToPark){
                goToPositionEndPP(endPoint.x, endPoint.y, 1.0, parkAngleTarget, 0.2, distanceToPark);
            }
            else {
                goToPositionPP(followMe.x, followMe.y, followMe.moveSpeed, zPowerFF, followMe.turnSpeed, parkAngleTarget, distanceToPark, startPath.x, startPath.y, endPoint.x, endPoint.y);
            }

//            if (debugFlag) {
//                RobotLog.d("FollowCurve - distanceToEndPoint %f, endPoint.x %f, endPoint.y %f, followMe.x %f, followMe.y %f, robotXPosition %f, robotYPosition %f, followMe.moveSpeed %f, followAngle %f, followMe.turnspeed %f",
//                        distanceToEndPoint, endPoint.x, endPoint.y, followMe.x, followMe.y, robotPositionXYV[4], robotPositionXYV[5], followMe.moveSpeed, followAngle, followMe.turnSpeed);
//            }

        }
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
    }

    public void followCurveArm(ArrayList<ArmShoulderPositions.CurvePoint> allPoints, double zPowerFF, double distanceToPark, double parkAngleTarget,
                               double parkRadius, ArmShoulderPositions initialArmPosition, ArmShoulderPositions targetShoulderPosition, FingerPositions targetFingerPosition, FingerPositions endFingerPosition,
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




        ArmShoulderPositions originalArmTargetPosition = targetShoulderPosition;
        ArmShoulderPositions intermediateArmTargetPosition = ArmShoulderPositions.HOME;
        ArmShoulderPositions currentArmTargetPosition;
        ArmShoulderPositions previousArmPosition = initialArmPosition;

        RobotLog.d("originalArmTargetPosition %d, intermediateArmTargetPosition %d, targetShoulderPosition %d, previousArmPosition %d",
                originalArmTargetPosition.getArmTarget(), intermediateArmTargetPosition.getArmTarget(),targetShoulderPosition.getArmTarget(),previousArmPosition.getArmTarget());


        boolean finalArmTargetReached = false;
        double armDelayTimer = 0.0;
        if(armDelay >0)
            armDelayTimer = armElapsedTime.seconds();

        currentArmTargetPosition = intermediateArmTargetPosition;

        armHoldStartTime = 0.0;

        leftGrab.setPosition(targetFingerPosition.getLeftFingerPosition());
        rightGrab.setPosition(targetFingerPosition.getRightFingerPosition());

        while (this.opmode.opModeIsActive() && !this.opmode.isStopRequested() &&
                !distanceTargetReached(distanceToEndPoint, parkRadius) &&
                !finalArmTargetReached && !isArmHoldTimeReached(armHoldPositionTime) ) {

            //ARM_11_08

            currentTimeArm = armElapsedTime.seconds();
            loopTimeArm = currentTimeArm - oldTimeArm;
            oldTimeArm = currentTimeArm;
            deltaTimeArm = currentTimeArm - startTimeArm;


//            for (int i = 0; i < allPoints.size() - 1; i++) {
//                ComputerDebugging.sendLine(new FloatPoint(allPoints.get(i).x, allPoints.get(i).y),
//                        new FloatPoint(allPoints.get(i + 1).x, allPoints.get(i + 1).y));
//            }

            double[] robotPositionXYV = findDisplacementOptical();

            ArmShoulderPositions.CurvePoint followMe = getFollowPointPath(allPoints, new PointPP(robotPositionXYV[4], robotPositionXYV[5]),
                    allPoints.get(0).followDistance);

            ArmShoulderPositions.CurvePoint endPoint = getEndPoint(allPoints, new PointPP(robotPositionXYV[4], robotPositionXYV[5]),
                    allPoints.get(0).followDistance);

            ArmShoulderPositions.CurvePoint startPath = getStartPath(allPoints, new PointPP(robotPositionXYV[4], robotPositionXYV[5]),
                    allPoints.get(0).followDistance);

//            ComputerDebugging.sendKeyPoint(new FloatPoint(followMe.x, followMe.y));

            distanceToEndPoint = Math.hypot(endPoint.x - robotPositionXYV[4], endPoint.y - robotPositionXYV[5]);

            if(distanceToEndPoint < distanceToPark){
                goToPositionEndPP(endPoint.x, endPoint.y, 1.0, parkAngleTarget, 0.2, distanceToPark);
            }
            else {
                goToPositionPP(followMe.x, followMe.y, followMe.moveSpeed, zPowerFF, followMe.turnSpeed, parkAngleTarget, distanceToPark, startPath.x, startPath.y, endPoint.x, endPoint.y);
            }

            //Skip the ARM portion if a delay is requested.
            if(armDelay > 0){
                if((armElapsedTime.seconds() - armDelayTimer) < armDelay) continue;
            }

            //ARM Start
            double armPidOutput = 0.0;
            double armMotorsign = 1.0;
            double armMotorPower = 0.0;

            RobotLog.d("NERD FRONT ONE #### FollowCurveArm - currentArmTargetPosition %d", frontEncoder.getCurrentPosition());


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
            RobotLog.d("NERDBLUEAUTON Motor powers %f, LooptimeARM %f", armMotorPower, loopTimeArm);
            frontEncoder.setPower(armMotorPower);
            rightEncoder.setPower(-armMotorPower);
            leftArmServo.setPosition(currentArmTargetPosition.getLeftWristServoPosition());
            rightArmServo.setPosition(currentArmTargetPosition.getRightWristServoPosition());

            RobotLog.d("originalArmTargetPosition %d, intermediateArmTargetPosition %d, targetShoulderPosition %d, previousArmPosition %d",
                    originalArmTargetPosition.getArmTarget(), intermediateArmTargetPosition.getArmTarget(),targetShoulderPosition.getArmTarget(),previousArmPosition.getArmTarget());

            if(currentArmTargetPosition.getArmTarget() == intermediateArmTargetPosition.getArmTarget()){

                if(isArmTargetReached(currentArmTargetPosition, frontEncoder.getCurrentPosition())){
                    RobotLog.d("NERD_11_08 #### FollowCurveArm - ArmTarget Reached SWAPPED, originalArmTargetPosition %d, currentArmTargetPosition %d, frontEncoder.getCurrentPosition %d",
                            originalArmTargetPosition.getArmTarget(),currentArmTargetPosition.getArmTarget(),frontEncoder.getCurrentPosition() );
                    previousArmPosition = currentArmTargetPosition;
                    currentArmTargetPosition = originalArmTargetPosition;
                }

                RobotLog.d("NERD_11_08 #### FollowCurveArm - Did NOT SWAP originalArmTargetPosition %d, currentArmTargetPosition %d, frontEncoder.getCurrentPosition %d",
                        originalArmTargetPosition.getArmTarget(),currentArmTargetPosition.getArmTarget(),frontEncoder.getCurrentPosition() );

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

            //ARM 11_08

            if(motor.equals("intake")) {
                //   if(finalArmTargetReached == true && originalArmTargetPosition == ArmShoulderPositions.INTAKE)
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


    private ArmShoulderPositions.CurvePoint getFollowPointPath(ArrayList<ArmShoulderPositions.CurvePoint> pathPoints, PointPP robotLocation, double followRadius){
        ArmShoulderPositions.CurvePoint followMe = new ArmShoulderPositions.CurvePoint(pathPoints.get(0));

        for(int i = 0; i < pathPoints.size() - 1; i++){
            ArmShoulderPositions.CurvePoint startLine = pathPoints.get(i);
            ArmShoulderPositions.CurvePoint endline = pathPoints.get(i + 1);

            double[] robotPositionXYV = findDisplacementOptical();

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
                double distance = Math.hypot(thisIntersection.x - robotPositionXYV[4], thisIntersection.y - robotPositionXYV[5]);

                if(distance < closestDistance){
                    closestDistance = distance;
                    followMe.setPoint(thisIntersection);
                }
            }
        }

//        if (debugFlag) {
//            RobotLog.d("CurvePoint - followMe.x %f, followMe.y %f, followMe.moveSpeed %f, followMe.turnSpeed %f, followRadius %f, pointLength %f, slowDownTurnAmount %f, slowDownTurnRadians %f",
//                    followMe.x, followMe.y, followMe.moveSpeed, followMe.turnSpeed, followRadius, followMe.pointLength, followMe.slowDownTurnAmount, followMe.slowDownTurnRadians);
//        }

        return followMe;
    }

    private ArmShoulderPositions.CurvePoint getEndPoint(ArrayList<ArmShoulderPositions.CurvePoint> pathPoints, PointPP robotLocation, double followRadius){

        int endPathCount = pathPoints.size() - 2;
        ArmShoulderPositions.CurvePoint endPoint = new ArmShoulderPositions.CurvePoint(pathPoints.get(endPathCount));

        return endPoint;
    }

    private ArmShoulderPositions.CurvePoint getStartPath(ArrayList<ArmShoulderPositions.CurvePoint> pathPoints, PointPP robotLocation, double followRadius){

        ArmShoulderPositions.CurvePoint startPath = new ArmShoulderPositions.CurvePoint(pathPoints.get(0));

        return startPath;
    }


    public void goToPositionPP(double x, double y, double movementSpeed, double zPowerFeedForward, double turnSpeed, double parkAngleTarget, double parkDistance,
                               double robotPositionXStart, double robotPositionYStart, double endPointX, double endPointY){

        currentTime = elapsedTime.seconds();
        loopTime = currentTime - oldTime;
        oldTime = currentTime;
//        deltaTime = currentTime - startTime;

        robotPositionXY = findDisplacement(xPosition, yPosition, robotVectorByOdo);
        robotPositionXYOptical = findDisplacementOptical();

//        distanceToTarget = Math.hypot(x - robotPositionXY[0], y - robotPositionXY[1]);
        distanceToTarget = Math.hypot(x - robotPositionXYOptical[4], y - robotPositionXYOptical[5]);

        double absoluteAngleToTarget = Math.atan2(y - robotPositionXYOptical[5], x - robotPositionXYOptical[4]) * 180 / Math.PI;

//            if (distanceToTarget < 5) {
//                robotTargetSpeed = 0;
//            } else {
//                robotTargetSpeed = movementSpeed;
//            }

        robotTargetAngle = absoluteAngleToTarget;

//        robotAngleToTarget = MathFunctions.AngleWrapDeg(robotTargetAngle - getAngle());
        motorAngleToTarget = MathFunctions.AngleWrapDeg((robotTargetAngle - 45) - getAngle());

        xPower = Math.cos(motorAngleToTarget * 3.14 / 180) * movementSpeed;
        yPower = Math.sin(motorAngleToTarget * 3.14 / 180) * movementSpeed;

        double relativeTurnAngle = MathFunctions.AngleWrapDeg(robotTargetAngle - (getAngle() + 90));

//        for (int i = 0; i < 2; i++){
//            angleIncrement = ((parkAngleTarget - angleStart) / (distanceToEndPoint - parkDistance));
//            robotFaceAngle = angleStart;
//            robotPositionXStart = robotPositionXYOptical[4];
//            robotPositionYStart = robotPositionXYOptical[5];
//        }

        double distanceFromStart = Math.hypot(robotPositionXYOptical[4] - robotPositionXStart, robotPositionXYOptical[5] - robotPositionYStart);
        double distanceAtStart = Math.hypot(endPointX - robotPositionXStart, endPointY - robotPositionYStart);
        angleIncrement = (parkAngleTarget - angleStart) / (distanceAtStart - parkDistance);

        if (angleStart < parkAngleTarget){
            robotFaceAngle = Range.clip(angleIncrement * distanceFromStart + angleStart, angleStart, parkAngleTarget);
        }else if (angleStart > parkAngleTarget){
            robotFaceAngle = Range.clip(angleIncrement * distanceFromStart + angleStart, parkAngleTarget, angleStart);
        }

        zPIDAngle = 90 + getAngle();
        robotTurnSpeed = NerdPID_PurePursuit.zPowerDrive(robotFaceAngle, zPIDAngle, loopTime);

//        if (Math.abs(parkAngleTarget - (getAngle() + 90)) > 90){
//            useZPID = false;
//        }
//        else if (Math.abs(parkAngleTarget - (getAngle() + 90)) < 80){
//            useZPID = true;
//        }
//
//        if (useZPID){
//            zPIDAngle = 90 + getAngle();
//            robotTurnSpeed = NerdPID_PurePursuit.zPowerDrive(parkAngleTarget, zPIDAngle, loopTime);
//        }
//        else if (!useZPID){
//            robotTurnSpeed = Range.clip((parkAngleTarget - (getAngle() + 90)) / 30, -1, 1) * turnSpeed;
//        }

        double robotTurnSpeedFF = Range.clip((zPowerStart + zPowerIncrease), -0.5, 0);
        zPowerStart = robotTurnSpeedFF;




//        zPower = Range.clip(relativeTurnAngle / 30, -0.3, 0.3) * robotTurnSpeed;
        zPower = Range.clip((robotTurnSpeed + zPowerStart), -0.3, 0.3);

        //Second calculate motor speeds for angular (z) motion

//            robotCircumference = 2 * Math.PI * robotRadius;
//            robotWheelCircumference = 2 * Math.PI * robotWheelRadius;
//            wheelRotPerRobotRot = robotCircumference / robotWheelCircumference;

        frontLeftMotorPower = -xPower + zPower;
        rearRightMotorPower = xPower + zPower;
        frontRightMotorPower = yPower + zPower;
        rearLeftMotorPower = -yPower + zPower;

//        frontLeftMotor.setPower(frontLeftMotorPower);
//        rearRightMotor.setPower(rearRightMotorPower);
//        frontRightMotor.setPower(frontRightMotorPower);
//        rearLeftMotor.setPower(rearLeftMotorPower);

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
            RobotLog.d("goToPositionPP - runTime %f, deltaTime %f,angleIncrement %f, parkAngleTarget %f, angleStart %f, distanceAtStart %f, parkDistance %f, robotFaceAngle %f, robotPositionXStart %f, robotPositionYStart %f, distanceFromStart %f, robotVectorByOdo %f, robotVectorMag %f, robotFieldAngle %f, omniDriveAngle %f, xPower %f, yPower %f, zPower %f, relativeTurnAngle %f, motorAngleToTarget %f, robotTargetAngle %f, getAngle() %f",
                    currentTime, deltaTime, angleIncrement, parkAngleTarget, angleStart, distanceAtStart, parkDistance, robotFaceAngle, robotPositionXStart, robotPositionYStart, distanceFromStart, robotVectorByOdo, robotVectorMag, robotFieldAngle, omniDriveAngle, xPower, yPower, zPower, relativeTurnAngle, motorAngleToTarget, robotTargetAngle, getAngle());
        }


//        if (debugFlag) {
//            RobotLog.d("goToPositionPP - runTime %f, deltaTime %f,robotPositionXY[0] %f, robotPositionXY[1] %f, distanceToTarget %f, robotFaceAngle %f, robotAngleToTarget %f, robotVectorByOdo %f, robotVectorMag %f, robotFieldAngle %f, omniDriveAngle %f, xPower %f, yPower %f, zPower %f, relativeTurnAngle %f, motorAngleToTarget %f, robotTargetAngle %f, getAngle() %f, frontLeftMotorSpeed %f, frontLeftMotorPower %f, frontRightMotorSpeed %f, frontRightMotorPower%f",
//                    currentTime, deltaTime, robotPositionXYOptical[4], robotPositionXYOptical[5], distanceToTarget, robotFaceAngle, robotAngleToTarget, robotVectorByOdo, robotVectorMag, robotFieldAngle, omniDriveAngle, xPower, yPower, zPower, relativeTurnAngle, motorAngleToTarget, robotTargetAngle, getAngle(), frontLeftMotorSpeed, motorSpeedCommand[0], frontRightMotorSpeed, motorSpeedCommand[1]);
//        }

    }

    public void goToPositionEndPP(double x, double y, double movementSpeed, double targetAngleForPark, double turnSpeed, double distanceToEndPoint){
//        startTime = runtime;

//        distanceToTarget = 10;
        distanceToTargetAngle = getAngle() + 90;

//        double [] robotPositionXYOpticalInit = findDisplacementOptical();
//        xPositionOpticalInit = robotPositionXYOpticalInit[4];
//        yPositionOpticalInit = robotPositionXYOpticalInit[5];

        //First calculate motor speeds for linear (x, y) motion

        currentTime = elapsedTime.seconds();
        loopTime = currentTime - oldTime;
        oldTime = currentTime;
//        deltaTime = currentTime - startTime;

        robotPositionXY = findDisplacement(xPosition, yPosition, robotVectorByOdo);

        robotPositionXYOptical = findDisplacementOptical();

        //distanceToTarget = Math.hypot(x - robotPositionXY[0], y - robotPositionXY[1]);
        distanceToTarget = Math.hypot(x - robotPositionXYOptical[4], y - robotPositionXYOptical[5]);

        double absoluteAngleToTarget = Math.atan2(y - robotPositionXYOptical[5], x - robotPositionXYOptical[4]) * 180 / Math.PI;

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

//        frontLeftMotor.setPower(frontLeftMotorPower);
//        frontRightMotor.setPower(frontLeftMotorPower);
//        rearLeftMotor.setPower(rearLeftMotorPower);
//        rearRightMotor.setPower(rearRightMotorPower);

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




//            if (debugFlag) {
//                RobotLog.d("goToPosition - timeSinceStart %f, robotTargetAngle %f, xPower %f, yPower %f , zPower %f, frontLeftMotorPower %f, rearRightMotorPower %f , frontRightMotorPower %f, rearLeftMotorPower %f, frontLeftMotorTicks %f, rearRightMotorTicks %f , frontRightMotorTicks %f , rearLeftMotorTicks %f, xPosition %f, yPosition %f, robotRot %f, robotRotDisplacement %f, robotAngleToTarget %f, robotVectorByOdoF %f, robotVectorByOdoR %f, frontVectorMag %f, rearVectorMag %f",
//                        deltaTime, robotTargetAngle, xPower, yPower, zPower, frontLeftMotorPower, rearRightMotorPower, frontRightMotorPower, rearLeftMotorPower, lfDisplacement, rrDisplacement, rfDisplacement, lrDisplacement, xPosition, yPosition, robotRot, robotRotDisplacement, robotAngleToTarget, robotVectorByOdoF, robotVectorByOdoR, frontVectorMag, rearVectorMag);
//            }

        // channels to record for velocity following
//            if (debugFlag) {
//                RobotLog.d("goToPosition - deltaTime %f, frontLeftMotorTarget %f, frontLeftMotorSpeed %f, frontRightMotorTarget %f, frontRightMotorSpeed %f, rearLeftMotorTarget %f, rearLeftMotorSpeed %f, rearRightMotorTarget %f, rearRightMotorSpeed %f, frontLeftMotorPower %f, frontRightMotorPower %f, rearLeftMotorPower %f, rearRightMotorPower %f, robotTargetAngle %f, robotAngleToTarget %f, zPIDAngle %f, relativeTurnAngle %f, xPosition %f, yPosition %f, frontOpticalEncoder %f, rightOpticalEncoder %f, leftOpticalEncoder %f, backOpticalEncoder %f, xPositionOpt %f, yPositionOpt %f, omniDriveFactorOpt %f",
//                        deltaTime, frontLeftMotorTarget, frontLeftMotorSpeed, frontRightMotorTarget, frontRightMotorSpeed, rearLeftMotorTarget, rearLeftMotorSpeed, rearRightMotorTarget, rearRightMotorSpeed, motorSpeedCommand [0], motorSpeedCommand [1], motorSpeedCommand [2], motorSpeedCommand [3], robotTargetAngle, robotAngleToTarget, zPIDAngle, relativeTurnAngle, robotPositionXY[0], robotPositionXY[1], robotPositionXYOptical[0], robotPositionXYOptical [1], robotPositionXYOptical [2], robotPositionXYOptical[3], robotPositionXYOptical[4], robotPositionXYOptical[5], omniDriveFactorOpt);
//            }

//            // channels to record to debug optical encoder field centric driving
//            if (debugFlag) {
//                RobotLog.d("goToPositionEndPP - deltaTime %f, robotXdisplacementOpt %f, robotYdisplacementOpt %f, robotVectorByOdoOpt %f, robotVectorByOdo %f, robotVectorMagOpt %f, robotVectorMag %f, robotFieldAngleOpt %f, robotFieldAngle %f, relativeTurnAngle %f, robotAngleToTarget %f, robotTargetAngle %f, xPositionOpt %f, xPosition %f, yPositionOpt %f, yPosition %f, omniDriveFactorOpt %f, omniDriveFactor %f, distanceToTarget %f, robotTargetSpeed %f, zPIDPower %f, zSpeedTargetAngle %f, zPIDAngle %f, distanceToTargetAngle %f",
//                        deltaTime, robotXdisplacementOpt, robotYdisplacementOpt, robotVectorByOdoOpt, robotVectorByOdo, robotVectorMagOpt, robotVectorMag, robotFieldAngleOpt, robotFieldAngle, relativeTurnAngle, robotAngleToTarget, robotTargetAngle, xPositionOpt, xPosition, yPositionOpt, yPosition, omniDriveFactorOpt, omniDriveFactor, distanceToTarget, robotTargetSpeed, zPower, zSpeedTargetAngle, zPIDAngle, distanceToTargetAngle);
//
//            }

        // channels to record to debug optical encoder field centric driving
        if (debugFlag) {
            RobotLog.d("goToPositionEndPP - runTime %f, deltaTime %f, distanceToTarget %f, robotTargetSpeed %f, robotTargetSpeedIPS %f, robotSpeed %f, decelRate %f, xPositionOptical %f, yPositionOptical %f, targetParkAngle %f, zPIDAngle %f, xPower %f, yPower %f, zPower %f, robotTargetSpeedPID %f, prevDistanceToTarget %f",
                    currentTime, loopTime, distanceToTarget, robotTargetSpeed, robotTargetSpeedIPS, robotSpeed, decelRate, xPositionOpt, yPositionOpt, targetParkAngle, zPIDAngle, xPower, yPower, zPower, robotTargetSpeedPID, prevDistanceToTarget);

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
            leftArmServo.setPosition(armTargetPosition.getLeftWristServoPosition());
            rightArmServo.setPosition(armTargetPosition.getRightWristServoPosition());

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

    private boolean turnPIDAngleTargetReached(double targetAngle){
        boolean angleTargetReached = false;
        if ((Math.abs(robotTargetAngle - getAngle()) < turnPIDangletolerance) || ((elapsedTime.seconds() - turnPIDtimer) >= 1.0)){
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




