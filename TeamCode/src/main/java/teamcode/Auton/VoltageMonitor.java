/*
 * Copyright (c) 2018 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.revextensions2.ExpansionHubEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(group = "RevExtensions2Examples")
public class VoltageMonitor extends OpMode
{
    ExpansionHubEx expansionHub;
    //Shoulder Motors
    Telemetry dashboardTelemetry;
    boolean usingFTCDashboard = false;

    //create motor and gyro variables
    private BNO055IMU imu;
    private DcMotor frontRightMotor;
    private DcMotor rearRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor rearLeftMotor;

    private DcMotor duckyDiskMotor;
    private DcMotor intakeMotor;



    //variables for the gyro code
    Orientation angles;
    Acceleration gravity;
    Orientation lastAngles = new Orientation();
    double globalAngle = 0.0;


    //create some timers
    private ElapsedTime ZPIDTime = new ElapsedTime();
    private ElapsedTime PIDTime = new ElapsedTime();

    private ElapsedTime elapsedTime = new ElapsedTime();

    private double ZPrevError = 0;


    private double ZTotalError = 0;


    private double ZSpeed = 0;


    private double ZDerror = 0;


    private double ZkP = 0.013; //0.011
    private double ZkI = 0.000; //0.000
    private double ZkD = 0.0013;//0.00145

    private double ZTar = 0;

    private double MaxSpeedZ = 1.0;

    //variables for movement
    double RRMP = 0;
    double FLMP = 0;
    double RLMP = 0;
    double FRMP = 0;

    double FX = 0;
    double FY = 0;

    double CA = 0;

    double RSA = 0;

    double Mag = 0;
    double zMag = 0;

    double mult = 1; //this controls how fast the robot will move; decrease it to decrease it's speed
    double multZ = 0.6;//this controls how fast the robot will turn; decrease it to decrease it's turn speed (or increase it to increase that. It likely wont work past one)

    double joyX = 0;
    double joyY = 0;



    boolean isSlowMode = false;

    //Freight Frenzy Arm Variables

    //Shoulder Motors
    private DcMotor leftArmMotor;
    private DcMotor rightArmMotor;
    //Wrist servos
    private Servo leftArmServo;
    private Servo rightArmServo;
    //Finger Servos
    private Servo leftGrab;
    private Servo rightGrab;

    //For Arm PID
    public static double armKp = 0.005;//0.01
    public static double armKi = 0.0;
    public static double armKd = 0.0002;
    public static double maxPower = 0.4;

    public ElapsedTime armInitTimer = new ElapsedTime();
    public ElapsedTime resetArmTimer = new ElapsedTime();

    public static int armServoPosition = 0;
    public static int grabPosition = 0;

    public static double armMotorsign = 1.0;
    public static int armTarget = 0;

    public static double rightArmTargetPosition;
    public static double leftArmTargetPosition;
    public static double leftGrabTargetPosition;
    public static double rightGrabTargetPosition;

    double WristjoyX=0;
    double WristjoyY=0;

    double propError = 0;
    double intError = 0;
    double loopTime = 0;
    double prevDerError = 0;
    double derError = 0;
    double angletolerance = 0;
    double motorPower = 0;
    double currentTime = 0;
    double oldTime = 0;
    double deltaTime = 0;
    double startTime = 0;



    public static double WRIST_SERVO_INCREMENT=0.0;
    public static double WRIST_SERVO_INCREMENT_STEP = 0.01;

    //If using FTC Dashboard

    public static double ARM_TARGET=0;
    public static double MAX_POWER = 0.4;
    public static double HOME_MAX_POWER = 0.2;
    public static double LEFT_WRIST_SERVO_POSITION=0.0;
    public static double RIGHT_WRIST_SERVO_POSITION=1.0;
    public static double LEFT_FINGER_SERVO_POSITION=0.53;
    public static double RIGHT_FINGER_SERVO_POSITION=0.55;
    public  static  double INTERMEDIATE_SERVO_POSIITON=0.0;

    @Override
    public void init()
    {
        /*
         * Before init() was called on this user code, REV Extensions 2
         * was notified via OpModeManagerNotifier.Notifications and
         * it automatically took care of initializing the new objects
         * in the hardwaremap for you. Historically, you would have
         * needed to call RevExtensions2.init()
         */
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "Rear_Left_Motor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front_Left_Motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front_Right_Motor");
        rearRightMotor = hardwareMap.get(DcMotor.class, "Rear_Right_Motor");

        duckyDiskMotor = hardwareMap.get(DcMotor.class, "Ducky_Disk");
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
        //initialize the gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        //reset the gyro angle to zero

        //Initialize Arm Components
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        leftArmMotor = hardwareMap.get(DcMotor.class, "leftArmMotor");
        rightArmMotor = hardwareMap.get(DcMotor.class, "rightArmMotor");
        leftGrab = hardwareMap.get(Servo.class, "leftGrab");
        rightGrab = hardwareMap.get(Servo.class, "rightGrab");
        leftArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 3");
    }

    @Override
    public void loop()
    {
        /*
         * ------------------------------------------------------------------------------------------------
         * Voltage monitors
         * ------------------------------------------------------------------------------------------------
         */

        String header =
                "**********************************\n" +
                        "VOLTAGE MONITORS EXAMPLE          \n" +
                        "**********************************\n";
        telemetry.addLine(header);

        telemetry.addData("5v monitor", expansionHub.read5vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)); //Voltage from the phone
        telemetry.addData("12v monitor", expansionHub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)); //Battery voltage
        telemetry.update();
    }
}