package teamcode.TeleOp;/*
Copyright 2018 FIRST Tech Challenge Team [Phone] SAMSUNG

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/*
Copyright 2018 FIRST Tech Challenge Team [Phone] SAMSUNG

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import teamcode.RobotUtilities.*;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Disabled
@TeleOp(name="NerdBotsTeleop", group="Final")
//@Config
public class NerdBotsTeleOp extends LinearOpMode {

    //FTC Dashboard
    FtcDashboard ftcDashboard;
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

    public  volatile ArmShoulderPositions shoulderPosition = ArmShoulderPositions.INTAKE;
    public  volatile FingerPositions fingerPosition = FingerPositions.ENTER_INTAKE;
    public volatile  ArmShoulderPositions previousShoulderPosition =ArmShoulderPositions.INTAKE;
    public volatile  ArmShoulderPositions currentShoulderPosition =ArmShoulderPositions.INTAKE;

    public static double WRIST_SERVO_INCREMENT=0.0;
    public static double WRIST_SERVO_INCREMENT_STEP = 0.1;

    //If using FTC Dashboard

    public static double ARM_TARGET=0;
    public static double MAX_POWER = 0.4;
    public static double HOME_MAX_POWER = 0.2;
    public static double LEFT_WRIST_SERVO_POSITION=0.0;
    public static double RIGHT_WRIST_SERVO_POSITION=1.0;
    public static double LEFT_FINGER_SERVO_POSITION=0.53;
    public static double RIGHT_FINGER_SERVO_POSITION=0.55;
    public  static  double INTERMEDIATE_SERVO_POSIITON=0.0;

    //Freight Frenzy Arm Variables


    @Override
    public void runOpMode() {


        //hardwaremaps
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
        resetAngle();

        //Initialize Arm Components
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");
        leftArmMotor = hardwareMap.get(DcMotor.class, "leftArmMotor");
        rightArmMotor = hardwareMap.get(DcMotor.class, "rightArmMotor");
        leftGrab = hardwareMap.get(Servo.class, "leftGrab");
        rightGrab = hardwareMap.get(Servo.class, "rightGrab");
        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Positions to get in the intake. This is initial position we will be at the beginning.

        leftArmServo.setPosition(0.3);
        rightArmServo.setPosition(0.7);
        leftGrab.setPosition(0.53);
        rightGrab.setPosition(0.55);
        //End Positions to get in the intake


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)

        //tells the driver when to press start
        if (!imu.isGyroCalibrated()) {
            telemetry.addData("Gyro", "Initialized");
            telemetry.update();
        }

        double armMotorPower = 0.0;

        ftcDashboard = FtcDashboard.getInstance();
        dashboardTelemetry = ftcDashboard.getTelemetry();

        //anything between here and while(opmodeIsActive) runs exactly once, right when the play button is pressed.
        waitForStart();

        //   resetAngle();

        //resets a timer
        ZPIDTime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            HOME_MAX_POWER = 0.4;
            previousShoulderPosition = shoulderPosition;

            currentTime = elapsedTime.seconds();
            loopTime = currentTime - oldTime;
            oldTime = currentTime;
            deltaTime = currentTime - startTime;

            //mapping the joysticks to turning.
            if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 ) {
                joyX = gamepad1.left_stick_x;
                joyY = gamepad1.left_stick_y;
            }

            //using the Dpad to turn. Up turns to 0 degrees, down to 180 degrees, 90 to right, and -90 to left.
            if (gamepad1.dpad_up) {
                joyX = 0;
                joyY = -1;
            } else if (gamepad1.dpad_down) {
                joyX = 0;
                joyY = 1;
            } else if (gamepad1.dpad_left) {
                joyX = -1;
                joyY = 0;
            } else if (gamepad1.dpad_right) {
                joyX = 1;
                joyY = 0;
            }else if(gamepad1.a){  //11_09 added below else if block
                joyX = 1;
                joyY = 1;
            }


            //sets the current angle of the gyro to 0
            if (gamepad1.y) {
                resetAngle();
            }

            //button to re-initialize the gyro. imu not found error comes up.
            if (gamepad1.b) {
                BNO055IMU.Parameters parametersb = new BNO055IMU.Parameters();

                parametersb.mode = BNO055IMU.SensorMode.IMU;
                parametersb.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                parametersb.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                parametersb.loggingEnabled = false;
                parametersb.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
                parametersb.loggingTag = "IMU";
                parametersb.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

                imu.initialize(parametersb);

            }

            driveMath(joyX, joyY);


            //Setting the power to the motors
            frontLeftMotor.setPower(FLMP * mult);
            rearRightMotor.setPower(RRMP * mult);

            rearLeftMotor.setPower(RLMP * mult);
            frontRightMotor.setPower(FRMP * mult);

            //ARM
            if (gamepad2.a){
                WRIST_SERVO_INCREMENT = 0.0;

                if(previousShoulderPosition.equals(ArmShoulderPositions.HOME)) {
                    HOME_MAX_POWER = 0.2;
                }
                shoulderPosition = ArmShoulderPositions.INTAKE;
                fingerPosition = FingerPositions.ENTER_INTAKE;
             }
            if (gamepad2.b) {
                WRIST_SERVO_INCREMENT = 0.0;
                shoulderPosition = ArmShoulderPositions.LEVEL1;

            }
            if (gamepad2.x) {
                WRIST_SERVO_INCREMENT = 0.0;
                shoulderPosition = ArmShoulderPositions.LEVEL2;
            }
            if (gamepad2.y) {

                WRIST_SERVO_INCREMENT = 0.0;
                shoulderPosition = ArmShoulderPositions.LEVEL3;

            }
             if(gamepad2.left_bumper){
                 WRIST_SERVO_INCREMENT = 0.0;
                 shoulderPosition = ArmShoulderPositions.GROUND_PICKUP;
             }

            if(gamepad2.right_bumper){
                WRIST_SERVO_INCREMENT = 0.0;
                shoulderPosition = ArmShoulderPositions.TSE_DROP;
            }

            if(gamepad2.dpad_up){
                fingerPosition = FingerPositions.GRAB;
            }

            if(gamepad2.dpad_down){
                fingerPosition = FingerPositions.ENTER_INTAKE;
            }

            if(gamepad2.dpad_right){
                fingerPosition = FingerPositions.INTAKE_READY;
            }

            if(gamepad2.dpad_left) {
                WRIST_SERVO_INCREMENT = 0.0;
                if(previousShoulderPosition.equals(ArmShoulderPositions.INTAKE)) {
                    HOME_MAX_POWER = 0.1;
                }
                shoulderPosition = ArmShoulderPositions.HOME;

            }

            //Minor Wrist adjustments

            if(gamepad2.right_bumper && gamepad2.dpad_up){
                WRIST_SERVO_INCREMENT += WRIST_SERVO_INCREMENT_STEP;
                WRIST_SERVO_INCREMENT -= WRIST_SERVO_INCREMENT_STEP;
            }

            if(gamepad2.right_bumper && gamepad2.dpad_down){
                WRIST_SERVO_INCREMENT -= WRIST_SERVO_INCREMENT_STEP;
                WRIST_SERVO_INCREMENT += WRIST_SERVO_INCREMENT_STEP;
            }

            intakeMotor.setPower(gamepad1.left_trigger-gamepad1.right_trigger);

            if(gamepad1.x){
                duckyDiskMotor.setPower(1.0);
            }
            //Nov 8 Change
            else {
                duckyDiskMotor.setPower(0);
            }
            //End of Nov 8 Change

//            if(gamepad1.y){
////                duckyDiskMotor.setPower(0);
////            }

            //Minor Wrist adjustments


            double armPidOutput = 0.0;

            if (usingFTCDashboard == true){

                armPidOutput = armPID(ARM_TARGET, leftArmMotor.getCurrentPosition() * -1);
                armMotorsign = Math.signum(armPidOutput);
                if (Math.abs(armPidOutput) > MAX_POWER) {
                    armMotorPower = armMotorsign * MAX_POWER;
                } else {
                    armMotorPower = armPidOutput;
                }

            }else{
                telemetry.addData("target angle", leftArmMotor.getTargetPosition());
                telemetry.addData("actual position", leftArmMotor.getCurrentPosition());
                telemetry.addData("Arm target", shoulderPosition.getArmTarget());
                telemetry.update();
                armPidOutput = armPID(shoulderPosition.getArmTarget(), leftArmMotor.getCurrentPosition() * -1);
                armMotorsign = Math.signum(armPidOutput);
                if(shoulderPosition.equals(ArmShoulderPositions.HOME) || shoulderPosition.equals(ArmShoulderPositions.INTAKE)) {
                    if (Math.abs(armPidOutput) > HOME_MAX_POWER) {
                        armMotorPower = armMotorsign * HOME_MAX_POWER;
                    } else {
                        armMotorPower = armPidOutput;
                    }
                }
                else {
                    if (Math.abs(armPidOutput) > shoulderPosition.getMaxPower()) {
                        armMotorPower = armMotorsign * shoulderPosition.getMaxPower();
                    } else {
                        armMotorPower = armPidOutput;
                    }
                }

            }
//
            leftArmMotor.setPower(armMotorPower);
            rightArmMotor.setPower(armMotorPower);

            if(usingFTCDashboard == true) {

                RIGHT_WRIST_SERVO_POSITION = 1.0 - LEFT_WRIST_SERVO_POSITION;
                dashboardTelemetry.addData("Right Wrist Servo Pos", RIGHT_WRIST_SERVO_POSITION);
                dashboardTelemetry.addData("Left Wrist Servo Pos", LEFT_WRIST_SERVO_POSITION);
                dashboardTelemetry.update();


            }else{

                LEFT_WRIST_SERVO_POSITION = shoulderPosition.getLeftWristServoPosition();
                RIGHT_WRIST_SERVO_POSITION = shoulderPosition.getRightWristServoPosition();
                LEFT_FINGER_SERVO_POSITION = fingerPosition.getLeftFingerPosition();
                RIGHT_FINGER_SERVO_POSITION = fingerPosition.getRightFingerPosition();
             }
                leftArmServo.setPosition(LEFT_WRIST_SERVO_POSITION + WRIST_SERVO_INCREMENT);
                rightArmServo.setPosition(RIGHT_WRIST_SERVO_POSITION - WRIST_SERVO_INCREMENT);
                leftGrab.setPosition(LEFT_FINGER_SERVO_POSITION);
                rightGrab.setPosition(RIGHT_FINGER_SERVO_POSITION);

                if(shoulderPosition.equals(ArmShoulderPositions.INTAKE) && fingerPosition.equals(FingerPositions.ENTER_INTAKE) ){
                    leftGrab.setPosition(FingerPositions.INTAKE_READY.getLeftFingerPosition());
                    rightGrab.setPosition(FingerPositions.INTAKE_READY.getRightFingerPosition());
                }

                previousShoulderPosition = shoulderPosition;

            //ARM

            //add telemetry

//            telemetry.addData("X", FX);
//            telemetry.addData("Y", FY);
//            telemetry.addData("CA", CA);
//            telemetry.addData("RSA", RSA);
//            telemetry.addData("RA", getAngle());
//
//            telemetry.addData("zMag", zMag);
//            telemetry.addData("ZTar", ZTar);
//
//            telemetry.addData("FREV", frontRightMotor.getCurrentPosition());
//            telemetry.addData("FLEV", frontLeftMotor.getCurrentPosition());
//            telemetry.addData("RREV", rearRightMotor.getCurrentPosition());
//            telemetry.addData("RLEV", rearLeftMotor.getCurrentPosition());
//
//
//            telemetry.addData("Status", "Running");
        }
    }


    private double armPID(double targetValue, double currentValue) {


        propError = (targetValue - currentValue);

        intError += (targetValue - currentValue) * loopTime;

        derError = ((targetValue - currentValue) - prevDerError) / loopTime;
        prevDerError = targetValue - currentValue;

        if (Math.abs(targetValue - currentValue) < angletolerance) {
//                    onTarget = true;
//                    runTest = false;
            motorPower = propError * armKp + intError * armKi;
            intError = 0;
        } else {
            motorPower = propError * armKp + intError * armKi + derError * armKd;
        }
//        if (Math.abs(targetValue - currentValue) < 0.5) {
//            motor1.setPower(0);
//       }
        double motorPowersin = Math.signum(motorPower);
        if (Math.abs(motorPower) > maxPower) {
            motorPower = maxPower * motorPowersin;

        }
        return motorPower;
    }

    //function to reset the gyro angle
    private void resetAngle () {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    //PID to control the Z axis for field centric; essentially goes to whatever Z value (degrees) you specify, and keeps it there.
    public void ZPID (double currentAngle, double targetAngle, double kP, double kI, double kD){

        double DError = 0;
        int DBanMin = -1;
        int DBanMax = 1;
        int MaxError = 10;
        double error = 0;
        double speed = 0;
        double TotalError = 0;
        double PrevError = 0;
        double MaxSpeed = 0;


        TotalError = ZTotalError;
        PrevError = ZPrevError;
        PIDTime = ZPIDTime;
        MaxSpeed = MaxSpeedZ;

        //calculate error (Proportional)
        error = targetAngle - currentAngle;

        //Calculate Total error (Integral)
        TotalError = (error * PIDTime.seconds()) + TotalError;

        //do deadban (if you are within 1 degree of the target, dont do anything)
        if (DBanMax > error && error > DBanMin) {
            error = 0;
            //TotalError = 0;
        }

        //calculate delta error (Derivative)
        DError = -(currentAngle/*error*/ - PrevError) / PIDTime.seconds();

        //reset elapsed timer
        PIDTime.reset();

        //Max total error
        if (Math.abs(TotalError) > MaxError) {

            if (TotalError > 0) {
                TotalError = MaxError;
            } else {
                TotalError = -MaxError;
            }

        }


        //Calculate final speed
        speed = (error * kP) + (TotalError * kI) + (DError * kD);


        //Make sure speed is no larger than MaxSpeed
        if (Math.abs(speed) > MaxSpeed) {
            if (speed > 0) {
                speed = MaxSpeed;
            } else {
                speed = -MaxSpeed;
            }
        }

        //not traditional I, if this PID is used for anything else, uncomment "error" and comment out currentAngle.
        PrevError = currentAngle/*error*/;

        //putting the local variables into global variables, so they're not reset.
        ZSpeed = speed;
        ZPrevError = PrevError;
        ZTotalError = TotalError;

    }


    //Function to get the angle of the Gyro sensor
    private double getAngle () {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle;
        deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    private void driveMath (double Xmath, double Ymath) {
        //turns the joystick/dpad outputs into something the math can understand
        zMag = (Xmath * Xmath) + (Ymath * Ymath);

        //Does more math with the joysticks
        if (Math.sqrt(zMag) > 0.5) {
            //11_09
//            ZTar = Math.atan2(-Xmath, -Ymath) * 180 / 3.14159;
            ZTar = Math.atan2(-Xmath, -Ymath) * 180 / 3.14159 + 90;

        }

        //If you press the right bumper, the robot will go slower
        if (isSlowMode) {
            multZ = 0.3;
            mult = 0.5;
        } else {
            multZ = 0.6;
            mult = 1;
        }

        //more complicated math I barely understand. If the drivetrain angle changes, play around with this number.
        // 11_09
//        CA = (Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x) * 180 / 3.14) + 45;
        CA = (Math.atan2(gamepad1.right_stick_y, -gamepad1.right_stick_x) * 180 / 3.14) + 135;


        RSA = (CA - getAngle()) * 3.14 / 180;

        Mag = Math.sqrt((gamepad1.right_stick_y * gamepad1.right_stick_y) + (gamepad1.right_stick_x * gamepad1.right_stick_x));

        FX = -Math.sin(RSA) * Mag;
        FY = -Math.cos(RSA) * Mag;

        //setting the target powers for wach of the motors (RRMP is rear right motor power target, FLMP front left motor power target, etc)
        if(isSlowMode){
            RRMP = -gamepad1.left_stick_x + FX;
            FLMP = -gamepad1.left_stick_x - FX;
            RLMP = -gamepad1.left_stick_x + FY;
            FRMP = -gamepad1.left_stick_x - FY;
            //11_09 added below
            ZTar = 0;

        } else {  //Allowing you to use tank turning if you're using slow mode
            RRMP = (ZSpeed * multZ) + FX; //multZ will only affect Z. This is because if joypad Z is zero then Z is zero.
            FLMP = (ZSpeed * multZ) - FX;
            RLMP = (ZSpeed * multZ) + FY;
            FRMP = (ZSpeed * multZ) - FY;
        }


        //using the PID
        ZPID(getAngle(), ZTar, ZkP, ZkI, ZkD);

    }
}

