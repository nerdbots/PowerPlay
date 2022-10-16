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
package teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the nexfd43t line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Disabled
@TeleOp(name="DriveOnly", group="Final")
public class TeleOpDriving extends LinearOpMode {
    private BNO055IMU imu;
    private DcMotor frontRightMotor;
    private DcMotor rearRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor rearLeftMotor;

    //ARM Start

    private Servo leftGrab;
    private Servo rightGrab;
    private Servo leftArmServo;
    private Servo rightArmServo;
    private DcMotor leftArmMotor;
    private DcMotor rightArmMotor;

    private double armKp = 0.005;
    private double armKi = 0.0;
    private double armKd = 0.0;
    private double maxPower = 0.4;

    private double previousTime = 0.0;
    private double currentError = 0.0;
    private double totalError = 0.0;

    private double deadBand = 10.0;
    private ElapsedTime elapsedTime ;

    int armServoPosition = 0;
    //ARM End

    Orientation angles;
    Acceleration gravity;

    Orientation lastAngles = new Orientation();

    double globalAngle = 0.0;


    private ElapsedTime ZPIDTime = new ElapsedTime();

    private ElapsedTime PIDTime = new ElapsedTime();

    private double ZPrevError = 0;

    private double ZTotalError = 0;


    private double ZSpeed = 0;


    private double ZDerror = 0;


    private double ZkP = 0.013; //0.011
    private double ZkI = 0.000; //0.000
    private double ZkD = 0.0013;//0.00145


    private double ZTar = 0;

    private double MaxSpeedZ = 1.0;

    private  NERDArmModule nerdArm;

    @Override
    public void runOpMode() {
//        leftGrab = hardwareMap.get(Servo.class, "LeftGrab");
//        rightGrab = hardwareMap.get(Servo.class, "RightGrab");

      //  globalAngle = 0;/imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


//        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 2");




        
        /*
        rearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
        
        rearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

*/

        //DRIVING

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        rearLeftMotor = hardwareMap.get(DcMotor.class, "Rear_Left_Motor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front_Left_Motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front_Right_Motor");
        rearRightMotor = hardwareMap.get(DcMotor.class, "Rear_Right_Motor");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);


        resetAngle();



        //Initialize Arm Components
        leftArmServo = hardwareMap.get(Servo.class, "LeftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "RightArmServo");
        leftArmMotor = hardwareMap.get(DcMotor.class, "LeftArmMotor");
        rightArmMotor = hardwareMap.get(DcMotor.class, "RightArmMotor");
        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightArmServo.setPosition(0.5);
        leftArmServo.setPosition(0.5);
        // End Initialize Arm Components

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)


        double positionPitch = 0.52;  // (MAX_POS - MIN_POS) / 2;
        double positionAngle = 0.15;  //(MAX_POS - MIN_POS) / 2; 0.25
        double tapeSpeed = 0.0;

        double LMP = 0;
        double RMP = 0;
        double FMP = 0;
        double BMP = 0;

        double FX = 0;
        double FY = 0;

        double CA = 0;

        double RSA = 0;

        double Mag = 0;
        double zMag = 0;

        double mult = 1; //THIS IS SPEED
        double multZ = 0.6;//0.3

        double power = 1;
        double upMult = 1;

        double joyX = 0;
        double joyY = 0;

        double armTarget=0;

        double armMotorPower = 0.0;
        waitForStart();

        //   resetAngle();

        ZPIDTime.reset();
        elapsedTime = new ElapsedTime();

        //Initial Arm Target and Position.
        armTarget = 80;
        armServoPosition = 1;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


//                if(gamepad1.a) {
//                    rightGrab.setPosition(.5);
//                    leftGrab.setPosition(.9);
//                    sleep(50007);
//                    rightGrab.setPosition(.6);
//                    leftGrab.setPosition(.5);
//                }

                  if(gamepad1.b){

                      maxPower = 0.6;
                      armTarget = 150;
                      armServoPosition = 1;

                  }
                  if(gamepad1.x){
                      maxPower=0.6;
                      armTarget = 250;
                      armServoPosition = 2;
                  }
                  if(gamepad1.y){
                        maxPower = 0.4;
                        armTarget = 50;
                      armServoPosition = 0;

                  }

                  armMotorPower = getArmPIDOutput(armTarget,leftArmMotor.getCurrentPosition());
                  leftArmMotor.setPower(armMotorPower);
                  rightArmMotor.setPower(armMotorPower);
                  setBothArmServoPosition(armServoPosition);

        }
    }

    public void setBothArmServoPosition(int targetPos) {
        if(targetPos == 1) {
            rightArmServo.setPosition(1);
            leftArmServo.setPosition(0);
        }
        else if(targetPos == 0) {
            rightArmServo.setPosition(.5);
            leftArmServo.setPosition(.5);
        }
        else if(targetPos == 2) {
            rightArmServo.setPosition(0);
            leftArmServo.setPosition(1);
        }
        else{
            telemetry.addData("Error", "Not a valid position");
            telemetry.update();
        }

    }

    public double getArmPIDOutput( double targetPosition, int currentPosition)
    {
        double pOutput = 0.0;
        double iOutput = 0.0;
        double dOutput = 0.0;
        double output = 0.0;

        final String funcName = "getOutput";

        //Before we get the error for this cycle, store the error from previous cycle in previous error.
        double prevError = currentError;
        //Store the current time. Will use this for calculating time between 2 cycles - delta time.
        double currTime = elapsedTime.seconds();
        //Find delta time, difference between time in this cycle and the previous.
        double deltaTime = currTime - previousTime;
        //Store the current time into previous time for using in next cycle.
        previousTime = currTime;
        //Call function to get the error based ont the set target and device input
        currentError = targetPosition - currentPosition;

        //Total error for finding I component is sum of errors in each cycle multiplied by delta time.
        totalError += (currentError * deltaTime);
        //Calculate P, I and D outputs.
        pOutput = this.armKp*currentError;
        iOutput = this.armKi*totalError;
        dOutput = deltaTime > 0.0? this.armKd*(currentError - prevError)/(deltaTime * 1000): 0.0;
        //Total PID output.
        //Since we are using device input instead of error for calculating dOutput, we make it negative.
        output = pOutput + iOutput - dOutput;

        //Return the output to the caller.
        return output;
    }


    //0 is rearMotor 1 is frontMotor \/
        private void resetAngle () {
            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            globalAngle = 0;
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
}

