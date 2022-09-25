package org.firstinspires.ftc.teamcode;/*
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
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
@TeleOp(name="EncoderDashboard", group="Final")
@Config
public class EncoderDashboard extends LinearOpMode {
    FtcDashboard ftcDashboard;
    Telemetry dashboardTelemetry;

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;

    private DcMotor frontEncoder;
private DcMotor rightEncoder;
    private DcMotor leftEncoder;
    private DcMotor backEncoder;
    //Freight Frenzy Arm Variables


    @Override
    public void runOpMode() {
        initializeHardware();
        ftcDashboard = FtcDashboard.getInstance();
        dashboardTelemetry = ftcDashboard.getTelemetry();

        waitForStart();

        while(opModeIsActive()) {
            dashboardTelemetry.addData("front encoder", frontEncoder.getCurrentPosition());
            dashboardTelemetry.addData("right encoder", rightEncoder.getCurrentPosition());
            dashboardTelemetry.addData("left encoder", leftEncoder.getCurrentPosition());
            dashboardTelemetry.addData("back encoder", backEncoder.getCurrentPosition());
            dashboardTelemetry.update();
        }

    }
    public void initializeHardware(){

        //Initialize Motors


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


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".



        this.frontEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.backEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        this.frontEncoder.setVelocityPIDFCoefficients(200, 0.1, 0, 16);

        this.frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //this.frontEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.backEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //this.frontEncoder.setDirection(DcMotorEx.Direction.REVERSE);
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


        //Initializes motors (obvi)
//        this.wobbleMotor = hardwareMap.get(DcMotor.class, "Left");
//        this.wobbleServo = hardwareMap.get(Servo.class, "wobble_Goal_Servo");


    }


}

