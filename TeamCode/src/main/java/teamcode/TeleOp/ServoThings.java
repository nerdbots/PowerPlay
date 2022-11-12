package teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
@TeleOp(name = "ServoThings")
public class ServoThings extends LinearOpMode {
    private Servo servo1;
    private Servo servo2;

    public void runOpMode() {
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        double joyX = 0;
        double joyY = 0;
        boolean x = gamepad1.x;


        waitForStart();
        servo1.setPosition(0.5);
        servo2.setPosition(0.5);
        while(opModeIsActive()) {




            if (gamepad1.a) {
//                servo2.setPosition(servo2.getPosition() + 0.0005);
                  servo2.setPosition(0.584);
                  servo1.setPosition(0.416);
            }
            else if(gamepad1.b) {

//                servo2.setPosition(servo2.getPosition() - 0.0005);
                servo2.setPosition(0.12);
                servo1.setPosition(0.88);

            }
            if (gamepad1.a && gamepad1.left_stick_y <= -0.95) {
                servo2.setPosition(servo2.getPosition() + 0.0005);


            }
            else if (gamepad1.a && gamepad1.left_stick_y >= 0.95) {
                servo2.setPosition(servo2.getPosition() - 0.0005);


            }
            else if (gamepad1.a && gamepad1.left_stick_y <= -0.95) {
                servo2.setPosition(servo2.getPosition() - 0.0005);


            }
            else if (gamepad1.b && gamepad1.left_stick_y >= 0.95) {
                servo1.setPosition(servo2.getPosition() + 0.0005);


            }
            else if(gamepad1.b && gamepad1.left_stick_y <= -0.95) {

                servo1.setPosition(servo2.getPosition() - 0.0005);


            }


            if(gamepad1.right_stick_y <=-0.95) {
                servo1.setPosition(servo1.getPosition() - 0.0005);

            }
            else if(gamepad1.right_stick_y >=0.95) {
                servo1.setPosition(servo1.getPosition() + 0.0005);

            }



            telemetry.addData("Things", servo1.getPosition());
            telemetry.addData("Things1", servo2.getPosition());
            telemetry.update();

        }

    }

}