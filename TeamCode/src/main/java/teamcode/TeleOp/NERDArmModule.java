package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class NERDArmModule {
    private Servo leftGrab;
    private Servo rightGrab;
    private Servo leftArmServo;
    private Servo rightArmServo;
    private DcMotor leftArmMotor;
    private DcMotor rightArmMotor;
    private LinearOpMode opMode;
    private HardwareMap hardwareMap;

    private double armKp = 0.005;
    private double armKi = 0.0;
    private double armKd = 0.0;
    private double maxPower = 0.4;

    private double previousTime = 0.0;
    private double currentError = 0.0;
    private double totalError = 0.0;

    private double deadBand = 10.0;
    private ElapsedTime elapsedTime ;

    public NERDArmModule(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;

    }

    public void setPIDValues(double kP, double kI, double kD, double maxPower){
        this.armKp = kP;
        this.armKi = kI;
        this.armKd = kD;
        this.maxPower = maxPower;
    }
    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public void initialize() {
//        leftGrab = hardwareMap.get(Servo.class, "LeftGrab");
//        rightGrab = hardwareMap.get(Servo.class, "RightGrab");
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


    }

    public void moveArmUsingPID (double targetPosition){
        elapsedTime = new ElapsedTime();

        double armMotorPower = 0.0;
        double armPidOutput = 0.0;
        double armMotorsign = 1.0;
        currentError = 0.0;
        previousTime = 0.0;
        totalError = 0.0;

        while (this.opMode.opModeIsActive() && !targetReached(targetPosition,this.leftArmMotor.getCurrentPosition())){

            this.opMode.telemetry.addData(("Current Pos"), this.leftArmMotor.getCurrentPosition());
            this.opMode.telemetry.update();

            armMotorPower = getPIDOutput(targetPosition, this.leftArmMotor.getCurrentPosition());
            armMotorsign = Math.signum(armMotorPower);
           if(Math.abs(armPidOutput) > this.maxPower){
               armMotorPower = armMotorsign * this.maxPower;
           }else{
               armMotorPower = armPidOutput;
           }

           this.leftArmMotor.setPower(armMotorPower);
           this.rightArmMotor.setPower(armMotorPower);


        }

//        this.leftArmMotor.setPower(0.0);
//        this.rightArmMotor.setPower(0.0);
        this.opMode.telemetry.addData(("Returning"), this.leftArmMotor.getCurrentPosition());
        this.opMode.telemetry.update();
    }

    public boolean targetReached(double targetPosition, double currentPosition){
        boolean onTarget = false;
        if(Math.abs(targetPosition - currentPosition) <= this.deadBand){

            onTarget = true;
        }

        return onTarget;

    }

    public double getPIDOutput( double targetPosition, int currentPosition)
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

    public void setRightArmServoPosition(double targetPos) {
        rightArmServo.setPosition(targetPos);
    }
    public void setLeftArmServoPosition(double targetPos) {
        leftArmServo.setPosition(targetPos);
    }
    public void setBothArmServoPosition(int targetPos) {
        if(targetPos == 0) {
            rightArmServo.setPosition(1);
            leftArmServo.setPosition(0);
        }
        else if(targetPos == 1) {
            rightArmServo.setPosition(.5);
            leftArmServo.setPosition(.5);
        }
        else if(targetPos == 2) {
            rightArmServo.setPosition(0);
            leftArmServo.setPosition(1);
        }
        else{
            opMode.telemetry.addData("Error", "Not a valid position");
            opMode.telemetry.update();
        }

    }


}
