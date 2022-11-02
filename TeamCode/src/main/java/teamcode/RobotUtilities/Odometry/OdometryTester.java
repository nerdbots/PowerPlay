package teamcode.RobotUtilities.Odometry;
import teamcode.Auton.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//import org.firstinspires.ftc.teamcode.ArmShoulderPositions;
//import org.firstinspires.ftc.teamcode.DuckDetector;
//import org.firstinspires.ftc.teamcode.FingerPositions;

import java.util.ArrayList;

import java.util.ArrayList;

import teamcode.RobotUtilities.Odometry.CurvePoint;
import teamcode.TeleOp.SleeveColorDetector;
import teamcode.TeleOp.SleeveColorDetectorTest;

//@Disabled
@Autonomous(name="OdoMetry Tester", group="Linear Opmode")

public class OdometryTester extends LinearOpMode {

    private PurePursuitRobotMovement6_Turn_MultiThread myPurePursuitRobotMovement6_Turn_MultiThread;
    boolean debugFlag = true;
    double armDelay = 0.0;
    double shippingHubPark = 0;
    int cbValue = 0;
    int path = 2;
    double ParkDistanceX = 0;
    double ParkDistanceY = 0;
    int purePursuitPath = 1;
    int sleeveColor;
    SleeveColorDetector sleeveColorDetector;


    @Override
    public void runOpMode() {

        //Create a NerdBOT object
        myPurePursuitRobotMovement6_Turn_MultiThread = new PurePursuitRobotMovement6_Turn_MultiThread(this);
        myPurePursuitRobotMovement6_Turn_MultiThread.setDebug(debugFlag);

        //Initialize Hardware
        myPurePursuitRobotMovement6_Turn_MultiThread.initializeHardware();


        telemetry.addData("NerdBOT", "Initialized");
        telemetry.update();

        sleeveColorDetector = new SleeveColorDetector(this);
        sleeveColorDetector.initSleeveColorDetector();
        waitForStart();

        telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
        telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
        telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

        telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
        telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());

        telemetry.addData("Thread Active", positionThread.isAlive());
        telemetry.update();

    }
}
