package teamcode.RobotUtilities.Odometry;
import teamcode.Auton.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//import org.firstinspires.ftc.teamcode.ArmShoulderPositions;
//import org.firstinspires.ftc.teamcode.DuckDetector;
//import org.firstinspires.ftc.teamcode.FingerPositions;

import teamcode.TeleOp.SleeveColorDetectorLeftOLD;

@Disabled
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
    SleeveColorDetectorLeftOLD sleeveColorDetector;
    OdometryGlobalCoordinatePosition globalPositionUpdate;


    @Override
    public void runOpMode() {

        //Create a NerdBOT object
        myPurePursuitRobotMovement6_Turn_MultiThread = new PurePursuitRobotMovement6_Turn_MultiThread(this);
        myPurePursuitRobotMovement6_Turn_MultiThread.setDebug(debugFlag);

        //Initialize Hardware
        myPurePursuitRobotMovement6_Turn_MultiThread.initializeHardware();


        telemetry.addData("NerdBOT", "Initialized");
        telemetry.update();

        sleeveColorDetector = new SleeveColorDetectorLeftOLD(this);
        sleeveColorDetector.initSleeveColorDetector();
        waitForStart();
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(myPurePursuitRobotMovement6_Turn_MultiThread.leftEncoder, myPurePursuitRobotMovement6_Turn_MultiThread.rightEncoder, myPurePursuitRobotMovement6_Turn_MultiThread.backEncoder, 194.044, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
    while(opModeIsActive()) {
        telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / 194.044);
        telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / 194.044);
        telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

        telemetry.addData("Left encoder position", myPurePursuitRobotMovement6_Turn_MultiThread.leftEncoder.getCurrentPosition());
        telemetry.addData("Right encoder position", myPurePursuitRobotMovement6_Turn_MultiThread.rightEncoder.getCurrentPosition());
        telemetry.addData("Back encoder position", myPurePursuitRobotMovement6_Turn_MultiThread.backEncoder.getCurrentPosition());

        telemetry.addData("Thread Active", positionThread.isAlive());
        telemetry.update();
    }
    }
}
