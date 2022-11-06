package teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//import org.firstinspires.ftc.teamcode.ArmShoulderPositions;
//import org.firstinspires.ftc.teamcode.DuckDetector;
//import org.firstinspires.ftc.teamcode.FingerPositions;

import java.util.ArrayList;
import teamcode.RobotUtilities.Odometry.CurvePoint;
import teamcode.RobotUtilities.ArmShoulderPositions;
import teamcode.RobotUtilities.FingerPositions;
import teamcode.TeleOp.SleeveColorDetector;

//@Disabled
@Autonomous(name="PowerPlay Red Side", group="Linear Opmode")

public class Powerplay_red_side extends LinearOpMode {

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
    public volatile ArmShoulderPositions shoulderPositions = ArmShoulderPositions.INTAKE;
    public volatile FingerPositions fingerPositions = FingerPositions.GRAB;

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

        sleeveColor = sleeveColorDetector.getAnalysis();
        telemetry.addData("Analysis", sleeveColorDetector.getAnalysis());
        telemetry.update();
        sleeveColorDetector.closeCameraDevice();
        myPurePursuitRobotMovement6_Turn_MultiThread.startOdometryThread();


//        myPurePursuitRobotMovement6_Turn_MultiThread.printI();
//
//        myPurePursuitRobotMovement6_Turn_MultiThread.resetITerm();
//
//        myPurePursuitRobotMovement6_Turn_MultiThread.printI();
//
//        myPurePursuitRobotMovement6_Turn_MultiThread.resetTimers();

        cbValue = sleeveColorDetector.getAnalysis();
        //Detecting purple
        if (cbValue <= 147 & cbValue >= 125) {

            path = 2;
        }
        //Detecting orange
        else if (cbValue <= 105 & cbValue >= 95) {
            path = 1;
        }
        //Detecting green
        else {
            path = 3;
        }


        if (purePursuitPath == 1) {
//            ArrayList<CurvePoint> allPoints = new ArrayList<>();

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 0, 0.5, 0.3, 25, 0, 0.3));
            allPoints.add(new CurvePoint(-21, 0, 0.5, 0.3, 23, 0, 0.8));
            allPoints.add(new CurvePoint(-18, 33, 0.5, 0.3, 25, 0, 0.3));
            allPoints.add(new CurvePoint(-18, 66, 0.5, 0.3, 25, 0, 0.3));
//            allPoints.add(new CurvePoint(-48, 60, 0.5, 0.3, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, 135, 3, ArmShoulderPositions.HOME,ArmShoulderPositions.LEVEL1, FingerPositions.GRAB, FingerPositions.GRAB,0,0, "none", 0);
            myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.LEVEL1, 400,FingerPositions.INTAKE_READY);
//            sleep(1500)
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(-20, 29, 0.7, 0.3, 25, 0, 0.3));
//            allPoints.add(new CurvePoint(-12, 25, 0.7, 0.3, 25, 0, 0.3));
//            allPoints.add(new CurvePoint(-32, 80, 0.7, 0.3, 18, 0, 0.3));
//            allPoints.add(new CurvePoint(0, 56, 0.7, 0.3, 20, 0, 0.3));
//            allPoints.add(new CurvePoint(16, 18, 0.7, 0.25, 25, 0, 0.3));
//            allPoints.add(new CurvePoint(22, 54, 0.7, 0.25, 25, 0, 0.3));
//            allPoints.add(new CurvePoint(36, 54, 0.7, 0.25, 25, 0, 0.3));
////            allPoints.add(new CurvePoint(-48, 60, 0.5,  , 25, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Turn_MultiThread.followCurve(allPoints, 0, 15, 360, 3);
//
//            sleep(1500);
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(5, 46, 0.7, 0.3, 20, 0, 0.3));
//            allPoints.add(new CurvePoint(-2, 54, 0.7, 0.3, 20, 0, 0.3));
//            allPoints.add(new CurvePoint(-12, 64, 0.7, 0.3, 20, 0, 0.3));
//
//            myPurePursuitRobotMovement6_Turn_MultiThread.followCurve(allPoints, 0, 15, 10, 3);
//            sleep(1500);
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(10, 36, 0.7, 0.3, 20, 0, 0.3));
//            allPoints.add(new CurvePoint(22, 55, 0.7, 0.3, 20, 0, 0.3));
//            allPoints.add(new CurvePoint(22, 55, 0.7, 0.3, 20, 0, 0.3));
//            allPoints.add(new CurvePoint(36, 65, 0.7, 0.3, 20, 0, 0.3));
//
//            myPurePursuitRobotMovement6_Turn_MultiThread.followCurve(allPoints, 0, 15, 0, 3);
//            sleep(1500);
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(5, 46, 0.7, 0.3, 20, 0, 0.3));
//            allPoints.add(new CurvePoint(-1, 42, 0.7, 0.3, 20, 0, 0.3));
//            allPoints.add(new CurvePoint(-11, 52, 0.7, 0.3, 20, 0, 0.3));
//            myPurePursuitRobotMovement6_Turn_MultiThread.followCurve(allPoints, 0, 15, -135, 3);
//            sleep(1500);
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(10, 36, 0.7, 0.3, 20, 0, 0.3));
//            allPoints.add(new CurvePoint(22, 52, 0.7, 0.3, 20, 0, 0.3));
//            allPoints.add(new CurvePoint(22, 54, 0.7, 0.3, 20, 0, 0.3));
//            allPoints.add(new CurvePoint(36, 54, 0.7, 0.3, 20, 0, 0.3));
//
//            myPurePursuitRobotMovement6_Turn_MultiThread.followCurve(allPoints, 0, 15, 0, 3);
//            sleep(1500);
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(0, 50, 0.7, 0.3, 20, 0, 0.3));
//            allPoints.add(new CurvePoint(-10, 54, 0.7, 0.3, 20, 0, 0.3));
//            allPoints.add(new CurvePoint(-26, 54, 0.7, 0.3, 20, 0, 0.3));
//            allPoints.add(new CurvePoint(-36, 64, 0.7, 0.3, 20, 0, 0.3));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurve(allPoints, 0, 15, -135, 3);
            myPurePursuitRobotMovement6_Turn_MultiThread.stopOdometryThread();
            if (path == 2) {
                ParkDistanceX = 0;
                ParkDistanceY = 0;

            } else if (path == 1) {
                ParkDistanceY = 32;
                ParkDistanceX = 32;

            } else if (path == 3) {
                ParkDistanceX = 0;
                ParkDistanceY = 0;
            }
        }


    }
}