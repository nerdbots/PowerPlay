package teamcode.Auton;

import
        com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//import org.firstinspires.ftc.teamcode.ArmShoulderPositions;
//import org.firstinspires.ftc.teamcode.DuckDetector;
//import org.firstinspires.ftc.teamcode.FingerPositions;

import java.util.ArrayList;

import treamcode.CurvePoint;

//@Disabled
@Autonomous(name="Powerplay_test_auton", group="Linear Opmode")

public class Powerplay_test_auton extends LinearOpMode {

    private PurePursuitRobotMovement6_Turn_MultiThread myPurePursuitRobotMovement6_Turn_MultiThread;

    boolean debugFlag = true;

    double armDelay = 0.0;
    double shippingHubPark = 0;

    int purePursuitPath = 1;
//    DuckDetector.DuckDeterminationPipeline.DuckPosition duckPosition;
//    DuckDetector duckDetector;
//
//    public  volatile ArmShoulderPositions shoulderPosition = ArmShoulderPositions.INTAKE;
//    public  volatile FingerPositions fingerPosition = FingerPositions.ENTER_INTAKE;


    @Override
    public void runOpMode() {

        //Create a NerdBOT object
        myPurePursuitRobotMovement6_Turn_MultiThread = new PurePursuitRobotMovement6_Turn_MultiThread(this);
        myPurePursuitRobotMovement6_Turn_MultiThread.setDebug(debugFlag);

        //Initialize Hardware
        myPurePursuitRobotMovement6_Turn_MultiThread.initializeHardware();


        telemetry.addData("NerdBOT", "Initialized");
        telemetry.update();

        waitForStart();

        myPurePursuitRobotMovement6_Turn_MultiThread.startOdometryThread();


        myPurePursuitRobotMovement6_Turn_MultiThread.printI();

        myPurePursuitRobotMovement6_Turn_MultiThread.resetITerm();

        myPurePursuitRobotMovement6_Turn_MultiThread.printI();

        myPurePursuitRobotMovement6_Turn_MultiThread.resetTimers();


        if (purePursuitPath == 1) {

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 0, 0.7, 0.3, 25, 0, 0.3));
            allPoints.add(new CurvePoint(28, 0, 0.9, 0.3, 25, 0, 0.8));
            allPoints.add(new CurvePoint(18, 24, 0.7, 0.3, 25, 0, 0.3));
            allPoints.add(new CurvePoint(26, 33, 0.7, 0.3, 25, 0, 0.3));
            allPoints.add(new CurvePoint(26, 46, 0.7, 0.3, 25, 0, 0.3));
//            allPoints.add(new CurvePoint(-48, 60, 0.5, 0.3, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurve(allPoints, 0, 15, 45, 3);
sleep(1500);
            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(20, 29, 0.7, 0.3, 25, 0, 0.3));
            allPoints.add(new CurvePoint(12, 25, 0.7, 0.3, 25, 0, 0.3));
            allPoints.add(new CurvePoint(28, 70, 0.7, 0.3, 18, 0, 0.3));
            allPoints.add(new CurvePoint(0, 50, 0.7, 0.3, 20, 0, 0.3));
            allPoints.add(new CurvePoint(-10, 26, 0.7, 0.25, 25, 0, 0.3));
            allPoints.add(new CurvePoint(-22, 54, 0.7, 0.25, 25, 0, 0.3));
            allPoints.add(new CurvePoint(-36, 54, 0.7, 0.25, 25, 0, 0.3));
//            allPoints.add(new CurvePoint(-48, 60, 0.5,  , 25, 180, 0.3));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurve(allPoints, 0, 15, -180, 3);

            sleep(1500);
            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-5, 46, 0.7, 0.3, 20, 0, 0.3));
            allPoints.add(new CurvePoint(3, 55, 0.7, 0.3, 20, 0, 0.3));
            allPoints.add(new CurvePoint(13, 65, 0.7, 0.3, 20, 0, 0.3));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurve(allPoints, 0, 15, -335, 3);
            sleep(1500);
            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-10, 32, 0.7, 0.3, 20, 0, 0.3));
            allPoints.add(new CurvePoint(-22, 62, 0.7, 0.3, 20, 0, 0.3));
            allPoints.add(new CurvePoint(-22, 55, 0.7, 0.3, 20, 0, 0.3));
            allPoints.add(new CurvePoint(-36, 65, 0.7, 0.3, 20, 0, 0.3));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurve(allPoints, 0, 15, -180, 3);
sleep(1500);
            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-5, 46, 0.7, 0.3, 20, 0, 0.3));
            allPoints.add(new CurvePoint(3, 50, 0.7, 0.3, 20, 0, 0.3));
            allPoints.add(new CurvePoint(13, 60, 0.7, 0.3, 20, 0, 0.3));
            myPurePursuitRobotMovement6_Turn_MultiThread.followCurve(allPoints, 0, 15, -40, 3);
sleep(1500);
            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-10, 46, 0.7, 0.3, 20, 0, 0.3));
            allPoints.add(new CurvePoint(-22, 52, 0.7, 0.3, 20, 0, 0.3));
            allPoints.add(new CurvePoint(-22, 54, 0.7, 0.3, 20, 0, 0.3));
            allPoints.add(new CurvePoint(-36, 54, 0.7, 0.3, 20, 0, 0.3));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurve(allPoints, 0, 15, -180, 3);
            sleep(1500);
            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 48, 0.7, 0.3, 20, 0, 0.3));
            allPoints.add(new CurvePoint(10, 54, 0.7, 0.3, 20, 0, 0.3));
            allPoints.add(new CurvePoint(26, 54, 0.7, 0.3, 20, 0, 0.3));
            allPoints.add(new CurvePoint(36, 64, 0.7, 0.3, 20, 0, 0.3));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurve(allPoints, 0, 15, -25, 3);
            myPurePursuitRobotMovement6_Turn_MultiThread.stopOdometryThread();

        }
    }
}


