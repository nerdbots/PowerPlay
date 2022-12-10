package teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import teamcode.RobotUtilities.ArmShoulderPositions;
import teamcode.RobotUtilities.FingerPositions;
import teamcode.RobotUtilities.Odometry.CurvePoint;
import teamcode.TeleOp.SleeveColorDetectorDoubleWebcam;


@Autonomous(name="Powerplay Right 5 TopPole", group="Linear Opmode")

public class Powerplay_Right_5_TopPole extends LinearOpMode {

    private PurePursuitRobotMovement6_Turn_MultiThread_V2 myPurePursuitRobotMovement6_Turn_MultiThread;
    boolean debugFlag = true;
    double armDelay = 0.0;
    double shippingHubPark = 0;
    int cbValue = 0;
    int path = 2;
    double ParkDistanceX = 0;
    double ParkDistanceY = 0;
    int purePursuitPath = 1;
    int sleeveColor;
    SleeveColorDetectorDoubleWebcam sleeveColorDetector;
    public volatile ArmShoulderPositions shoulderPositions = ArmShoulderPositions.INTAKE;
    public volatile FingerPositions fingerPositions = FingerPositions.GRAB;
    String color = "Not Found";
    ElapsedTime OpmodeTimer = new ElapsedTime();
    double timeCheck = 0;
    int timeCheckCount = 0;
    private int debugIncrement = 0;
    @Override
    public void runOpMode() {

        //Create a NerdBOT object
        myPurePursuitRobotMovement6_Turn_MultiThread = new PurePursuitRobotMovement6_Turn_MultiThread_V2(this);
        myPurePursuitRobotMovement6_Turn_MultiThread.setDebug(debugFlag);

        //Initialize Hardware
        myPurePursuitRobotMovement6_Turn_MultiThread.initializeHardware();


        telemetry.addData("NerdBOT", "Initialized");
        telemetry.update();

        sleeveColorDetector = new SleeveColorDetectorDoubleWebcam(this,"RIGHT");
        sleeveColorDetector.InitSleeveColorDetectorDoubleWebcam();
        timeCheckCount = 0;

        waitForStart();

        OpmodeTimer.reset();

        sleeveColor = sleeveColorDetector.getAnalysis();
        telemetry.addData("Analysis", sleeveColorDetector.getAnalysis());
        telemetry.update();
//        sleeveColorDetector.closeCameraDevice();
        myPurePursuitRobotMovement6_Turn_MultiThread.startOdometryThread();


        myPurePursuitRobotMovement6_Turn_MultiThread.printI();

        myPurePursuitRobotMovement6_Turn_MultiThread.resetITerm();

        myPurePursuitRobotMovement6_Turn_MultiThread.printI();

        myPurePursuitRobotMovement6_Turn_MultiThread.resetTimers();

        cbValue = sleeveColorDetector.getAnalysis();
        if(cbValue <=100) {
            color = "YELLOW";
            path = 2;
        }
        else if(cbValue >=145) {
            color = "BLUE";
            path = 1;
        }
        else {
            color = "RED";
            path = 3;
        }

//        if (path == 1) {
//        telemetry.addData("path", "1");
//        telemetry.update();
        if (purePursuitPath == 1) {
            telemetry.addData("purePursuitPath", "1");
            //Dropoff Cone One
            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 0, 0.9, 0.3, 12, 0, 90));
            allPoints.add(new CurvePoint(3, 26, 0.9, 0.3, 12, 0, 90));
            allPoints.add(new CurvePoint(3, 58, 0.9, 0.3, 12, 0, 130));
            allPoints.add(new CurvePoint(-1, 56, 0.7, 0.3, 12, 0, 130));
            allPoints.add(new CurvePoint(-1, 80, 0.7, 0.3, 12, 0, 130));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, 130, 1, ArmShoulderPositions.HOME, ArmShoulderPositions.LEVEL3, FingerPositions.GRAB, FingerPositions.GRAB, 0, 0, "none", 0);
            myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.LEVEL3, 250, FingerPositions.INTAKE_READY);
            myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.LEVEL3, 0, FingerPositions.INTAKE_READY);

            //Pickup Cone Two
            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-0.5, 59, 0.95, 0.3, 12, 0, 130));
            allPoints.add(new CurvePoint(5, 54, 0.95, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(10, 53.5, 0.95, 0.5, 12, 0, 0));
            allPoints.add(new CurvePoint(24, 52.5, 0.95, 0.5, 12, 0, 0));
            allPoints.add(new CurvePoint(40, 52.5, 0.95, 0.5, 12, 0, 0));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, 0, 1, ArmShoulderPositions.S3, ArmShoulderPositions.S4, FingerPositions.INTAKE_READY, FingerPositions.INTAKE_READY, 0, 0, "none", 0);
            myPurePursuitRobotMovement6_Turn_MultiThread.setFingerPositions(FingerPositions.GRAB, 0.8);
            myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.S4, -200, FingerPositions.GRAB);

            //Dropoff Cone Two
            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(28, 53, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(20, 51, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(5, 51, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(6, 58, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(-0.5, 54.5, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(-0.5, 80, 0.8, 0.3, 12, 0, 0));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, 145, 1, ArmShoulderPositions.HOME, ArmShoulderPositions.LEVEL3, FingerPositions.GRAB, FingerPositions.GRAB, 0, 0, "none", 0);
            myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.LEVEL3, 250, FingerPositions.INTAKE_READY);
            myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.LEVEL3, 0, FingerPositions.INTAKE_READY);

            //Pickup Cone Three
            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-0.5, 59, 0.9, 0.3, 12, 0, 130));
            allPoints.add(new CurvePoint(5, 54, 0.9, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(10, 50, 0.9, 0.5, 12, 0, 0));
            allPoints.add(new CurvePoint(24, 51.5, 0.9, 0.5, 12, 0, 0));
            allPoints.add(new CurvePoint(40, 51.5, 0.9, 0.5, 12, 0, 0));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, 0, 1, ArmShoulderPositions.S3, ArmShoulderPositions.S3, FingerPositions.INTAKE_READY, FingerPositions.INTAKE_READY, 0, 0, "none", 0);
            myPurePursuitRobotMovement6_Turn_MultiThread.setFingerPositions(FingerPositions.GRAB, 0.8);
            myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.S3, -200, FingerPositions.GRAB);

            //Dropoff Cone Three
            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(28, 53, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(20, 51, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(5, 51, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(6, 58, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(-1.5, 53, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(-1.5, 80, 0.8, 0.3, 12, 0, 0));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, 145, 1, ArmShoulderPositions.HOME, ArmShoulderPositions.LEVEL3, FingerPositions.GRAB, FingerPositions.GRAB, 0, 0, "none", 0);
            myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.LEVEL3, 250, FingerPositions.INTAKE_READY);
            myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.LEVEL3, 0, FingerPositions.INTAKE_READY);

            //Pickup Cone Four
            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-0.5, 59, 0.9, 0.3, 12, 0, 130));
            allPoints.add(new CurvePoint(5, 54, 0.9, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(10, 50, 0.9, 0.5, 12, 0, 0));
            allPoints.add(new CurvePoint(24, 51.5, 0.9, 0.5, 12, 0, 0));
            allPoints.add(new CurvePoint(40, 51.5,0.9, 0.5, 12, 0, 0));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, 0, 1, ArmShoulderPositions.S2, ArmShoulderPositions.S2, FingerPositions.INTAKE_READY, FingerPositions.INTAKE_READY, 0, 0, "none", 0);
            myPurePursuitRobotMovement6_Turn_MultiThread.setFingerPositions(FingerPositions.GRAB, 0.8);
            myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.S2, -200, FingerPositions.GRAB);

            //Dropoff Cone Four
            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(28, 53, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(20, 51, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(5, 51, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(6, 58, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(-1.5, 52, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(-1.5, 80, 0.8, 0.3, 12, 0, 0));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, 145, 1, ArmShoulderPositions.HOME, ArmShoulderPositions.LEVEL3, FingerPositions.GRAB, FingerPositions.GRAB, 0, 0, "none", 0);
            myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.LEVEL3, 250, FingerPositions.INTAKE_READY);
            myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.LEVEL3, 0, FingerPositions.INTAKE_READY);

            //Pickup Cone Five
            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-0.5, 59, 0.9, 0.3, 12, 0, 130));
            allPoints.add(new CurvePoint(5, 54, 0.9, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(10, 50, 0.9, 0.5, 12, 0, 0));
            allPoints.add(new CurvePoint(24, 51.5, 0.9, 0.5, 12, 0, 0));
            allPoints.add(new CurvePoint(40, 51.5, 0.9, 0.5, 12, 0, 0));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, 0, 1, ArmShoulderPositions.S1, ArmShoulderPositions.S1, FingerPositions.INTAKE_READY, FingerPositions.INTAKE_READY, 0, 0, "none", 0);
            myPurePursuitRobotMovement6_Turn_MultiThread.setFingerPositions(FingerPositions.GRAB, 0.8);
            myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.S1, -200, FingerPositions.GRAB);

        }

        telemetry.addData("path %s",path);
        telemetry.update();
        if(path == 2){
            //Dropoff Cone Five
            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(28, 53, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(20, 51, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(5, 51, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(6, 56, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(-1, 52, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(-1, 80, 0.8, 0.3, 12, 0, 0));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, 145, 1, ArmShoulderPositions.HOME, ArmShoulderPositions.LEVEL3, FingerPositions.GRAB, FingerPositions.GRAB, 0, 0, "none", 0);
            myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.LEVEL3, 250, FingerPositions.INTAKE_READY);
            myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.LEVEL3, 0, FingerPositions.INTAKE_READY);

            //Park
            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-2, 53.5, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(0, 53.5, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(1, 51, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(30, 51, 0.8, 0.3, 12, 0, 0));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, -2, 1, ArmShoulderPositions.INTAKE, ArmShoulderPositions.INTAKE, FingerPositions.INTAKE_READY, FingerPositions.INTAKE_READY, 0, 0, "none", 0);

        }

        if(path == 3){
            //Dropoff Cone Five
            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(28, 53, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(20, 51, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(5, 51, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(6, 56, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(-1, 52, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(-1, 80, 0.8, 0.3, 12, 0, 0));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, 145, 1, ArmShoulderPositions.HOME, ArmShoulderPositions.LEVEL3, FingerPositions.GRAB, FingerPositions.GRAB, 0, 0, "none", 0);
            myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.LEVEL3, 250, FingerPositions.INTAKE_READY);
            myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.LEVEL3, 0, FingerPositions.INTAKE_READY);

            //Park
            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-0.5, 59, 0.9, 0.3, 12, 0, 130));
            allPoints.add(new CurvePoint(5, 54, 0.9, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(10, 50, 0.9, 0.5, 12, 0, 0));
            allPoints.add(new CurvePoint(22, 51.5, 0.9, 0.5, 12, 0, 0));
            allPoints.add(new CurvePoint(40, 51.5, 0.9, 0.5, 12, 0, 0));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, -2, 1, ArmShoulderPositions.INTAKE, ArmShoulderPositions.INTAKE, FingerPositions.INTAKE_READY, FingerPositions.INTAKE_READY, 0, 0, "none", 0);
        }
        if(path == 1){
            //Dropoff Cone Five
            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(28, 53, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(20, 51, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(5, 51, 0.8, 0.3, 12, 0, 0));
            allPoints.add(new CurvePoint(6, 51, 0.8, 0.3, 12, 0, 45));
            allPoints.add(new CurvePoint(-11, 49, 0.8, 0.3, 12, 0, 60));
            allPoints.add(new CurvePoint(-35, 80, 0.8, 0.3, 12, 0, 80));

            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, 90, 1, ArmShoulderPositions.HOME, ArmShoulderPositions.LEVEL3, FingerPositions.GRAB, FingerPositions.GRAB, 0, 0, "none", 0);
            myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.LEVEL3, 250, FingerPositions.INTAKE_READY);
            myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.LEVEL3, 0, FingerPositions.INTAKE_READY);

            //Park
            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-2.5, 48, 0.8, 0.3, 12, 0, 90));
            allPoints.add(new CurvePoint(-21,48, 0.8, 0.3, 12, 0, 90));
            allPoints.add(new CurvePoint(-30, 52, 0.8, 0.3, 12, 0,0 ));
            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, -10 , 1, ArmShoulderPositions.INTAKE, ArmShoulderPositions.INTAKE, FingerPositions.INTAKE_READY, FingerPositions.INTAKE_READY, 0, 0, "none", 0);

        }

        myPurePursuitRobotMovement6_Turn_MultiThread.stopOdometryThread();
    }
}