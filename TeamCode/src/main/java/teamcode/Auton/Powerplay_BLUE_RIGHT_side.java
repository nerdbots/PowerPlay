package teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.ArmShoulderPositions;
//import org.firstinspires.ftc.teamcode.DuckDetector;
//import org.firstinspires.ftc.teamcode.FingerPositions;

import java.util.ArrayList;
import teamcode.RobotUtilities.Odometry.CurvePoint;
import teamcode.RobotUtilities.ArmShoulderPositions;
import teamcode.RobotUtilities.FingerPositions;
import teamcode.TeleOp.SleeveColorDetector;
import teamcode.TeleOp.SleeveColorDetectorDoubleWebcam;

//@Disabled
@Autonomous(name="Powerplay BLUE RIGHT side", group="Linear Opmode")

public class Powerplay_BLUE_RIGHT_side extends LinearOpMode {

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
    SleeveColorDetectorDoubleWebcam sleeveColorDetector;
    public volatile ArmShoulderPositions shoulderPositions = ArmShoulderPositions.INTAKE;
    public volatile FingerPositions fingerPositions = FingerPositions.GRAB;
    String color = "Not Found";
ElapsedTime OpmodeTimer = new ElapsedTime();
   double timeCheck = 0;
   int timeCheckCount = 0;
    @Override
    public void runOpMode() {

        //Create a NerdBOT object
        myPurePursuitRobotMovement6_Turn_MultiThread = new PurePursuitRobotMovement6_Turn_MultiThread(this);
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
        sleeveColorDetector.closeCameraDevice();
        myPurePursuitRobotMovement6_Turn_MultiThread.startOdometryThread();


        myPurePursuitRobotMovement6_Turn_MultiThread.printI();

        myPurePursuitRobotMovement6_Turn_MultiThread.resetITerm();

        myPurePursuitRobotMovement6_Turn_MultiThread.printI();

        myPurePursuitRobotMovement6_Turn_MultiThread.resetTimers();

        cbValue = sleeveColorDetector.getAnalysis();
        //Detecting purple
        if(cbValue <=108) {
            color = "ORANGE";
            path = 1;
        }
        else if(cbValue >=138) {
            color = "PURPLE";
            path = 2;
        }
        else {
            color = "GREEN";
            path = 3;
        }
        telemetry.addData("path %s",path);
        telemetry.update();
        if (path == 1) {
            telemetry.addData("path", "1");
            telemetry.update();
            if (purePursuitPath == 1) {
                telemetry.addData("purePursuitPath", "1");
                telemetry.update();
//            ArrayList<CurvePoint> allPoints = new ArrayList<>();
                ArrayList<CurvePoint> allPoints = new ArrayList<>();
                allPoints.add(new CurvePoint(0, 0, 0.5, 0.3, 25, 0, 0.3));
                allPoints.add(new CurvePoint(-27, 0, 0.5, 0.3, 23, 0, 0.8));
                allPoints.add(new CurvePoint(-22, 32.5, 0.5, 0.3, 25, 0, 0.3));
                allPoints.add(new CurvePoint(-22.5, 77.5, 0.5, 0.3, 25, 0, 0.3));
//            allPoints.add(new CurvePoint(-48, 60, 0.5, 0.3, 25, 180, 0.3));

                myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, 135, 1.5, ArmShoulderPositions.HOME, ArmShoulderPositions.LEVEL3, FingerPositions.GRAB, FingerPositions.GRAB, 0, 0, "none", 0);
                myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.LEVEL3, 400, FingerPositions.INTAKE_READY);
                sleep(500);
                allPoints.add(new CurvePoint(-22, 32.5, 0.5, 0.3, 25, 0, 0.3));
                allPoints.add(new CurvePoint(-20, 28, 0.5, 0.3, 23, 0, 0.8));
                allPoints.add(new CurvePoint(20, 15, 0.5, 0.3, 25, 0, 0.3));
//            allPoints.add(new CurvePoint(-48, 60, 0.5, 0.3, 25, 180, 0.3));

                myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, 22.5, 1.5, ArmShoulderPositions.HOME, ArmShoulderPositions.INTAKE, FingerPositions.INTAKE_READY, FingerPositions.INTAKE_READY, 0, 0, "none", 0);
            }
        }
            else if (path == 2) {
                telemetry.addData("Path", "2");
                telemetry.update();
                if (purePursuitPath == 1) {
                    telemetry.addData("purePursuitPath", "1");
                    telemetry.update();
//            ArrayList<CurvePoint> allPoints = new ArrayList<>();
                    ArrayList<CurvePoint> allPoints = new ArrayList<>();
                    allPoints.add(new CurvePoint(0, 0, 0.5, 0.3, 25, 0, 0.3));
                    allPoints.add(new CurvePoint(-27, 0, 0.5, 0.3, 23, 0, 0.8));
                    allPoints.add(new CurvePoint(-22, 32.5, 0.5, 0.3, 25, 0, 0.3));
                    allPoints.add(new CurvePoint(-22.5, 77.5, 0.5, 0.3, 25, 0, 0.3));

                    myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, 135, 1.5, ArmShoulderPositions.HOME, ArmShoulderPositions.LEVEL3, FingerPositions.GRAB, FingerPositions.GRAB, 0, 0, "none", 0);
                    myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.LEVEL3, 400, FingerPositions.INTAKE_READY);
                    sleep(500);
                    allPoints = new ArrayList<>();
                    allPoints.add(new CurvePoint(-22, 32.5, 0.5, 0.3, 25, 0, 0.3));
                    allPoints.add(new CurvePoint(-22.5, 28, 0.5, 0.3, 23, 0, 0.8));
                    allPoints.add(new CurvePoint(6, 28, 0.5, 0.3, 25, 0, 0.3));
                    allPoints.add(new CurvePoint(40, 28, 0.5, 0.3, 25, 0, 0.3));

                    myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, 0, 1.5, ArmShoulderPositions.HOME, ArmShoulderPositions.INTAKE, FingerPositions.INTAKE_READY, FingerPositions.INTAKE_READY, 0, 0, "none", 0);

                }
            }
                else if (path == 3) {
                telemetry.addData("Path", "3");
                telemetry.update();
                if (purePursuitPath == 1) {
                    telemetry.addData("purePursuitPath", "1");
                    telemetry.update();
//            ArrayList<CurvePoint> allPoints = new ArrayList<>();
                    ArrayList<CurvePoint> allPoints = new ArrayList<>();
                    allPoints.add(new CurvePoint(0, 0, 0.5, 0.3, 25, 0, 0.3));
                    allPoints.add(new CurvePoint(-27, 0, 0.5, 0.3, 23, 0, 0.8));
                    allPoints.add(new CurvePoint(-22, 32.5, 0.5, 0.3, 25, 0, 0.3));
                    allPoints.add(new CurvePoint(-22.5, 77.5, 0.5, 0.3, 25, 0, 0.3));

                    myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, 135, 1.5, ArmShoulderPositions.HOME, ArmShoulderPositions.LEVEL3, FingerPositions.GRAB, FingerPositions.GRAB, 0, 0, "none", 0);
                    myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.LEVEL3, 400, FingerPositions.INTAKE_READY);
                    sleep(500);

                    allPoints = new ArrayList<>();
                    allPoints.add(new CurvePoint(-22, 32.5, 0.5, 0.3, 25, 0, 0.3));
                    allPoints.add(new CurvePoint(-22, 28, 0.5, 0.3, 25, 0, 0.8));
                    allPoints.add(new CurvePoint(15, 30, 0.5, 0.3, 25, 0, 0.3));
                    allPoints.add(new CurvePoint(26, 30, 0.5, 0.3, 25, 0, 0.3));
                    allPoints.add(new CurvePoint(72, 28, 0.5, 0.3, 25, 0, 0.3));

                    myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, 0, 1.5, ArmShoulderPositions.HOME, ArmShoulderPositions.INTAKE, FingerPositions.INTAKE_READY, FingerPositions.INTAKE_READY, 0, 0, "none", 0);

                }
            }

                    //Park Positon 2

//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(-22.5, 32.5, 0.5, 0.3, 25, 0, 0.3));
//            allPoints.add(new CurvePoint(-22.5,28 , 0.5, 0.3, 23, 0, 0.8));
//            allPoints.add(new CurvePoint(10, 28, 0.5, 0.3, 25, 0, 0.3));
//            allPoints.add(new CurvePoint(40, 28, 0.5, 0.3, 25, 0, 0.3));
////            allPoints.add(new CurvePoint(-48, 60, 0.5, 0.3, 25, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, 180, 1.5, ArmShoulderPositions.HOME, ArmShoulderPositions.LEVEL3, FingerPositions.GRAB, FingerPositions.GRAB, 0, 0, "none", 0);

                    //ParkPostion 3
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(-22.5, 32.5, 0.5, 0.3, 25, 0, 0.3));
//            allPoints.add(new CurvePoint(-22.5, 28, 0.5, 0.3, 25, 0, 0.8));
//            allPoints.add(new CurvePoint(15, 30, 0.5, 0.3, 25, 0, 0.3));
//            allPoints.add(new CurvePoint(32, 30, 0.5, 0.3, 25, 0, 0.3));
//            allPoints.add(new CurvePoint(72, 28, 0.5, 0.3, 25, 0, 0.3));
////            allPoints.add(new CurvePoint(-48, 60, 0.5, 0.3, 25, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, 180, 1.5, ArmShoulderPositions.HOME, ArmShoulderPositions.LEVEL3, FingerPositions.GRAB, FingerPositions.GRAB, 0, 0, "none", 0);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(-22.5, 32.5, 0.5, 0.3, 15, 0, 0.3));
//            allPoints.add(new CurvePoint(-32, 75, 0.5, 0.3, 15, 0, 0.3));
//            allPoints.add(new CurvePoint(-12, 35, 0.5, 0.3, 15, 0, 0.3));
//            allPoints.add(new CurvePoint(0, 56, 0.5, 0.3, 15, 0, 0.3));
//            allPoints.add(new CurvePoint(16, 18, 0.5, 0.25, 15, 0, 0.3));
//            allPoints.add(new CurvePoint(22, 54, 0.5, 0.25, 15, 0, 0.3));
//            allPoints.add(new CurvePoint(62, 54, 0.5, 0.25, 15, 0, 0.3));
////            allPoints.add(new CurvePoint(-48, 60, 0.5,  , 25, 180, 0.3));
//            myPurePursuitRobotMovement6_Turn_MultiThread.followCurveArm(allPoints, 0, 15, 360, 3, ArmShoulderPositions.LEVEL3,ArmShoulderPositions.PICK_UP_CONE, FingerPositions.INTAKE_READY, FingerPositions.GRAB,0,0, "none", 0);
//            myPurePursuitRobotMovement6_Turn_MultiThread.moveArmsOnly(ArmShoulderPositions.PICK_UP_CONE, -400,FingerPositions.GRAB);

//
//            sleep(1500);,
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

//            myPurePursuitRobotMovement6_Turn_MultiThread.followCurve(allPoints, 0, 15, -135, 3);
                    myPurePursuitRobotMovement6_Turn_MultiThread.stopOdometryThread();
//                    if (path == 2) {
//                        ParkDistanceX = 0;
//                        ParkDistanceY = 0;
//
//                    } else if (path == 1) {
//                        ParkDistanceY = 32;
//                        ParkDistanceX = 32;
//
//                    } else if (path == 3) {
//                        ParkDistanceX = 0;
//                        ParkDistanceY = 0;
//                    }





    }
}