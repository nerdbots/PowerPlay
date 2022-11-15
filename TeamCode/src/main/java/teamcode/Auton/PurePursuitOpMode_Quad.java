package teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

import teamcode.RobotUtilities.ArmShoulderPositions.CurvePoint;

//import Functions.CurvePoint;
//@Disabled
@Autonomous(name="PurePursuitOpMode_Quad", group="Auton")

public class PurePursuitOpMode_Quad extends LinearOpMode {

    private PurePursuitRobotMovement6_Quad2 myPurePursuitRobotMovement6_Quad2;
//    private wobble_Pickup mywobble_Pickup;

    boolean debugFlag = true;
    int robotPath = 4;

    @Override
    public void runOpMode() {
        //Create a NerdBOT object
        myPurePursuitRobotMovement6_Quad2 = new PurePursuitRobotMovement6_Quad2(this);
//        mywobble_Pickup = new wobble_Pickup (this);

        myPurePursuitRobotMovement6_Quad2.setDebug(debugFlag);

        //Initialize Hardware
        myPurePursuitRobotMovement6_Quad2.initializeHardware();
//        mywobble_Pickup.wobbleInit();
        //Initialize the PID Calculators

//        telemetry.addData("Init", "Completed");
//        telemetry.update();

        telemetry.addData("Okay press play", "now");
        telemetry.update();

        waitForStart();

        myPurePursuitRobotMovement6_Quad2.printI();

        myPurePursuitRobotMovement6_Quad2.resetITerm();

        myPurePursuitRobotMovement6_Quad2.printI();

        myPurePursuitRobotMovement6_Quad2.resetTimers();

//        myPurePursuitRobotMovement6.goToPosition(0, 100, 1.0, 90, 0.3, 30);
//        myPurePursuitRobotMovement5.goToPosition(0,0, 1.0, 270, 0.2);

        robotPath = 4;

 
        if (robotPath == 4) {

            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 0, 1, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(0, 80, 1, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(60, 80, 1, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(60, 20, 1, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(60, -40, 1, 0.4, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 30, 90, 5);


            //Steps
//            ArrayList<CurvePoint> allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(0, 0, 0.4, 0.4, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(0, 10, 0.4, 0.4, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(0, 50, 0.4, 0.4, 25, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 7, 60, 5);
//
//            allPoints.add(new CurvePoint(0, 10, 0.4, 0.4, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(0, 20, 0.4, 0.4, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(0, 60, 0.4, 0.4, 25, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 7, 120, 5);
//
//            allPoints.add(new CurvePoint(0, 20, 0.4, 0.4, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(0, 30, 0.4, 0.4, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(0, 70, 0.4, 0.4, 25, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 7, 60, 5);
//
//            allPoints.add(new CurvePoint(0, 30, 0.4, 0.4, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(0, 40, 0.4, 0.4, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(0, 80, 0.4, 0.4, 25, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 7, 120, 5);

           //  Small Spiral
//            ArrayList<CurvePoint> allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(0, 0, 0.6, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(24, 12, 1.0, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(48, 26, 1.0, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(65, 55, 1.0, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(67, 72, 1.0, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(53, 84, 1.0, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(38, 90, 1.0, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(-4, 90, 1.0, 0.3, 25, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 5, 90, 10);
//
//            allPoints.add(new CurvePoint(38, 90, 0.6, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(-10, 90, 1.0, 0.3, 20, 180, 0.3));
//            allPoints.add(new CurvePoint(-10, 45, 1.0, 0.3, 20, 180, 0.3));
//            allPoints.add(new CurvePoint(24, 45, 1.0, 0.3, 20, 180, 0.3));
//            allPoints.add(new CurvePoint(72, 45, 1.0, 0.3, 20, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 26, 90, 5);
            //Big Spiral to the center
//            ArrayList<CurvePoint> allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(0, 0, 0.6, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(30, 15, 1.0, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(60, 33, 1.0, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(84, 69, 1.0, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(84, 90, 1.0, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(66, 105, 1.0, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(48, 111, 1.0, 0.3, 25, 180, 0.3));
//            allPoints.add(new CurvePoint(-4, 123, 1.0, 0.3, 25, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 5, 90, 10);
//
//            allPoints.add(new CurvePoint(48, 111, 0.6, 0.3, 10, 180, 0.3));
//            allPoints.add(new CurvePoint(18, 108, 1.0, 0.3, 20, 180, 0.3));
//            allPoints.add(new CurvePoint(0, 93, 1.0, 0.3, 20, 180, 0.3));
//            allPoints.add(new CurvePoint(-10, 70, 1.0, 0.3, 20, 180, 0.3));
//            allPoints.add(new CurvePoint(-10, 40, 1.0, 0.3, 20, 180, 0.3));
//            allPoints.add(new CurvePoint(24, 40, 1.0, 0.3, 20, 180, 0.3));
//            allPoints.add(new CurvePoint(36, 48, 1.0, 0.3, 20, 180, 0.3));
//            allPoints.add(new CurvePoint(72, 0, 1.0, 0.3, 20, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 4, 90, 5);
//
////            allPoints.add(new CurvePoint(24, 45, 1.0, 0.3, 20, 180, 0.3));
//            allPoints.add(new CurvePoint(36, 48, 0.6, 0.3, 10, 180, 0.3));
//            allPoints.add(new CurvePoint(45, 52, 1.0, 0.3, 20, 180, 0.3));
//            allPoints.add(new CurvePoint(54, 63, 1.0, 0.3, 20, 180, 0.3));
//            allPoints.add(new CurvePoint(51, 75, 1.0, 0.3, 20, 180, 0.3));
//            allPoints.add(new CurvePoint(39, 78, 1.0, 0.3, 20, 180, 0.3));
//            allPoints.add(new CurvePoint(0, 78, 1.0, 0.3, 20, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 15, 90, 5);

            //First Path to the Square 4
//            ArrayList<CurvePoint> allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(0, 0, 1.0, 0.3, 40, 0, 0.3));
//            allPoints.add(new CurvePoint(-8, 40, 1.0, 0.3, 40, 180, 0.3));
//            allPoints.add(new CurvePoint(-12, 116, 1.0, 0.3, 40, 180, 0.3));
//            allPoints.add(new CurvePoint(-12, 171, 1.0, 0.3, 40, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 40, 90, 3);

//    //        Lower arm and release wobble goal
//    //        mywobble_Pickup.beginningDown();
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(-12, 116, 1.0, 0.4, 35, 0, 0.3));
//            allPoints.add(new CurvePoint(14, 100, 1.0, 0.4, 35, 180, 0.3));
//            allPoints.add(new CurvePoint(21, 28, 1.0, 0.4, 35, 180, 0.3));
//            allPoints.add(new CurvePoint(21, -44, 1.0, 0.3, 35, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 35, 225, 1);
//
//    //        Grab wobble goal and raise arm
//    //        mywobble_Pickup.pickupWobble();
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(24, 14, 1.0, 0.4, 40, 0, 0.3));
//            allPoints.add(new CurvePoint(24, 60, 1.0, 0.4, 40, 180, 0.3));
//            allPoints.add(new CurvePoint(-12, 110, 1.0, 0.4, 40, 180, 0.3));
//            allPoints.add(new CurvePoint(-16, 160, 1.0, 0.3, 40, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 35, 90, 3);
//
//    //        Lower arm and release wobble goal
//    //        mywobble_Pickup.setDownWobble();
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(-8, 110, 1.0, 0.4, 40, 0, 0.3));
//            allPoints.add(new CurvePoint(4, 60, 1.0, 0.4, 40, 180, 0.3));
//            allPoints.add(new CurvePoint(4, 10, 1.0, 0.4, 40, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 30, 90, 2);
//
//    //        sleep(1000);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(4, 60, 0.6, 0.4, 30, 0, 0.3));
//            allPoints.add(new CurvePoint(2, 48, 0.6, 0.4, 30, 180, 0.3));
//            allPoints.add(new CurvePoint(2, 0, 0.6, 0.4, 30, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 10, 90, 3);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(2, 48, 0.6, 0.4, 30, 0, 0.3));
//            allPoints.add(new CurvePoint(4, 60, 0.6, 0.4, 30, 180, 0.3));
//            allPoints.add(new CurvePoint(4, 110, 0.6, 0.4, 30, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 10, 90, 2);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(4, 60, 0.6, 0.4, 30, 0, 0.3));
//            allPoints.add(new CurvePoint(2, 48, 0.6, 0.4, 30, 180, 0.3));
//            allPoints.add(new CurvePoint(2, 0, 0.6, 0.4, 30, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 10, 90, 3);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(2, 48, 0.6, 0.4, 30, 0, 0.3));
//            allPoints.add(new CurvePoint(4, 60, 0.6, 0.4, 30, 180, 0.3));
//            allPoints.add(new CurvePoint(4, 110, 0.6, 0.4, 30, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 10, 90, 2);
//
//            allPoints = new ArrayList<>();
//            allPoints.add(new CurvePoint(4, 60, 0.6, 0.4, 30, 0, 0.3));
//            allPoints.add(new CurvePoint(2, 70, 0.6, 0.4, 30, 180, 0.3));
//            allPoints.add(new CurvePoint(2, 130, 0.6, 0.4, 30, 180, 0.3));
//
//            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 7, 90, 4);


        }else if (robotPath == 1) {


            //Second Path to the Square 1
            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 0, 1.0, 0.3, 35, 0, 0.3));
            allPoints.add(new CurvePoint(-15, 40, 1.0, 0.3, 35, 180, 0.3));
            allPoints.add(new CurvePoint(6, 88, 1.0, 0.3, 35, 180, 0.3));
            allPoints.add(new CurvePoint(27, 148, 1.0, 0.3, 35, 180, 0.3));

            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 35, 90, 3);

    //        sleep(1000);
    //        mywobble_Pickup.beginningDown();

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(6, 88, 1.0, 0.4, 35, 0, 0.3));
            allPoints.add(new CurvePoint(18, 60, 1.0, 0.4, 35, 180, 0.3));
            allPoints.add(new CurvePoint(21, 28, 1.0, 0.4, 35, 180, 0.3));
            allPoints.add(new CurvePoint(21, -44, 1.0, 0.3, 35, 180, 0.3));

            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 35, 225, 1);

    //        sleep(1000);
    //        mywobble_Pickup.pickupWobble();

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(21, 28, 1.0, 0.4, 35, 0, 0.3));
            allPoints.add(new CurvePoint(18, 60, 1.0, 0.4, 35, 180, 0.3));
            allPoints.add(new CurvePoint(12, 80, 1.0, 0.4, 35, 180, 0.3));
            allPoints.add(new CurvePoint(0, 140, 1.0, 0.3, 35, 180, 0.3));

            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 30, 90, 3);

    //        sleep(1000);
    //        mywobble_Pickup.setDownWobble();


            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(12, 86, 0.6, 0.4, 40, 0, 0.3));
            allPoints.add(new CurvePoint(5, 60, 0.6, 0.4, 40, 180, 0.3));
            allPoints.add(new CurvePoint(4, 10, 0.6, 0.4, 40, 180, 0.3));

            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 10, 90, 3);

    //        sleep(1000);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(5, 60, 0.6, 0.4, 25, 0, 0.3));
            allPoints.add(new CurvePoint(3, 48, 0.6, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(1, 0, 0.6, 0.4, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 10, 90, 3);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(3, 48, 0.6, 0.4, 40, 0, 0.3));
            allPoints.add(new CurvePoint(5, 60, 0.6, 0.4, 40, 180, 0.3));
            allPoints.add(new CurvePoint(5, 110, 0.6, 0.4, 40, 180, 0.3));

            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 10, 90, 3);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(5, 60, 0.6, 0.4, 40, 0, 0.3));
            allPoints.add(new CurvePoint(5, 70, 0.6, 0.4, 40, 180, 0.3));
            allPoints.add(new CurvePoint(5, 130, 0.6, 0.4, 40, 180, 0.3));

            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 7, 90, 4);

        }else {

            //Third Path to the Square 0
            ArrayList<CurvePoint> allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(0, 0, 1.0, 0.3, 35, 0, 0.3));
            allPoints.add(new CurvePoint(-18, 66, 1.0, 0.3, 35, 180, 0.3));
            allPoints.add(new CurvePoint(-24, 140, 1.0, 0.3, 35, 180, 0.3));

            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 35, 90, 3);

//        sleep(1000);
//        mywobble_Pickup.beginningDown();

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-18, 66, 1.0, 0.4, 25, 0, 0.3));
            allPoints.add(new CurvePoint(18, 50, 1.0, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(23, 28, 1.0, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(23, -44, 1.0, 0.3, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 30, 225, 1);

//        sleep(1000);
//        mywobble_Pickup.pickupWobble();

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(22, 28, 1.0, 0.4, 35, 0, 0.3));
            allPoints.add(new CurvePoint(-12, 65, 1.0, 0.4, 35, 180, 0.3));
            allPoints.add(new CurvePoint(0, 140, 1.0, 0.3, 35, 180, 0.3));

            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 35, 90, 3);

//        sleep(1000);
//        mywobble_Pickup.setDownWobble();

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-12, 65, 0.6, 0.4, 25, 0, 0.3));
            allPoints.add(new CurvePoint(-12, 50, 0.6, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(-12, 0, 0.6, 0.4, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 10, 90, 4);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(-12, 50, 0.6, 0.4, 25, 0, 0.3));
            allPoints.add(new CurvePoint(6, 60, 0.6, 0.4, 25, 180, 0.3));
            allPoints.add(new CurvePoint(24, 100, 0.6, 0.4, 25, 180, 0.3));

            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 10, 90, 2);

            allPoints = new ArrayList<>();
            allPoints.add(new CurvePoint(6, 60, 0.6, 0.4, 35, 0, 0.3));
            allPoints.add(new CurvePoint(6, 70, 0.6, 0.4, 35, 180, 0.3));
            allPoints.add(new CurvePoint(6, 130, 0.6, 0.4, 35, 180, 0.3));

            myPurePursuitRobotMovement6_Quad2.followCurve(allPoints, 90, 7, 90, 4);

        }

    }

}

