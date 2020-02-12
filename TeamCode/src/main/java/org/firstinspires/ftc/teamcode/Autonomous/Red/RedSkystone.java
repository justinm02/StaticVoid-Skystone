package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;
import org.firstinspires.ftc.teamcode.Autonomous.Spline.Waypoint;


@Autonomous(name = "RedSkystone", group = "Autonomous")
public class RedSkystone extends Auto {

    private double xSkystone[], ySkystone[], xSkystone2[], ySkystone2[], movePlatformX[];
    private double xPlatform[], yPlatform[], xCrossBridge[], yCrossBridge[], movePlatformY[];

    private SKYSTONE_POSITION position;

    public void runOpMode() {
        initialize();
        waitForStart();

        try {
            position = determineSkystonePosition("red");

            movePlatformX = new double[]{0, 24, 24};
            movePlatformY = new double[]{0, 0, -12};

//            double current = runtime.time();
//            while (runtime.time() - current < .5) {
//                telemetry.addData("position", position);
//                telemetry.update();
//            }

            bringAlignerDown();
            releasePlatform();

            if (position.equals(SKYSTONE_POSITION.MIDDLE)) {
                goMiddle();
            }
            else if (position.equals(SKYSTONE_POSITION.LEFT)) {
                goLeft();
            }
            else {
                goRight();
            }

        } catch (InterruptedException e) {
        }
    }

    public void goLeft() throws InterruptedException {
        newMove(new Waypoint(0,0), new Waypoint(1, -27), 0, .3, .55, 0, true);

        xSkystone = new double[]{0, -5, -8, -30};
        ySkystone = new double[]{0, 0, 2, 2};

        xPlatform = new double[]{0, -15, -15, -43};
        yPlatform = new double[]{5, 5, 0, 0};

        xCrossBridge = new double[]{-43, -15, -15, 0};
        yCrossBridge = new double[]{-3, -3, 0, 0};

        xSkystone2 = new double[]{0, -5, -8, -46};
        ySkystone2 = new double[]{0, 0, 3, 3};

        grabBlock();

        //move(0, 2.5, 91, .5, .35, .35, "strafe", true);

        splineMove(xSkystone, ySkystone, -.8, -.35, -.6, 0, false);
        splineMove(xPlatform, yPlatform, -.6, -.6, -.4, 0, true);

        placeStone();
//
//        move(0, 1.5, 90, .5, .5, .3, "strafe", true);
//        grip();
//
//        splineMove(xCrossBridge, yCrossBridge,.8, .5, .65, 0, false);
//        bringAlignerDown();
//        releaseBlock();
//
//        move(0, 43, 0, .65, .5, .5, "straight", false);
//        moveByTime(0.5, 0.5, 0);
//
//        move(0, 5, -84, .5, .5, .3, "strafe", true);
//
//        grabBlock();
//        pause(.25);
//
//        move(0, 2, 90, .5, .5, .5, "strafe", true);
//        //move(0, 60, 152, .8, .5, .5, "strafe", false);
//
//        //move(0, 35, -145, .5, .5, .5, "strafe", true);
//
//        splineMove(xSkystone2, ySkystone2, -.8, -.4, -.65, 0, false);
//        splineMove(xPlatform, yPlatform, -.65, -.65, -.5, 0, false);
//        move(0, 19, 180, .45, .45, .45, "straight", true);
//        pause(.25);
//        move(0, 1, -87, .5, .5, 0, "strafe", true);
//
//        placeStone();
//        pause(.25);
//
//        getPlatform();
//
//        park();
    }

    public void goMiddle() throws InterruptedException {
        newMove(new Waypoint(0,0), new Waypoint(-7, -27.5), 0, .3, .4, 0, true);
        xSkystone = new double[]{0, -5, -8, -26};
        ySkystone = new double[]{0, 0, 2, 2};

        xPlatform = new double[]{0, -15, -15, -40};
        yPlatform = new double[]{5, 5, 0, -0.5};

        xCrossBridge = new double[]{-35, -15, 10, 10};
        yCrossBridge = new double[]{-3, -3, 2, 2};

        xSkystone2 = new double[]{0, -5, -8, -44};
        ySkystone2 = new double[]{0, 0, 2.5, 2.5};

        grabBlock();
        pause(.25);

        //move(0, 2, 90, .3, .35, 0, "strafe", true);

        splineMove(xSkystone, ySkystone, -.8, -.35, -.6, 0, false);
        splineMove(xPlatform, yPlatform, -.6, -.6, -.4, 0, true);

        placeStone();
//
//        move(0, 0.5, 90, .3, .3, 0, "strafe", true);
//        grip();
//
//        splineMove(xCrossBridge, yCrossBridge, .8, .35, .65, 0, false);
//        bringAlignerDown();
//        releaseBlock();
//
//        //move(0, 58, -25, .65, .5, .5, "strafe", true);
//        move(0, 41.5, 0, .65, .5, .5, "straight", true);
//
////        move(0, 2, -87, .3, .3, 0, "strafe", true);
//        move(0, 9, -87, .3, .3, 0, "strafe", true);
//
//        grabBlock();
//        pause(.25);
//
//        move(0, 2, 90, .5, .5, .5, "strafe", true);
//        //move(0, 60, 152, .8, .5, .5, "strafe", false);
//
//        //move(0, 35, -145, .5, .5, .5, "strafe", true);
//
//        splineMove(xSkystone2, ySkystone2, -.6, -.35, -.6, 0, false);
//        splineMove(xPlatform, yPlatform, -.6, -.6, -.45, 0, false);
//        move(0, 15, 180, .45, .45, .3, "straight", true);
//        move(0, 4, -90, .5, .5, 0, "strafe", true);
//
//        placeStone();
//        pause(.25);
//
//        getPlatform();
//
//        park();
    }

    public void goRight() throws InterruptedException {
        newMove(new Waypoint(0,0), new Waypoint(-15, -27), 0, .2, .3, 0, true);
//        xSkystone = new double[]{0, -5, 0, -25};
//        ySkystone = new double[]{0, 2, 2, 2};

        xSkystone = new double[]{0, -5, -8, -30};
        ySkystone = new double[]{0, 0, 2, 2};

        xPlatform = new double[]{0, -15, -15, -30};
        yPlatform = new double[]{3, 3, 0, -0.5};

        xCrossBridge = new double[]{-27, -15, -15, 15};
        yCrossBridge = new double[]{-3, -3, 1, 1};

        xSkystone2 = new double[]{0, -5, 0, -44};
        ySkystone2 = new double[]{0, 2, -1, -1};

        grabBlock();

        //newMove(new Waypoint(0, 0), new Waypoint(0, -3), 0, .2, .3, 0, true);
        //newMove(new Waypoint(-15, -24), new Waypoint(30, -24), 0, .2, .6, 0, true);

        //move(0, 44, 180, .8, .35, .35, "straight", false);
//
        splineMove(xSkystone, ySkystone, -.8, -.35, -.6, 0, false);
        splineMove(xPlatform, yPlatform, -.6, -.6, -.4, 0, true);

        placeStone();
//
//        move(0, 1, 90, .3, .3, 0, "strafe", true);
//        grip();
//
//        splineMove(xCrossBridge, yCrossBridge, .8, .4, .6, 0, false);
//        releaseBlock();
//        bringAlignerDown();
//
//        move(0, 39.5, 0, .6, .6, .35, "straight", true);
//        pause(.25);
//
//        move(0, 5, -90, .5, .5, .35, "strafe", true);
//
//        grabBlock();
//        pause(.25);
//
//        move(0, 4, 90, .3, .3, 0, "strafe", true);
//        //move(0, 60, 152, .8, .5, .5, "strafe", false);
//
//        //move(0, 35, -145, .5, .5, .5, "strafe", true);
//
//        splineMove(xSkystone2, ySkystone2, -.6, -.35, -.6, 0, false);
//        splineMove(xPlatform, yPlatform, -.6, -.6, -.45, 0, false);
//        move(0, 6, 180, .45, .45, .3, "straight", true);
//        pause(.25);
//        move(0, 3.5, -90, .3, .3, .3, "strafe", true);
//
//        placeStone();
//        pause(.25);
//
//        getPlatform();
//
//        park();
    }

    public void grabBlock() throws InterruptedException {
        gripBlock();
        pause(0.25);
        bringAlignerUp();
    }

    public void placeStone() throws InterruptedException {
        bringAlignerDown();
        pause(.25);
        releaseBlock();
        pause(.1);
        bringAlignerUp();
    }

    public void getPlatform() throws InterruptedException {
        PIDTurn(90, .75);
        move(90, 3, -90, .3, .3, .3, "straight", true);

        gripPlatform();
        pause(1.25);

        splineMove(movePlatformX, movePlatformY, .5, .3, 0, Math.PI/2, true);

        releasePlatform();
        pause(.5);
    }

    public void park() throws InterruptedException {
        grip();
        move(0, 11, -77, 0.95, 0.65, 0.7, "strafe", true);
        move(0, 20, 0, 0.95, .65, .7, "straight", true);
    }
}