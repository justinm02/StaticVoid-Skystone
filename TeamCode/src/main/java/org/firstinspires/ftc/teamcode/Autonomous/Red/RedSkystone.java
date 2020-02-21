package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;
import org.firstinspires.ftc.teamcode.Autonomous.Spline.Waypoint;


@Autonomous(name = "RedSkystone", group = "Autonomous")
public class RedSkystone extends Auto {

    private double xSkystone[], ySkystone[], xSkystone2[], ySkystone2[], xCrossBridge2[], xPlatform2[], xCrossBridge3[], xStone3[], xCrossBridge4[], xPlatform3[], movePlatformX[];
    private double xPlatform[], yPlatform[], xCrossBridge[], yCrossBridge[], yCrossBridge2[], yPlatform2[], yCrossBridge3[], yStone3[], yCrossBridge4[], yPlatform3[], movePlatformY[];

    private SKYSTONE_POSITION position;

    public void runOpMode() {
        initialize();
        waitForStart();

        try {
            position = determineSkystonePosition("red");

            movePlatformX = new double[]{0, 24, 24};
            movePlatformY = new double[]{0, 0, -11};

//            double current = runtime.time();
//            while (runtime.time() - current < .5) {
//                telemetry.addData("position", position);
//                telemetry.update();
//            }

            bringAlignerDown("right");
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
        newMove(new Waypoint(0,0), new Waypoint(2, -27), 0, .35, .75, 0, true, 3, 2);

        xSkystone = new double[]{0, -5, -8, -31};
        ySkystone = new double[]{0, 0, 2.7, 2.7};

        xPlatform = new double[]{0, -15, -15, -40};
        yPlatform = new double[]{5, 5, 1, 1};

        xCrossBridge = new double[]{-35, -20, -10, 7};
        yCrossBridge = new double[]{-2.5, -2.5, 2.5, 2.5};

        xSkystone2 = new double[]{0, 8, 44, 49.5};
        ySkystone2 = new double[]{0, 0, -1.25, -1.25};

        xCrossBridge2 = new double[]{0, -5, -12, -56};
        yCrossBridge2 = new double[]{0, 0, 2.5, 2.5};

        xPlatform2 = new double[]{0, -15, -30, -58};
        yPlatform2 = new double[]{5, 5, -1.5, -1.5};

        xCrossBridge3 = new double[]{0, 15, 30, 55};
        yCrossBridge3 = new double[]{0, 0, 5.25, 5.25};

        xStone3 = new double[]{0, 0, 13, 30};
        yStone3 = new double[]{0, 0, -1, -1};

        xCrossBridge4 = new double[]{0, -5, -8, -24};
        yCrossBridge4 = new double[]{0, 0, 3.2, 3.2};

        xPlatform3 = new double[]{0, -15, -15, -47.75};
        yPlatform3 = new double[]{3.8, 3.8, 0, 0};

        grabBlock();

        //move(0, 2.5, 91, .5, .35, .35, "strafe", true);

        splineMove(xSkystone, ySkystone, -.8, -.4, -.8, 0, false);
        splineMove(xPlatform, yPlatform, -.8, -.8, -.35, 0, true);

        placeStone();
        pause(.3);
        grip();

        splineMove(xCrossBridge, yCrossBridge, .8, .4, .8, 0, false);
        bringAlignerDown("right");
        splineMove(xSkystone2, ySkystone2, .8, .8, .4, 0, false);
        moveByTime(0.8, 0.4, 0);
        move(0, 1, -88, .35, .35, .35, "strafe", true);
        strafeToBlock("right");
        grabBlock();

        splineMove(xCrossBridge2, yCrossBridge2, -.8, -.4, -.8, 0, false);
        splineMove(xPlatform2, yPlatform2, -.8, -.8, -.4, 0, true);

        placeStone();
        pause(.3);
        grip();

        splineMove(xCrossBridge3, yCrossBridge3, .8, .4, .65, 0, false);
        bringAlignerDown("right");
        splineMove(xStone3, yStone3, .65, .65, .3, 0, true);
        pause(.25);
        move(0, 0.5, -90, .35, .35, .35, "strafe", true);
        strafeToBlock("right");
        grabBlock();

        splineMove(xCrossBridge4, yCrossBridge4, -.8, -.4, -.8, 0, false);
        splineMove(xPlatform3, yPlatform3, -.8, -.8, -.4, 0, true);

        placeStone();
        pause(.3);

        getPlatform();

        grip();
        move(0, 10, -77, 1, 0.7, 0.7, "strafe", true);
        move(0, 18, 0, 1, .7, .7, "straight", true);
    }

    public void goMiddle() throws InterruptedException {
        newMove(new Waypoint(0,0), new Waypoint(-6, -28.75), 0, .3, .45, 0, true, 2, 2.9);

        xSkystone = new double[]{0, -3, -5, -32};
        ySkystone = new double[]{0, 0, 2.25, 2.25};

        xPlatform = new double[]{0, -15, -15, -33};
        yPlatform = new double[]{5, 5, 0, 0};

        xCrossBridge = new double[]{-35, -25, -10, 20};
        yCrossBridge = new double[]{-3, -3, 2.6, 2.6};

        xSkystone2 = new double[]{0, 0, 15, 33.75};
        ySkystone2 = new double[]{0, 0, -1.75, -1.75};

        xCrossBridge2 = new double[]{0, -5, -12, -46.5};
        yCrossBridge2 = new double[]{0, 0, 2.5, 2.5};

        xPlatform2 = new double[]{0, -15, -15, -59.75};
        yPlatform2 = new double[]{4.7, 4.7, 0, 0};

        xCrossBridge3 = new double[]{0, 15, 25, 60};
        yCrossBridge3 = new double[]{0, 0, 5.2, 5.2};

        xStone3 = new double[]{0, 0, 13, 30};
        yStone3 = new double[]{0, 0, 0, 0};

        xCrossBridge4 = new double[]{0, -5, -8, -30.25};
        yCrossBridge4 = new double[]{0, 0, 3, 3};

        xPlatform3 = new double[]{0, -15, -15, -52};
        yPlatform3 = new double[]{5, 5, 0, 0};

        grabBlock();
        pause(.25);

        splineMove(xSkystone, ySkystone, -.8, -.5, -.8, 0, false);
        splineMove(xPlatform, yPlatform, -.8, -.8, -.4, 0, true);

        placeStone();
        pause(.3);
        grip();

        splineMove(xCrossBridge, yCrossBridge, .8, .5, .6, 0, false);
        bringAlignerDown("right");

        splineMove(xSkystone2, ySkystone2, .6, .6, .2, 0, true);
        pause(.25);
        move(0, 1, -90, .35, .35, .35, "strafe", false);
        strafeToBlock("right");
        grabBlock();

        splineMove(xCrossBridge2, yCrossBridge2, -.8, -.35, -.6, 0, false);
        splineMove(xPlatform2, yPlatform2, -.8, -.8, -.4, 0, true);

        placeStone();
        pause(.3);
        grip();

        splineMove(xCrossBridge3, yCrossBridge3, .8, .5, .6, 0, false);

        bringAlignerDown("right");
        splineMove(xStone3, yStone3, .6, .6, .3, 0, true);
        pause(.25);
        move(0, 1, -90, .35, .35, .35, "strafe", false);
        strafeToBlock("right");
        grabBlock();

        splineMove(xCrossBridge4, yCrossBridge4, -.8, -.5, -.8, 0, false);
        splineMove(xPlatform3, yPlatform3, -.8, -.8, -.4, 0, true);

        placeStone();
        pause(.3);

        getPlatform();

        grip();
        move(0, 12, -77, 1, 1, 0.7, "strafe", true);
        move(0, 20, 0, 1, .7, .7, "straight", true);
    }

    public void goRight() throws InterruptedException {
        newMove(new Waypoint(0,0), new Waypoint(-13.75, -29), 0, .2, .3, 0, true, 2, 2.25);

        xSkystone = new double[]{0, -2, -5, -28.5};
        ySkystone = new double[]{0, 0, 1.9, 1.9};

        xPlatform = new double[]{0, -5, -24, -30};
        yPlatform = new double[]{3, 3, -4.2, -4.2};

        xCrossBridge = new double[]{-27, -15, -15, 15};
        yCrossBridge = new double[]{-3, -3, 1.7, 1.7};

        xSkystone2 = new double[]{0, 0, 30, 41};
        ySkystone2 = new double[]{0, 0, -1.9, -1.9};

        xCrossBridge2 = new double[]{0, -5, -12, -32};
        yCrossBridge2 = new double[]{0, 0, 2.35, 2.35};

        xPlatform2 = new double[]{0, -15, -15, -68};
        yPlatform2 = new double[]{5, 5, 1.25, 1.25};

        xCrossBridge3 = new double[]{0, 15, 25, 61};
        yCrossBridge3 = new double[]{0, 0, 5.2, 5.2};

        xStone3 = new double[]{0, 0, 13, 31.5};
        yStone3 = new double[]{0, 0, -1.4, -1.4};

        xCrossBridge4 = new double[]{0, -5, -8, -30.5};
        yCrossBridge4 = new double[]{0, 0, 2.7, 2.7};

        xPlatform3 = new double[]{0, -15, -15, -51};
        yPlatform3 = new double[]{5, 5, 1.5, 1.5};

        grabBlock();

        splineMove(xSkystone, ySkystone, -.8, -.35, -.6, 0, false);
        splineMove(xPlatform, yPlatform, -.8, -.8, -.4, 0, true);

        placeStone();
        pause(.3);
        grip();

        splineMove(xCrossBridge, yCrossBridge, .8, .4, .6, 0, false);
        bringAlignerDown("right");
        splineMove(xSkystone2, ySkystone2, .6, .6, .2, 0, true);
        pause(.25);
        move(0, 0.4, -90, .35, .35, .35, "strafe", false);
        strafeToBlock("right");
        grabBlock();

        splineMove(xCrossBridge2, yCrossBridge2, -.8, -.35, -.6, 0, false);
        splineMove(xPlatform2, yPlatform2, -.8, -.8, -.2, 0, true);

        placeStone();
        pause(.3);
        grip();

        splineMove(xCrossBridge3, yCrossBridge3, .8, .4, .6, 0, false);
        bringAlignerDown("right");
        splineMove(xStone3, yStone3, .6, .6, .3, 0, true);
        pause(.25);
        move(0, 0.9, -90, .35, .35, .35, "strafe", false);
        strafeToBlock("right");
        grabBlock();

        splineMove(xCrossBridge4, yCrossBridge4, -.8, -.35, -.8, 0, false);
        splineMove(xPlatform3, yPlatform3, -.8, -.8, -.3, 0, true);

        placeStone();
        pause(.3);

        getPlatform();

        grip();
        move(0, 11, -77, 1, 0.7, 0.7, "strafe", true);
        move(0, 17, 0, 1, .7, .7, "straight", true);
    }

    public void grabBlock() throws InterruptedException {
        gripBlock("right");
        pause(0.25);
        bringAlignerUp("right");
    }

    public void placeStone() throws InterruptedException {
        releaseBlock("right");
        pause(.1);
        bringAlignerUp("right");
    }

    public void getPlatform() throws InterruptedException {
        PIDTurn(90, .75);
        move(90, 4, -90, .3, .3, .3, "straight", true);

        gripPlatform();
        pause(0.75);

        splineMove(movePlatformX, movePlatformY, .4, .3, 0, Math.PI/2, true);

        releasePlatform();
        pause(.5);
    }
}