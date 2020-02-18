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
            movePlatformY = new double[]{0, 0, -12.25};

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
        newMove(new Waypoint(0,0), new Waypoint(2, -26.25), 0, .35, .75, 0, true, 3, 2);

        xSkystone = new double[]{0, -5, -8, -31};
        ySkystone = new double[]{0, 0, 2.7, 2.7};

        xPlatform = new double[]{0, -15, -15, -42};
        yPlatform = new double[]{5, 5, 0.5, 0.5};

        xCrossBridge = new double[]{-35, -20, -10, 5};
        yCrossBridge = new double[]{-3, -3, 2, 2};

        xSkystone2 = new double[]{0, 8, 44, 49.5};
        ySkystone2 = new double[]{0, 0, -2.2, -2.2};

        xCrossBridge2 = new double[]{0, -5, -12, -56};
        yCrossBridge2 = new double[]{0, 0, 2.7, 2.7};

        xPlatform2 = new double[]{0, -15, -30, -60};
        yPlatform2 = new double[]{5, 5, -1, -1};

        xCrossBridge3 = new double[]{0, 15, 30, 55};
        yCrossBridge3 = new double[]{0, 0, 5.3, 5.3};

        xStone3 = new double[]{0, 0, 13, 31.75};
        yStone3 = new double[]{0, 0, -1.2, -1.2};

        xCrossBridge4 = new double[]{0, -5, -8, -24};
        yCrossBridge4 = new double[]{0, 0, 3.25, 3.25};

        xPlatform3 = new double[]{0, -15, -15, -49};
        yPlatform3 = new double[]{5.5, 5.5, 0, 0};

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

        move(0, 1.75, -88, .3, .3, .3, "strafe", true);

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
        move(0, 1.55, -90, .5, .5, .3, "strafe", true);
        grabBlock();

        splineMove(xCrossBridge4, yCrossBridge4, -.8, -.4, -.8, 0, false);
        splineMove(xPlatform3, yPlatform3, -.8, -.8, -.4, 0, true);

        placeStone();
        pause(.3);

        getPlatform();

        grip();
        move(0, 11, -77, 1, 0.7, 0.7, "strafe", true);
        move(0, 18, 0, 1, .7, .7, "straight", true);
    }

    public void goMiddle() throws InterruptedException {
        newMove(new Waypoint(0,0), new Waypoint(-6, -28.75), 0, .3, .45, 0, true, 2, 2.9);

        xSkystone = new double[]{0, -3, -5, -32};
        ySkystone = new double[]{0, 0, 2.25, 2.25};

        xPlatform = new double[]{0, -15, -15, -34};
        yPlatform = new double[]{5.5, 5.5, 0, 0};

        xCrossBridge = new double[]{-35, -25, -10, 19};
        yCrossBridge = new double[]{-3, -3, 2.3, 2.3};

        xSkystone2 = new double[]{0, 0, 15, 38.5};
        ySkystone2 = new double[]{0, 0, -2.1, -2.1};

        xCrossBridge2 = new double[]{0, -5, -12, -48};
        yCrossBridge2 = new double[]{0, 0, 2.4, 2.4};

        xPlatform2 = new double[]{0, -15, -15, -61};
        yPlatform2 = new double[]{5, 5, 0, 0};

        xCrossBridge3 = new double[]{0, 15, 25, 60};
        yCrossBridge3 = new double[]{0, 0, 5.3, 5.3};

        xStone3 = new double[]{0, 0, 13, 33.75};
        yStone3 = new double[]{0, 0, -1, -1};

        xCrossBridge4 = new double[]{0, -5, -8, -30.25};
        yCrossBridge4 = new double[]{0, 0, 3, 3};

        xPlatform3 = new double[]{0, -15, -15, -54};
        yPlatform3 = new double[]{5, 5, 1, 1};

        grabBlock();
        pause(.25);

        splineMove(xSkystone, ySkystone, -.8, -.4, -.8, 0, false);
        splineMove(xPlatform, yPlatform, -.8, -.8, -.4, 0, true);

        placeStone();
        pause(.3);
        grip();

        splineMove(xCrossBridge, yCrossBridge, .8, .4, .6, 0, false);
        bringAlignerDown("right");

        splineMove(xSkystone2, ySkystone2, .6, .6, .2, 0, true);
        pause(.25);
        move(0, 1.4, -90, .5, .5, .3, "strafe", true);

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
        move(0, 1.7, -90, .5, .5, .3, "strafe", true);
        grabBlock();

        splineMove(xCrossBridge4, yCrossBridge4, -.8, -.35, -.8, 0, false);
        splineMove(xPlatform3, yPlatform3, -.8, -.8, -.3, 0, true);

        placeStone();
        pause(.3);

        getPlatform();

        grip();
        move(0, 12, -77, 1, 1, 0.7, "strafe", true);
        move(0, 20, 0, 1, .7, .7, "straight", true);
    }

    public void goRight() throws InterruptedException {
        newMove(new Waypoint(0,0), new Waypoint(-13.75, -28.5), 0, .2, .3, 0, true, 2, 2.85);

        xSkystone = new double[]{0, -2, -5, -28.5};
        ySkystone = new double[]{0, 0, 1.5, 1.5};

        xPlatform = new double[]{0, -5, -24, -30};
        yPlatform = new double[]{3, 3, -3.6, -3.6};

        xCrossBridge = new double[]{-27, -15, -15, 15};
        yCrossBridge = new double[]{-3, -3, 1.6, 1.6};

        xSkystone2 = new double[]{0, 0, 30, 42.25};
        ySkystone2 = new double[]{0, 0, -3.1, -3.1};

        xCrossBridge2 = new double[]{0, -5, -12, -32};
        yCrossBridge2 = new double[]{0, 0, 3, 3};

        xPlatform2 = new double[]{0, -15, -15, -68};
        yPlatform2 = new double[]{5, 5, 1.2, 1.2};

        xCrossBridge3 = new double[]{0, 15, 25, 61};
        yCrossBridge3 = new double[]{0, 0, 5, 5};

        xStone3 = new double[]{0, 0, 13, 31.5};
        yStone3 = new double[]{0, 0, -2.8, -2.8};

        xCrossBridge4 = new double[]{0, -5, -8, -30.5};
        yCrossBridge4 = new double[]{0, 0, 2.7, 2.7};

        xPlatform3 = new double[]{0, -15, -15, -52};
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
        move(0, 1.4, -90, .5, .5, .3, "strafe", true);

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
        move(0, 1.5, -90, .5, .5, .3, "strafe", true);
        grabBlock();

        splineMove(xCrossBridge4, yCrossBridge4, -.8, -.35, -.8, 0, false);
        splineMove(xPlatform3, yPlatform3, -.8, -.8, -.3, 0, true);

        placeStone();
        pause(.3);

        getPlatform();

        grip();
        move(0, 12.5, -77, 1, 0.7, 0.7, "strafe", true);
        move(0, 20, 0, 1, .7, .7, "straight", true);
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