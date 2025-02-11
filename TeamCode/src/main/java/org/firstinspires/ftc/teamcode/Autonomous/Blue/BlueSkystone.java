package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;
import org.firstinspires.ftc.teamcode.Autonomous.Spline.Waypoint;


@Autonomous(name = "BlueSkystone", group = "Autonomous")
public class BlueSkystone extends Auto {

    private double xSkystone[], ySkystone[], xSkystone2[], ySkystone2[], xCrossBridge2[], xPlatform2[], xCrossBridge3[], xStone3[], xCrossBridge4[], xPlatform3[], movePlatformX[];
    private double xPlatform[], yPlatform[], xCrossBridge[], yCrossBridge[], yCrossBridge2[], yPlatform2[], yCrossBridge3[], yStone3[], yCrossBridge4[], yPlatform3[], movePlatformY[];

    private SKYSTONE_POSITION position;

    public void runOpMode() {
        initialize();
        waitForStart();

        try {
            position = determineSkystonePosition("blue");
            bringAlignerDown("right");

            if (position.equals(SKYSTONE_POSITION.MIDDLE)) {
                goMiddle();
            } else if (position.equals(SKYSTONE_POSITION.LEFT)) {
                goLeft();
            } else {
                goRight();
            }
        } catch (InterruptedException e) {

        }
    }

    public void goLeft() throws InterruptedException {
        xSkystone = new double[]{0, 3, 13.5, 21.5};
        ySkystone = new double[]{0, 0, 3, 3};

        xPlatform = new double[]{0, 5, 32, 37};
        yPlatform = new double[]{5, 5, -1.55, -1.55};

        xCrossBridge = new double[]{42.5, 15, 15, 0};
        yCrossBridge = new double[]{-3, -3, 1.1, 1.1};

        xSkystone2 = new double[]{0, -5, -34, -38.75};
        ySkystone2 = new double[]{0, 0, -2.25, -2.25};

        xCrossBridge2 = new double[]{-2, 5, 12, 40};
        yCrossBridge2 = new double[]{0, 0, 3, 3};

        xPlatform2 = new double[]{0, 5, 50, 60};
        yPlatform2 = new double[]{6.4, 6.4, -1, -1};

        xCrossBridge3 = new double[]{0, -15, -30, -60};
        yCrossBridge3 = new double[]{-1, -1, 4.2, 4.2};

        xStone3 = new double[]{0, 0, -18, -27.25};
        yStone3 = new double[]{0, 0, -2.25, -2.25};

        xCrossBridge4 = new double[]{-2, 5, 8, 24};
        yCrossBridge4 = new double[]{0, 0, 3.2, 3.2};

        xPlatform3 = new double[]{0, 15, 40, 49.5};
        yPlatform3 = new double[]{5, 5, 0, 0};

        movePlatformX = new double[]{0, 17, 20, 20};
        movePlatformY = new double[]{0, 1, 3.5, 9};

        newMove(new Waypoint(0,0), new Waypoint(5.5, -28), 0, .4, .75, 0, true, 4, 2);
        grabBlock("right");

        splineMove(xSkystone, ySkystone, .9, .4, .7, 0, false);
        splineMove(xPlatform, yPlatform, .7, .7, .45, 0, true);

        placeStone("right");
        pause(.3);
        grip();

        splineMove(xCrossBridge, yCrossBridge, -.85, -.4, -.75, 0, false);
        bringAlignerDown("right");
        splineMove(xSkystone2, ySkystone2, -.75, -.75, -.4, 0, true);
        pause(.25);
        //move(0, 1.4, -90, .5, .3, .3, "strafe", true);
        strafeToBlock("right");
        grabBlock("right");

        splineMove(xCrossBridge2, yCrossBridge2, .8, .4, .8, 0, false);
        splineMove(xPlatform2, yPlatform2, .8, .8, .4, 0, true);

        placeStone("right");
        pause(.3);
        grip();

        splineMove(xCrossBridge3, yCrossBridge3, -.9, -.4, -.65, 0, false);
        bringAlignerDown("right");
        splineMove(xStone3, yStone3, -.65, -.65, -.3, 0, true);
        pause(.25);
        //move(0, 2.1, -90, .5, .5, .3, "strafe", true);
        strafeToBlock("right");
        grabBlock("right");

        splineMove(xCrossBridge4, yCrossBridge4, .9, .4, .9, 0, false);
        splineMove(xPlatform3, yPlatform3, .9, .9, .4, 0, true);

        placeStone("right");
        pause(.3);

        getPlatform();

        grip();
        move(180, 8.25, -103, 0.85, 0.6, 0.7, "strafe", true);
        move(180, 17,180, 0.85, 0.6, 0.7, "straight", true);
    }

    public void goMiddle() throws InterruptedException {
        xSkystone = new double[]{0, 5, 15, 25};
        ySkystone = new double[]{0, 0, 3, 3};

        xPlatform = new double[]{0, 5, 30, 39};
        yPlatform = new double[]{5, 5, -1.7, -1.7};

        xCrossBridge = new double[]{43, 36, 15, 0.5};
        yCrossBridge = new double[]{-3, -3, 1.7, 1.7};

        xSkystone2 = new double[]{0, -5, -38, -46};
        ySkystone2 = new double[]{0, 0, -1.75, -1.75};

        xCrossBridge2 = new double[]{0, 5, 12, 40};
        yCrossBridge2 = new double[]{0, 0, 3.9, 3.9};

        xPlatform2 = new double[]{0, 5, 60, 69};
        yPlatform2 = new double[]{7.35, 7.35, 0, 0};

        xCrossBridge3 = new double[]{0, -15, -30, -59.25};
        yCrossBridge3 = new double[]{0, 0, 3.35, 3.35};

        xStone3 = new double[]{0, 0, -10, -17};
        yStone3 = new double[]{0, 0, -1.45, -1.45};

        xCrossBridge4 = new double[]{0, 5, 22, 30};
        yCrossBridge4 = new double[]{0, 0, 4, 4};

        xPlatform3 = new double[]{0, 8, 30, 36.6};
        yPlatform3 = new double[]{7, 7, 0, 0};

        movePlatformX = new double[]{0, 17, 20, 20};
        movePlatformY = new double[]{0, 1, 3.5, 9};

        newMove(new Waypoint(0,0), new Waypoint(-1, -26.4), 0, .4, .9, 0, true, 5, 1.7);
        grabBlock("right");

        splineMove(xSkystone, ySkystone, .85, .4, .7, 0, false);
        splineMove(xPlatform, yPlatform, .7, .7, .45, 0, true);

        placeStone("right");
        pause(.3);
        grip();

        splineMove(xCrossBridge, yCrossBridge, -.85, -.4, -.75, 0, false);
        bringAlignerDown("right");
        splineMove(xSkystone2, ySkystone2, -.75, -.75, -.5, 0, true);
        moveByTime(.5, -.5, 0);
        //move(0, 1.5, -90, .5, .3, .3, "strafe", true);
        strafeToBlock("right");
        grabBlock("right");

        splineMove(xCrossBridge2, yCrossBridge2, .7, .4, .7, 0, false);
        splineMove(xPlatform2, yPlatform2, .7, .7, .4, 0, true);

        placeStone("right");
        pause(.3);
        grip();

        splineMove(xCrossBridge3, yCrossBridge3, -.9, -.4, -.65, 0, false);
        bringAlignerDown("right");
        splineMove(xStone3, yStone3, -.65, -.65, -.3, 0, true);
        pause(.25);
        //move(0, 2.5, -90, .5, .5, .3, "strafe", true);
        strafeToBlock("right");
        grabBlock("right");

        splineMove(xCrossBridge4, yCrossBridge4, .9, .4, .9, 0, false);
        splineMove(xPlatform3, yPlatform3, .9, .9, .4, 0, true);

        placeStone("right");
        pause(.3);

        getPlatform();

        grip();
        move(180, 5.75, -103, 0.85, 0.6, 0.7, "strafe", true);
        move(180, 11, 180, 0.85, 0.6, 0.7, "straight", true);
    }

    public void goRight() throws InterruptedException {
        xSkystone = new double[]{0, 5, 28, 34};
        ySkystone = new double[]{0, 0, 4.5, 4.5};

        xPlatform = new double[]{0, 5, 26, 38};
        yPlatform = new double[]{5, 5, -1.1, -1.1};

        xCrossBridge = new double[]{43, 36, 15, 0.5};
        yCrossBridge = new double[]{-3, -3, 2, 2};

        xSkystone2 = new double[]{0, -5, -38, -46};
        ySkystone2 = new double[]{0, 0, -1.75, -1.75};

        xCrossBridge2 = new double[]{0, 5, 12, 40};
        yCrossBridge2 = new double[]{0, 0, 3.6, 3.6};

        xPlatform2 = new double[]{0, 5, 60, 69};
        yPlatform2 = new double[]{7.9, 7.9, 0, 0};

        xCrossBridge3 = new double[]{0, -15, -30, -59.25};
        yCrossBridge3 = new double[]{0, 0, 4.4, 4.4};

        xStone3 = new double[]{0, 0, -13, -17};
        yStone3 = new double[]{0, 0, -1.4, -1.4};

        xCrossBridge4 = new double[]{0, 3, 16, 30};
        yCrossBridge4 = new double[]{0, 0, 4, 4};

        xPlatform3 = new double[]{0, 8, 30, 37.1};
        yPlatform3 = new double[]{7.1, 7.1, 0, 0};

        movePlatformX = new double[]{0, 17, 20, 20};
        movePlatformY = new double[]{0, 1, 3.5, 9};

        newMove(new Waypoint(0,0), new Waypoint(-8.5, -28.25), 0, .35, .55, 0, true, 2, 2.35);
        grabBlock("right");

        splineMove(xSkystone, ySkystone, .8, .6, .8, 0, false);
        splineMove(xPlatform, yPlatform, .8, .8, .4, 0, true);

        placeStone("right");
        pause(.3);
        grip();

        splineMove(xCrossBridge, yCrossBridge, -.8, -.5, -.7, 0, false);
        bringAlignerDown("right");
        splineMove(xSkystone2, ySkystone2, -.7, -.7, -.4, 0, true);
        moveByTime(.5, -.5, 0);
        //move(0, 1.5, -90, .5, .3, .3, "strafe", true);
        strafeToBlock("right");
        grabBlock("right");

        splineMove(xCrossBridge2, yCrossBridge2, .7, .4, .7, 0, false);
        splineMove(xPlatform2, yPlatform2, .7, .7, .4, 0, true);

        placeStone("right");
        pause(.3);
        grip();

        splineMove(xCrossBridge3, yCrossBridge3, -.9, -.4, -.65, 0, false);
        bringAlignerDown("right");
        splineMove(xStone3, yStone3, -.65, -.65, -.3, 0, true);
        pause(.25);
        //move(0, 2.5, -90, .5, .5, .3, "strafe", true);
        strafeToBlock("right");
        grabBlock("right");

        splineMove(xCrossBridge4, yCrossBridge4, .9, .4, .9, 0, false);
        splineMove(xPlatform3, yPlatform3, .9, .9, .4, 0, true);

        placeStone("right");
        pause(.3);

        getPlatform();

        grip();
        move(180, 6.5, -103, 0.85, 0.6, 0.7, "strafe", true);
        move(180, 11, 180, 0.85, 0.6, 0.7, "straight", true);
    }

    public void grabBlock(String aligner) throws InterruptedException {
        gripBlock(aligner);
        pause(0.32);
        bringAlignerUp(aligner);
    }

    public void placeStone(String aligner) throws InterruptedException {
        releaseBlock(aligner);
        pause(.1);
        bringAlignerUp(aligner);
    }

    public void getPlatform() throws InterruptedException {
        PIDTurn(90, .75);
        move(90, 4, -90, .3, .3, .3, "straight", true);

        gripPlatform();
        pause(0.75);

        splineMove(movePlatformX, movePlatformY, .4, .3, .3, Math.PI/2, true);

        releasePlatform();
        pause(.5);
    }
}