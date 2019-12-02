package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Autonomous.Auto;


@Autonomous(name = "RedSkystonePlatform", group = "Autonomous")
public class RedSkystonePlatform extends Auto {

    private double xSkystone[], ySkystone[];
    private double xCrossBridge[], yCrossBridge[];
    private double xPlatform[] = new double[]{0, 16, 16};
    private double yPlatform[] = new double[]{0, 0, -12};

    private ALLIANCE_COLOR color = ALLIANCE_COLOR.RED;

    public void runOpMode() {
        initialize();
        waitForStart();

        //start auto
        try {
            moveFromWall();

            SKYSTONE_POSITION position = predetermineMovement();

            getFirstSkystone();

            deliverFirstSkystone();

            getPlatform();
        }
        catch(InterruptedException e) { }
    }

    public void moveFromWall() throws InterruptedException {
        strafe(.5, 0, "straferight", 6);

        adjustClaw();
    }

    public SKYSTONE_POSITION predetermineMovement() throws InterruptedException {
        SKYSTONE_POSITION position = determineSkystonePlacement(color);

        if (position.equals(SKYSTONE_POSITION.RIGHT)) {
            xSkystone = new double[]{0, 18, 18};
            ySkystone = new double[]{0, 0, -8};

            xCrossBridge = new double[]{0, -11, -11};
            yCrossBridge = new double[]{0, 0, -104};

        } else if (position.equals(SKYSTONE_POSITION.MIDDLE)) {
            xSkystone = new double[]{0, 26, 26};
            ySkystone = new double[]{0, 0, -6};

            xCrossBridge = new double[]{0, -11, -11};
            yCrossBridge = new double[]{0, 0, -112};
        }
        else {
            strafe(.3, 0, "strafeleft", 3);

            xSkystone = new double[]{0, 10, 10};
            ySkystone = new double[]{0, 0, -8};

            xCrossBridge = new double[]{0, -11, -11};
            yCrossBridge = new double[]{0, 0, -120};
        }
        return position;
    }

    public void getFirstSkystone() throws InterruptedException {
        splineMove(xSkystone, ySkystone, .3, 0); //backwards spline from cube to bridge, ending in same orientation

        move(-90, .3, 19, "straight");

        gripBlock();
        pause(1);
    }

    public void deliverFirstSkystone() throws InterruptedException {
        splineMove(xCrossBridge, yCrossBridge, -.4, -Math.PI / 2);

        PIDTurn(180, 1.0);
    }

    public void getPlatform() throws InterruptedException {
        PIDTurn(90, 1.0);

        move(90, -.5, 21, "straight");
        gripPlatform();
        pause(1);

        splineMove(xPlatform, yPlatform, 0.3, Math.PI/2);

        move(0, .3, 4, "straight");

        releasePlatform();
        pause(.75);

        moveByTime(3, 0, -.3);
    }

    public void parkBot() throws InterruptedException {
        adjustClaw();

        move(179, -.7, 12, "straight");
    }
}

