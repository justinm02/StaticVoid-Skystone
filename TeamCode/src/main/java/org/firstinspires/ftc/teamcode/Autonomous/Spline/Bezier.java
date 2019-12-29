package org.firstinspires.ftc.teamcode.Autonomous.Spline;

public class Bezier {
    private Waypoint[] coords;
    private double[] xCoords, yCoords;
    private int n;
    public Bezier(Waypoint[] points)
    {
        coords = points;
        n = points.length - 1;
        xCoords = new double[points.length];
        yCoords = new double[points.length];
        for(int i = 0; i < points.length; i++) {
            xCoords[i] = points[i].getXcoord();
            yCoords[i] = points[i].getYcoord();
        }
    }
    public Waypoint getPosition(double t)
    {
        double x = 0;
        double y = 0;
        for(int i = 0; i < n; i++) {
            x += Combinatorics.nCr(n,i)*Math.pow(1-t,n-i)*Math.pow(t,i)*xCoords[i];
            y += Combinatorics.nCr(n,i)*Math.pow(1-t,n-i)*Math.pow(t,i)*yCoords[i];
        }
        return new Waypoint(x,y);
    }
    public double getAngle(double t, double offset)
    {
        double dxdt = 0;
        double dydt = 0;
        for(int i = 0; i < n; i++) {
            dxdt += n*Combinatorics.nCr(n-1,i)*Math.pow(1-t,n-i-1)*Math.pow(t,i)*(xCoords[i+1]-xCoords[i]);
            dydt += n*Combinatorics.nCr(n-1,i)*Math.pow(1-t,n-i-1)*Math.pow(t,i)*(yCoords[i+1]-yCoords[i]);
        }
        /*if(dxdt == 0) {
            return (Math.PI / 2 * Math.signum(dydt)) + offset;
        }
        else {
            double thisAngle = offset + Math.atan(dydt/dxdt);
            if (dxdt > 0)
                return thisAngle;
            else {
                double angle = thisAngle + Math.PI;
                if (angle < Math.PI)
                    return angle;
                return angle-2*Math.PI;
            }
        }*/
        double angle = Math.atan2(dydt,dxdt);
        angle += offset;
        if (angle > Math.PI) {
            return angle - 2 * Math.PI;
        } else if (angle <= -Math.PI) {
            return angle + 2 * Math.PI;
        }
        return angle;

    }
    public double getdxdt(double t)
    {
        double dxdt = 0;
        for(int i = 0; i <= n-1; i++) {
            dxdt += n*Combinatorics.nCr(n-1,i)*Math.pow(1-t,n-i-1)*Math.pow(t,i)*(xCoords[i+1]-xCoords[i]);
        }
        return dxdt;
    }
    public double getArcLength()
    {
        double stepsize = .0001;
        double arclength = 0;
        double dxdt = 0;
        double dydt = 0;
        double t = 0;
        while(t < 1)
        {
            for(int i = 0; i <= n-1; i++)
            {
                dxdt += n*Combinatorics.nCr(n-1,i)*Math.pow(1-t,n-i-1)*Math.pow(t,i)*(xCoords[i+1]-xCoords[i]);
                dydt += n*Combinatorics.nCr(n-1,i)*Math.pow(1-t,n-i-1)*Math.pow(t,i)*(yCoords[i+1]-yCoords[i]);
            }
            arclength += Math.sqrt(Math.pow(dxdt,2.0) + Math.pow(dydt,2.0)) * stepsize;
            t += stepsize;
            dxdt = 0;
            dydt = 0;
        }
        return arclength;
    }
}
