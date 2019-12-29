package org.firstinspires.ftc.teamcode.Autonomous.Spline;

public class Combinatorics
{
    public static int nCr(int n, int r)
    {
        return fact(n)/(fact(n-r)*fact(r));
    }
    public static int fact(int x)
    {
        if(x == 0 || x == 1)
        {
            return 1;
        }
        return x*fact(x-1);
    }
}
