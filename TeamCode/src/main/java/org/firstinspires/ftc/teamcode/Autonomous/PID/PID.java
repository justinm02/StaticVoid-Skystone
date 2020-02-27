package org.firstinspires.ftc.teamcode.Autonomous.PID;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    private double kp, ki, kd;
    private double totalError, lastError, lastTime;

    public PID (double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        lastTime = 0;
    }

    public double getP(double error) {
        return kp * error;
    }

    public double getI(double error) {
        if (Math.abs(error) < 0.15)
            totalError = 0;
        return ki * totalError;
    }

    public double getD(double currentError, ElapsedTime runtime) {
        return kd * (currentError - lastError)/(runtime.time()-lastTime);
    }

    public double getCorrection(double error, ElapsedTime runtime) {
        totalError += error;

        double output = getP(error) + getI(error) + getD(error, runtime);

        lastError = error;
        lastTime = runtime.time();

        return output;
    }
}
