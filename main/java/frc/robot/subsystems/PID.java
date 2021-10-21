package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.Teleop;

public class PID
{
    private static float cof;
    private static int j;
    private float kp;
    private float ki;
    private float kd;
    private float minLimit;
    private float maxLimit;

    private float sampleTime = 0.01f;
    public float setpoint = 0;
    private boolean autoMode = true;

    public float realEnc = 0;

    public float error;
    public float lastError;

    private float lastOut = 0;
    private float lastInp = 0;
    private float lastTime = 0;

    private float p = 0;
    private float i = 0;
    private float d = 0;

    private float[][] arrForPIDEncs = {{0, 10f, 35f, 55f, 75f}, { 0, 20, 50, 80, 100}};

    public float dProcessCheck;

    public void MyPID2(float KP, float KI, float KD, float[] range)
    {
        this.kp = KP; 
        this.ki = KI;
        this.kd = KD;
        this.minLimit = range[0];
        this.maxLimit = range[1];

        this.reset();
    }

    public float call(float process)
    {
        if (!this.autoMode) return this.lastOut;
        float now = (float)Timer.getFPGATimestamp() * 1000;
        float dt = (now - this.lastTime > 0) && (now - this.lastTime < 20f) ? now - this.lastTime : 20f;
        dt /= 1000;
        
        if (this.sampleTime != 0f && dt < this.sampleTime)
        {
            return this.lastOut;
        }

        float dProcess = TransFunc.TransF(arrForPIDEncs, (float) (process - this.lastInp) * 100f, j, cof);

        this.error = this.setpoint - dProcess;

        this.p = this.error;
        float plusToI = this.error * dt;
        this.i += plusToI;
        this.i = Teleop.InRange(this.i, this.minLimit, this.maxLimit);
        this.d = (error - this.lastError) / dt;

        float out = this.p * this.kp + this.i * this.ki + this.d * this.kd;
        out = Teleop.InRange(out, this.minLimit, this.maxLimit);

        this.lastError = error;
        this.lastOut = out;
        this.lastInp = process;
        this.lastTime = now;

        return out;
    }

    public void reset()
    {
        this.p = 0;
        this.i = 0;
        this.d = 0;

        this.lastTime = (float)Timer.getFPGATimestamp() * 1000;
        this.lastOut = 0;
    }

    public void resetAll()
    {
        this.lastInp = 0;
        this.reset();
    }

    public void setNewParams(float KP, float KI, float KD)
    {
        this.kp = KP;
        this.ki = KI;
        this.kd = KD;
    }
}