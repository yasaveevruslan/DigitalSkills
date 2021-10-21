package frc.robot.commands;

import java.util.ArrayList;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.JavaCamera;
import frc.robot.subsystems.StateClass;
import frc.robot.subsystems.TransFunc;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.gamepad.OI;
import com.studica.frc.Servo;

public class Teleop extends CommandBase {

    /**
     * Bring in DriveTrain and OI
     */

    private static final DriveTrain driveTrain = RobotContainer.driveTrain;
    private static final OI oi = RobotContainer.oi;
    private static ArrayList<StateClass> states = new ArrayList<>();
    private static int currentState = 0;
    private static float startTime = 0;
    private ShuffleboardTab tab = Shuffleboard.getTab("Check Robot");
    private NetworkTableEntry Time = tab.add("Timeerrr", 0).getEntry();

    public static double x;
    public static double y;

    public double LastR;
    public double LastL;
    public double LastB;

    public static float xSpeed;
    public static float ySpeed;
    public static float zSpeed;
    public static boolean xSpeedStop;
    public static boolean ySpeedStop;
    public static boolean zSpeedStop;

    public static boolean xLastStop;
    public static boolean yLastStop;
    public static boolean zLastStop;

    public static float LastX;
    public static float LastY;
    public static float LastZ;
  
    public int j;
    public static float cof;

    double xjk = 0;
    double yjk = 0;
    double zjk = 0;

    double inputLeftY = 0;
    double inputLeftX = 0;
    double inputRightX = 0;
    double inputRightY = 0;

    public static int ser = 0;
    public static int led = 0;
    public static int turbo = 0;
    public static float lift = 0;

    double deltaLeftY = 0;
    double deltaLeftX = 0;
    double deltaRightX = 0;
    double prevLeftY = 0;
    double prevLeftX = 0;
    double prevRightX = 0;
    double elev = 0;
    double deltaElev = 0;
    double prevElev = 0;

    boolean prevser = false;
    boolean prevled = false;
    boolean prevturbo = false;

    double leftMotor = 0;
    double rightMotor = 0;
    double backMotor = 0;
    double max = 0;

    private Servo claw;

    private static final double RAMP_UP = 0.05;
    private static final double RAMP_DOWN = 0.05;
    private static final double DELTA_LIMIT = 0.075;

    public Teleop() {
        addRequirements(driveTrain);
        initialize();
        driveTrain.resetYaw();
        driveTrain.resetEncoders();

        x = 0;
        y = 0;
        this.LastB = 0;
        this.LastR = 0;
        this.LastL = 0;

        claw = new Servo(Constants.DIF_SERVO);

        Thread odoCalcThread = new Thread(() -> {
            while (true) {
                odoDrive(driveTrain.getRightEncoderDistanceMM(), driveTrain.getLeftEncoderDistanceMM(),
                        driveTrain.getBackEncoderDistanceMM(), driveTrain.getYaw());
                try {
                    Thread.sleep(5);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });
        odoCalcThread.setDaemon(false);
        odoCalcThread.start();

        states.add(new StateClass("Servo_cup", 120, 0, 0, 0, false));
        states.add(new StateClass("Servo_cup", 120, 0, 0, 0, false));
        states.add(new StateClass("Servo_cup", 0, 0, 0, 0, false));
        // states.add(new StateClass("sensors", 8, 10, 1, 0, false));
        // states.add(new StateClass("reset", 0, 0, 0, 0, false));
        // states.add(new StateClass("odo", 0, -800, 0, 0, false));
        // states.add(new StateClass("odo", 900, -800, 0, 0, false));
        // states.add(new StateClass("odo", 900, -200, 90, 0, false));
        // states.add(new StateClass("odo", 900, -800, 0, 0, false));
        // states.add(new StateClass("odo", 1800, -800, 0, 0, false));
        // states.add(new StateClass("odo", 1800, -200, 90, 0, false));
        // states.add(new StateClass("odo", 1800, -800, 0, 0, false));
        // states.add(new StateClass("odo", 2800, -800, 0, 0, false));
        // states.add(new StateClass("odo", 2800, -200, 90, 0, false));
        // states.add(new StateClass("odo", 2800, -800, 0, 0, false));
        // states.add(new StateClass("odo", 0, -800, 0, 0, false));
        // states.add(new StateClass("odo", 0, -100, 0, 0, false));
        // states.add(new StateClass("sensors", 8, 10, 1, 0, false));
        // states.add(new StateClass("reset", 0, 0, 0, 0, false));
        // states.add(new StateClass("odo", -400, -100, 0, 0, false));
    }

    @Override
    public void initialize()
    {
        driveTrain.resetEncoders();
        startTime = (float)Timer.getFPGATimestamp();
        driveTrain.resetYaw();
        driveTrain.resetZYaw();

        x = 0;
        y = 0;
        this.LastB = 0;
        this.LastR = 0;
        this.LastL = 0;

        Teleop.LastX = 0;
        Teleop.LastY = 0;
        Teleop.LastZ = 0;

        zSpeedStop = false;
        ySpeedStop = false;
        xSpeedStop = false;

        xLastStop = false;
        yLastStop = false;
        zLastStop = false;
    }

    public void setServoPosition(double degrees)
    {
        claw.setAngle(degrees);
    }

    public void odoDrive(double r,double l,double b,double yaw)
    {
        double dr = r - this.LastR;
        double dl = l - this.LastL;
        double db = b - this.LastB;
        double noneX = (((double) dl - (double) dr) / 2);
        double noneY = ( ((((double) dl) + (double) dr) * 0.5
             - (double)db) / 3) / 0.53;
        double noneZ = Math.toRadians(driveTrain.getYaw());

        x += Math.cos(noneZ) * noneX - Math.sin(noneZ) * noneY;
        y += Math.cos(noneZ) * noneY + Math.sin(noneZ) * noneX;

        this.LastR = r;
        this.LastL = l;
        this.LastB = b;
    }

    public void ByCoordinates(float X, float Y, float Z) {
        float newx = X - (float)x; 
        float newy = Y - (float)y;
        float[][] xTransF = {{0f, 3f, 10f, 20f, 40f, 80f, 100f, 130f, 180f}, {0f, 0.05f, 0.1f, 0.15f, 0.2f, 0.36f, 0.5f, 0.6f, 0.8f}};
        float[][] zTransF = {{ 0f, 0.1f, 1f, 10f, 13f, 15f, 25f, 40f}, {0f, 0.03f, 0.07f, 0.13f, 0.15f, 0.2f, 0.4f, 0.6f}};

        float newT = (float)Math.atan2(newy, newx);
        float newTh = (float)(Math.toRadians((float)driveTrain.getYaw()));
        float newThetaR = newT - newTh;
        float NewR = TransFunc.TransF(xTransF, (float) (Math.sqrt((newx * newx + newy * newy))), j, cof);

        Time.getDouble(Timer.getFPGATimestamp());
        if((Timer.getFPGATimestamp() - startTime) > 1){
            Teleop.xSpeed = InRange((NewR * (float) Math.cos(newThetaR)), -1, 1);
            Teleop.ySpeed = InRange((NewR * (float) Math.sin(newThetaR)), -1, 1);
            Teleop.zSpeed = TransFunc.TransF(zTransF, (Z - (float)driveTrain.getYaw()), j, cof);

        }else{
            Teleop.xSpeed = InRange((NewR * (float) Math.cos(newThetaR)), -1, 1)
                    * (((float) (Timer.getFPGATimestamp() - startTime)));
            Teleop.ySpeed = InRange((NewR * (float) Math.sin(newThetaR)), -1, 1)
                    * (((float) (Timer.getFPGATimestamp() - startTime)));
            Teleop.zSpeed = TransFunc.TransF(zTransF, (Z - (float)driveTrain.getYaw()), j, cof)
             * (((float)(Timer.getFPGATimestamp() - startTime)));
            }
    }

    @Override
    public void execute()
    {
        if (currentState == states.size())
        {
            end(true);
        }

        StateClass curState = states.get(currentState);
        switch (curState.action)
        {
            case "start":
                driveTrain.holonomicDrive(0, 0, 0);
                DriveTrain.GREEN_ST(true);
                DriveTrain.RED_ST(false);
                if (DriveTrain.getConcBoolean() > 1){
                    DriveTrain.GREEN_ST(false);
                    DriveTrain.RED_ST(false);
                    driveTrain.holonomicDrive(0, 0, 0);
                    startTime = (float)Timer.getFPGATimestamp();
                    currentState ++;
                }
                break;

            case "init":
                    Time.getDouble(Timer.getFPGATimestamp());
                    driveTrain.holonomicDrive(curState.p1, curState.p2, curState.p3);

                if (((Timer.getFPGATimestamp()) - startTime > curState.p4)) {
                    driveTrain.holonomicDrive(0, 0, 0);
                    startTime = (float)Timer.getFPGATimestamp();
                    currentState ++;
                    }
                    break;
            
            case "odo":
                    ByCoordinates(curState.p1, curState.p2, curState.p3);
                    driveTrain.holonomicDrive(xSpeed, ySpeed, zSpeed);
                if (InRangeBool((float)(curState.p1 - x), -3, 3) 
                    && InRangeBool((float)(curState.p2 - y), -3, 3) 
                    && InRangeBool((float)driveTrain.getYaw() - curState.p3, -0.05f, 0.05f))
                    {
                    driveTrain.holonomicDrive(0, 0, 0);
                    startTime = (float)Timer.getFPGATimestamp();
                    currentState ++;
                }
                break;
 
            case "sensors":
                SensorsOdo(curState.p1, curState.p2, curState.p3);
                driveTrain.holonomicDrive(xSpeed, ySpeed, zSpeed);
                if (xSpeedStop && ySpeedStop && zSpeedStop){
                    driveTrain.holonomicDrive(0, 0, 0);
                    startTime = (float)Timer.getFPGATimestamp();  
                    currentState ++;
                    xSpeedStop = false;
                    ySpeedStop = false;
                    zSpeedStop = false;
                }
                break;

            case "reset":
                driveTrain.holonomicDrive(0, 0, 0);
                x = curState.p1;
                y = curState.p2;
                DriveTrain.newyaw = curState.p3;
                driveTrain.resetZYaw();
                Time.getDouble(Timer.getFPGATimestamp());
                if (Timer.getFPGATimestamp() - startTime > 0.1){
                    driveTrain.holonomicDrive(0, 0, 0);
                    startTime = (float)Timer.getFPGATimestamp();  
                    currentState ++;
                }
                break;

            case "Servo_cup":
                driveTrain.holonomicDrive(0, 0, 0);
                setServoPosition(curState.p1);
                Time.getDouble(Timer.getFPGATimestamp());
                if (Timer.getFPGATimestamp() - startTime > 5){
                    driveTrain.holonomicDrive(0, 0, 0);
                    startTime = (float)Timer.getFPGATimestamp();  
                    currentState ++;
                }
                break;

            case "Servo_grab":
                driveTrain.holonomicDrive(0, 0, 0);
                setServoPosition(120);
                Time.getDouble(Timer.getFPGATimestamp());
                if (Timer.getFPGATimestamp() - startTime > 2){
                    driveTrain.holonomicDrive(0, 0, 0);
                    startTime = (float)Timer.getFPGATimestamp();  
                    currentState ++;
                }
                break;

            case "Blue":
                driveTrain.holonomicDrive(0, 0, 0);
                if (JavaCamera.nowResult == 3){
                    driveTrain.holonomicDrive(0, 0, 0);
                    startTime = (float)Timer.getFPGATimestamp();
                    currentState ++;
                }
                break;

            case "White":
                driveTrain.holonomicDrive(0, 0, 0);
                if (JavaCamera.nowResult == 2){
                    driveTrain.holonomicDrive(0, 0, 0);
                    startTime = (float)Timer.getFPGATimestamp();
                    currentState ++;
                }
                break;

            case "Yellow":
                driveTrain.holonomicDrive(0, 0, 0);
                if (JavaCamera.nowResult == 1){
                    driveTrain.holonomicDrive(0, 0, 0);
                    startTime = (float)Timer.getFPGATimestamp();
                    currentState ++;
                }
                break;

            case "JK":
                inputLeftX = oi.getLeftDriveX();
                inputLeftY = - oi.getLeftDriveY();
                inputRightX = oi.getRightDriveX();
                inputRightY = oi.getRightDriveY();
                elev = oi.getElevator();

                if (inputRightX > 0){
                    zjk = - inputRightX * 0.4;
                }else if (inputRightY > 0){
                        zjk = inputRightY * 0.4;
                    }
                else{
                    zjk = 0;
                }
        
                deltaLeftX = inputLeftX - prevLeftX;
                deltaLeftY = inputLeftY - prevLeftY;
                deltaElev = elev - prevElev;
                if(deltaLeftX >= DELTA_LIMIT)
                    inputLeftX -= RAMP_UP;
                else if (deltaLeftX <= -DELTA_LIMIT)
                    inputLeftX += RAMP_DOWN;
                if(deltaLeftY >= DELTA_LIMIT)
                    inputLeftY += RAMP_UP;
                else if (deltaLeftY <= -DELTA_LIMIT)
                    inputLeftY -= RAMP_DOWN;
                if (DriveTrain.getConcBoolean() > 1 && elev < 0){
                    driveTrain.resetElevator();
                    elev = 0;
                }
                else if (driveTrain.getElevatorEncoderDistance() < -65 && elev > 0){
                    elev = 0;
                }
                else if(deltaLeftX >= DELTA_LIMIT)
                    elev += RAMP_UP;
                else if (deltaLeftX <= -DELTA_LIMIT)
                    elev -= RAMP_DOWN;
                prevLeftY = inputLeftY;
                prevLeftX = inputLeftX;
                prevElev = elev;
                driveTrain.setElevatorMotorSpeed(-elev * 0.4);

                if (oi.getDriveRightTrigger() && prevser){
                    ser += 1;
                    if (ser >= 2){
                        ser = 0;
                    }
                }

                if (ser == 0){
                    setServoPosition(0);   
                    ser = 0;
                }

                if (ser == 1){
                    setServoPosition(120);
                    ser = 1;
                }

                prevser = !oi.getDriveRightTrigger();

                if (oi.getDriveYButton() && prevled){
                    led += 1;
                    if (led >= 3){
                        led = 0;
                    }
                }
                
                if (led == 0){
                    DriveTrain.GREEN_ST(true);
                    DriveTrain.RED_ST(false);
                }

                if (led == 1){
                    DriveTrain.GREEN_ST(false);
                    DriveTrain.RED_ST(false);                    
                }

                if (led == 2){
                    DriveTrain.GREEN_ST(true);
                    DriveTrain.RED_ST(true);
                }

                prevled = !oi.getDriveYButton();
                if (oi.getDriveLeftTrigger() && prevturbo) {
                    turbo += 1;
                    if (turbo >= 2){
                        turbo = 0;
                    }
                }

                if (turbo == 0){
                    driveTrain.holonomicDrive(prevLeftY, prevLeftX * 0.5, zjk);  
                    turbo = 0;
                }

                if (turbo == 1){
                    driveTrain.holonomicDrive(prevLeftY * 2, prevLeftX, zjk / 0.4);
                    turbo = 1;
                }

                prevturbo = !oi.getDriveLeftTrigger();
                break;

            case "elevator_up":
                lift = curState.p1;
                driveTrain.holonomicDrive(0, 0, 0);
                if (lift == 0){
                    if (DriveTrain.getConcBoolean() > 1){
                        driveTrain.setElevatorMotorSpeed((double)(0));
                        driveTrain.resetElevator();
                        startTime = (float)Timer.getFPGATimestamp();  
                        currentState ++;
                    }else{
                        driveTrain.setElevatorMotorSpeed((double)(0.35));
                    }
                }

                if (lift == 1){
                    driveTrain.setElevatorMotorSpeed(-0.3);
                    if (Math.abs(driveTrain.getElevatorEncoderDistance()) >= 61){
                        driveTrain.setElevatorMotorSpeed(0);
                        startTime = (float)Timer.getFPGATimestamp(); 
                        driveTrain.setElevatorMotorSpeed(0); 
                        currentState ++;
                    }
                }
                break;

            case "stop_prog":
                driveTrain.holonomicDrive(0, 0, 0);
                DriveTrain.GREEN_ST(true);
                DriveTrain.RED_ST(true);
                end(true);
                break;

            default:
                break;
        }
    }

    public static float InRange(float in, float min ,float max)
    {
        return in < min ? min : in > max ? max : in;
    }

    private static boolean InRangeBool(float in, float min, float max)
    {
        return in >= min && in <= max;
    }

    public void SensorsOdo(float X, float Y, float Z){
        int j = 0;
        float cof = 0;
        int j1 = 0;
        float cof1 = 0;
        int j2 = 0;
        float cof2 = 0;

        float[][] xIRtransF = {{0f, 3f, 5f, 7f, 10f, 15f}, {0, 0.13f, 0.32f, 0.51f, 0.64f, 0.8f}};
        float[][] yIRtransF = {{0f, 1f, 3f, 5f, 7f, 10f, 15f}, {0, 0.1f, 0.27f, 0.4f, 0.55f, 0.8f, 1f}};
        float[][] zIRtransF = {{0f, 1f, 2f, 4f, 7f}, {0, 0.1f, 0.2f, 0.4f, 0.7f}};
        float[][] zTransFgyro = {{ 0f, 1f, 5f, 10f, 15f}, {0f, 0.02f, 0.1f, 0.4f, 0.7f}};
        Time.getDouble(Timer.getFPGATimestamp());

        if (X != 0){
            Teleop.xSpeed = TransFunc.TransF(xIRtransF, ((((float)(DriveTrain.getIRRDistance() + DriveTrain.getIRLDistance()) / 2)) - X), j, cof) 
            * InRange((((float)(Timer.getFPGATimestamp() - startTime))), -1, 1);
            xSpeedStop = InRangeBool((((((float)(DriveTrain.getIRRDistance() + DriveTrain.getIRLDistance()) / 2)) - X)), -0.6f, 0.6f);
            Teleop.xLastStop = InRangeBool((((((float)(DriveTrain.getIRRDistance() + DriveTrain.getIRLDistance()) / 2)) - Teleop.LastX)), -0.6f, 0.6f);
        }else{
            Teleop.xSpeed = 0;
            xSpeedStop = true;
            Teleop.xLastStop = true;
        }
        if (Y == 0){
            Teleop.ySpeed = 0;
            ySpeedStop = true;
            Teleop.yLastStop = true;
        }else{
            if (Y > 0){
                Teleop.ySpeed = TransFunc.TransF(yIRtransF, ((float)(DriveTrain.getSonicRDistance(true)) - Y), j1, cof1) 
                * InRange((((float)(Timer.getFPGATimestamp() - startTime))), -1, 1);
                ySpeedStop = InRangeBool((((float)(DriveTrain.getSonicRDistance(true)) - Y)), -0.4f, 0.4f);
                Teleop.yLastStop = InRangeBool((((float)(DriveTrain.getSonicRDistance(true)) - Teleop.LastY)), -0.4f, 0.4f);
            }else if (Y < 0){
                Teleop.ySpeed = TransFunc.TransF(yIRtransF, (Math.abs(Y) - (float)(DriveTrain.getSonicLDistance(true))), j1, cof1)
                 * InRange((((float)(Timer.getFPGATimestamp() - startTime))), -1, 1); 
                ySpeedStop = InRangeBool((Math.abs(Y) - ((float)(DriveTrain.getSonicLDistance(true)))), -0.4f, 0.4f);
                Teleop.yLastStop = InRangeBool((Math.abs(Teleop.LastY) - ((float)(DriveTrain.getSonicLDistance(true)))), -0.4f, 0.4f);
            }
        }

        if (Z == 1){
            if ((float)(DriveTrain.getIRLDistance() + DriveTrain.getIRRDistance()) / 2 < 20){
                Teleop.zSpeed = TransFunc.TransF(zIRtransF, (float)(DriveTrain.getIRLDistance() - DriveTrain.getIRRDistance()), j2, cof2)
                 * InRange((((float)(Timer.getFPGATimestamp() - startTime))), -1, 1);
                zSpeedStop = InRangeBool(((float)(DriveTrain.getIRRDistance() - DriveTrain.getIRLDistance())), -0.2f, 0.2f);
                Teleop.zLastStop = InRangeBool(((float)(DriveTrain.getIRRDistance() - DriveTrain.getIRLDistance())), -0.2f, 0.2f);
            }else{
                Teleop.zSpeed = 0;
                zSpeedStop = true;
                Teleop.zLastStop = true;
            }
        }else{
            Teleop.zSpeed = TransFunc.TransF(zTransFgyro, (Z - (float)driveTrain.getYaw()), j, cof)
             * InRange((((float)(Timer.getFPGATimestamp() - startTime))), -1, 1);  
            zSpeedStop = InRangeBool((Z - (float) driveTrain.getYaw()), -0.5f, 0.5f);
            zLastStop = InRangeBool((Teleop.LastZ - (float) driveTrain.getYaw()), -0.5f, 0.5f);
        }
        Teleop.LastX = X;
        Teleop.LastY = Y;
        Teleop.LastZ = Z;
    }

    @Override
    public void end(boolean interrupted)
    {
       driveTrain.holonomicDrive(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    } 
}