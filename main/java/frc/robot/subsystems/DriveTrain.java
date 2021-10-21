package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;


import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;

import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
// import edu.wpi.first.wpilibj.MedianFilter; 
// import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Teleop;


public class DriveTrain extends SubsystemBase
{
    
    public static float yaw;
    public static float newyaw;
    public static AnalogInput analog;

    public double lasrEncR = 0;
    public double lasrEncL = 0;
    public double lasrEncB = 0;

    private static Ultrasonic sonicL;
    private static Ultrasonic sonicR;
    private static AnalogInput sharpL;
    private static AnalogInput sharpR;

    private TitanQuad leftMotor;
    private TitanQuad rightMotor;
    private TitanQuad backMotor;
    private TitanQuad elevator;

    private float pidSpeedB;
    private float pidSpeedL;
    private float pidSpeedR;

    private static double maxx = 0;
    private static double maxy = 0;
    private static double maxz = 0;

    private TitanQuadEncoder leftEncoder;
    private TitanQuadEncoder rightEncoder;
    private TitanQuadEncoder backEncoder;
    private TitanQuadEncoder elevatorEncoder;

    private TitanQuadEncoder leftEncoderMM;
    private TitanQuadEncoder rightEncoderMM;
    private TitanQuadEncoder backEncoderMM;

    private static double LB = 0;
    private static double blya = 0;

    private PID pidForRightMotor = new PID();
    private PID pidForLeftMotor = new PID();
    private PID pidForBackMotor = new PID();
    private PID pidForLiftMotor = new PID();

    // private static MedianFilter mf = new MedianFilter(15);
    public static DigitalOutput green;
    public static DigitalOutput red;

    private static ArrayList<Double> filter = new ArrayList<>();
    private static double[] sonic = new double[13];

    private AHRS navx;

    private ShuffleboardTab tab = Shuffleboard.getTab("Training Robot");
    private NetworkTableEntry leftEncoderValue = tab.add("Left Encoder", 0).getEntry();
    private NetworkTableEntry rightEncoderValue = tab.add("Right Encoder", 0).getEntry();
    private NetworkTableEntry backEncoderValue = tab.add("Back Encoder", 0).getEntry();
    private NetworkTableEntry gyroValue = tab.add("NavX Yaw", 0).getEntry();
    private NetworkTableEntry XposValueSB = tab.add("x", 0).getEntry();
    private NetworkTableEntry YposValueSB = tab.add("y", 0).getEntry();
    private NetworkTableEntry xSpeedTest = tab.add("xSpeed", 0).getEntry();
    private NetworkTableEntry ySpeedTest = tab.add("ySpeed", 0).getEntry();
    private NetworkTableEntry zSpeedTest = tab.add("zSpeed", 0).getEntry();
    private NetworkTableEntry cof = tab.add("cof", 0).getEntry();
    private NetworkTableEntry sharpIRL = tab.add("Sharp IR left", 0).getEntry();
    private NetworkTableEntry ultraSonicL = tab.add("Ultrasonic left", 0).getEntry();
    private NetworkTableEntry sharpIRR = tab.add("Sharp IR right", 0).getEntry();
    private NetworkTableEntry ultraSonicR = tab.add("Ultrasonic right", 0).getEntry();
    public NetworkTableEntry orderBoardSB = tab.add("Board", 0).getEntry();
    public NetworkTableEntry analogg = tab.add("Conc", 0).getEntry();
    public NetworkTableEntry liftenc = tab.add("elevatorEnc", 0).getEntry();
    public NetworkTableEntry c = tab.add("ser", 0).getEntry();
    public NetworkTableEntry motorR = tab.add("motorR", 0).getEntry();
    public NetworkTableEntry motorL = tab.add("motorL", 0).getEntry();
    public NetworkTableEntry motorB = tab.add("motorB", 0).getEntry();
    public NetworkTableEntry PIDR = tab.add("PIDR", 0).getEntry();
    public NetworkTableEntry PIDL = tab.add("PIDL", 0).getEntry();
    public NetworkTableEntry PIDB = tab.add("PIDB", 0).getEntry();
    public NetworkTableEntry speed = tab.add("speed", 0).getEntry();
    public NetworkTableEntry blue = tab.add("blue", 0).getEntry();
    public NetworkTableEntry white = tab.add("white", 0).getEntry();
    public NetworkTableEntry yellow = tab.add("yellow", 0).getEntry();
    public NetworkTableEntry result = tab.add("result", 0).getEntry();

    public DriveTrain() {

        leftMotor = new TitanQuad(Constants.TITAN_ID, Constants.LEFT_MOTOR);
        rightMotor = new TitanQuad(Constants.TITAN_ID, Constants.RIGHT_MOTOR);
        backMotor = new TitanQuad(Constants.TITAN_ID, Constants.BACK_MOTOR);
        elevator = new TitanQuad(Constants.TITAN_ID, Constants.ELEVATOR_MOTOR);

        leftEncoder = new TitanQuadEncoder(leftMotor, Constants.LEFT_MOTOR, 4. / 256);
        rightEncoder = new TitanQuadEncoder(rightMotor, Constants.RIGHT_MOTOR, 4. / 256);
        backEncoder = new TitanQuadEncoder(backMotor, Constants.BACK_MOTOR, 4. / 256);
        elevatorEncoder = new TitanQuadEncoder(elevator, Constants.ELEVATOR_MOTOR, Constants.ELEVATOR_DIST_TICK);

        leftEncoderMM = new TitanQuadEncoder(leftMotor, Constants.LEFT_MOTOR, Constants.WHEEL_DIST_PER_TICK);
        rightEncoderMM = new TitanQuadEncoder(rightMotor, Constants.RIGHT_MOTOR, Constants.WHEEL_DIST_PER_TICK);
        backEncoderMM = new TitanQuadEncoder(backMotor, Constants.BACK_MOTOR, Constants.WHEEL_DIST_PER_TICK);

        navx = new AHRS(SPI.Port.kMXP);
        sharpL = new AnalogInput(Constants.SHARP_1);
        analog = new AnalogInput(2);
        sonicL = new Ultrasonic(Constants.SONIC_TRIGG_1, Constants.SONIC_ECHO_1);
        sharpR = new AnalogInput(Constants.SHARP_2);
        sonicR = new Ultrasonic(Constants.SONIC_TRIGG_2, Constants.SONIC_ECHO_2);

        green = new DigitalOutput(1);
        red = new DigitalOutput(5);

        float[] range = {-100, 100};
        pidForRightMotor.MyPID2(0.21f, 6.9f, 0.001f, range);
        pidForLeftMotor.MyPID2(0.21f, 6.9f, 0.001f, range);
        pidForBackMotor.MyPID2(0.21f, 6.9f, 0.001f, range);
        pidForLiftMotor.MyPID2(0.21f, 6.9f, 0.001f, range);
    }

    public static void GREEN_ST(boolean gl) {
        green.set(gl);
    }

    public static void RED_ST(boolean rl) {
        red.set(rl);
    }

    /**
     * Sets the speed of the motor
     * <p>
     * 
     * @param speed range -1 to 1 (0 stop)
     */
    public void setLeftMotorSpeed(double speed) {
        if (speed == 0.0) {
            pidForLeftMotor.reset();
            leftMotor.set(0);
        } else {
            pidForLeftMotor.setpoint = (float) speed * 100;
            this.pidSpeedL = pidForLeftMotor.call((float) getLeftEncoderDistance());
            float outSpeed = Teleop.InRange(this.pidSpeedL / 100, -1, 1);
            leftMotor.set(outSpeed);
        }
    }

    /**
     * Sets the speed of the motor
     * <p>
     * 
     * @param speed range -1 to 1 (0 stop)
     */
    public void setRightMotorSpeed(double speed) {
        if (speed == 0.0) {
            pidForRightMotor.reset();
            rightMotor.set(0);
        } else {
            pidForRightMotor.setpoint = (float) speed * 100;
            this.pidSpeedR = pidForRightMotor.call((float) getRightEncoderDistance());
            float outSpeed = Teleop.InRange(this.pidSpeedR / 100, -1, 1);
            rightMotor.set(outSpeed);
        }
    }

    /**
     * Sets the speed of the motor
     * <p>
     * 
     * @param speed range -1 to 1 (0 stop)
     */

    public void setBackMotorSpeed(double speed) {
        if (speed == 0.0) {
            pidForBackMotor.reset();
            backMotor.set(0);
        } else {
            pidForBackMotor.setpoint = (float) speed * 100;
            this.pidSpeedB = pidForBackMotor.call((float) getBackEncoderDistance());
            float outSpeed = Teleop.InRange(this.pidSpeedB / 100, -1, 1);
            backMotor.set(outSpeed);
            // backMotor.set(speed);
        }
    }

    public void setElevatorMotorSpeed(double speed) {
        elevator.set(speed);
    }

    /**
     * Sets the speed of the drive motors
     * <p>
     * 
     * @param speed range -1 to 1 (0 stop)
     */

    public void holonomicDrive(double x, double y, double z) {

        double max = Math.abs(x) + Math.abs(y) + Math.abs(z);
        double cofmax = 1 / max;
        if (max > 1){
            maxx = x * cofmax;
            maxy = y * cofmax;
            maxz = z * cofmax;
        }else{
            maxx = x;
            maxy = y;
            maxz = z;
        }

        double rightSpeed = (-(maxx) + (maxy * 0.5) + maxz);
        double leftSpeed = ((maxx) + (maxy * 0.5) + maxz);
        double backSpeed = -maxy + maxz;

        // if (Math.abs(leftSpeed) > max)
        //     max = Math.abs(leftSpeed);
        // if (Math.abs(backSpeed) > max)
        //     max = Math.abs(backSpeed);

        // if (max > 1) {
        //     rightSpeed /= max;
        //     leftSpeed /= max;
        //     backSpeed /= max;
        // }

        setRightMotorSpeed(rightSpeed);
        setLeftMotorSpeed(leftSpeed);
        setBackMotorSpeed(backSpeed);
    }

    public static double getConcBoolean() {
        return analog.getAverageVoltage();

    }

    public static double getIRLDistance() {
        return (Math.pow(sharpL.getAverageVoltage(), -1.2045) * 27.726);
    }

    public static double getIRRDistance() {
        return (Math.pow(sharpR.getAverageVoltage(), -1.2045) * 27.726);
    }

    public static double getSonicLDistance(boolean metric) {
        sonicL.ping();
        Timer.delay(0.005);
        if (metric)
            return medianFilter(sonicL.getRangeMM() / 10);
        else
            return sonicL.getRangeInches();
    }

    public static double getSonicRDistance(boolean metric) {
        sonicR.ping();
        Timer.delay(0.005);
        if (metric)
            return medianFilter(sonicR.getRangeMM() / 10);
        else
            return sonicR.getRangeInches();
    }
 
    public static double medianFilter(double number){
        if (filter.size() > 12){
            filter.add(number);
            filter.remove(0);
            for (int i = 0 ; i < 13; i ++){
                sonic[i] = filter.get(i);
            }
            Arrays.sort(sonic);
            return sonic[9];
        }else{
            filter.add(number);
            return number;
        }
    }

    /**
     * Gets the encoder distance for the left drive motor
     * <p>
     * 
     * @return distance traveled in mm
     */
    public double getLeftEncoderDistance() {
        return leftEncoder.getEncoderDistance();
    }

    public double getLeftEncoderDistanceMM(){
        return leftEncoderMM.getEncoderDistance();
    }

    /**
     * Gets the encoder distance for the right drive motor
     * <p>
     * 
     * @return distance traveled in mm
     */
    public double getRightEncoderDistance() {
        return rightEncoder.getEncoderDistance();
    }

    public double getRightEncoderDistanceMM(){
        return rightEncoderMM.getEncoderDistance();
    }

    /**
     * Gets the encoder distance for the back drive motor
     * <p>
     * 
     * @return distance traveled in mm
     */
    public double getBackEncoderDistance() {
        return backEncoder.getEncoderDistance();
    }

    public double getBackEncoderDistanceMM(){
        return backEncoderMM.getEncoderDistance();
    }

    public double getElevatorEncoderDistance() {
        return elevatorEncoder.getEncoderDistance();
    }

    public double Speed() {
        blya = backEncoder.getEncoderDistance() - LB;
        LB = backEncoder.getEncoderDistance();
        return blya;
    }

    /**
     * Gets the average forward or reverse encoder distance
     * <p>
     * 
     * @return distance traveled in mm
     */
    public double getAverageForwardEncoderDistance() {
        return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2;
    }

    /**
     * Call for the current Yaw angle from the internal NavX
     * <p>
     * 
     * @return yaw angle in degrees range -180° to 180°
     */
    public double getYaw() {
        float yawtest = navx.getYaw() - DriveTrain.yaw;
        if (Math.abs(yawtest) < 200) {
            DriveTrain.yaw = navx.getYaw();
            return DriveTrain.newyaw += yawtest;
        } else {
            if (yawtest <= 0) {
                DriveTrain.yaw = navx.getYaw();
                return newyaw += yawtest + 360;
            } else {
                DriveTrain.yaw = navx.getYaw();
                return newyaw += yawtest - 360;
            }
        }
    }

    public void resetElevator() {
        elevatorEncoder.reset();
    }
    
    public void resetEncoders()
    {
        leftEncoder.reset();
        rightEncoder.reset();
        backEncoder.reset();
    }

    public void resetYaw()
    {
        navx.zeroYaw();
    }

    public void resetZYaw()
    {
        DriveTrain.newyaw = 0;
        DriveTrain.yaw = 0;
        navx.zeroYaw();
    }

    @Override
    public void periodic()
    {
        leftEncoderValue.setDouble(getLeftEncoderDistance());
        rightEncoderValue.setDouble(getRightEncoderDistance());
        backEncoderValue.setDouble(getBackEncoderDistance());
        gyroValue.setDouble(getYaw());
        XposValueSB.setDouble(Teleop.x);
        YposValueSB.setDouble(Teleop.y);
        zSpeedTest.getDouble(Teleop.zSpeed);
        ySpeedTest.getDouble(Teleop.ySpeed);
        xSpeedTest.getDouble(Teleop.xSpeed);
        cof.getDouble(Teleop.cof);
        sharpIRL.setDouble(getIRLDistance());
        ultraSonicL.setDouble(getSonicLDistance(true));
        sharpIRR.setDouble(getIRRDistance());
        ultraSonicR.setDouble(getSonicRDistance(true));
        analogg.setDouble(getConcBoolean());
        liftenc.setDouble(getElevatorEncoderDistance());
        c.setDouble(Teleop.ser);
        motorR.setDouble(pidForRightMotor.setpoint);
        motorL.setDouble(pidForLeftMotor.setpoint);
        motorB.setDouble(pidForBackMotor.setpoint);
        PIDR.setDouble(pidSpeedR);
        PIDL.setDouble(pidSpeedL);
        PIDB.setDouble(pidSpeedB);
        speed.setDouble(Speed());
        blue.setDouble(JavaCamera.blueArea);
        white.setDouble(JavaCamera.whiteArea);
        yellow.setDouble(JavaCamera.yellowArea);
        result.setDouble(JavaCamera.nowResult);
    }
}