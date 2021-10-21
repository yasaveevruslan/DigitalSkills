/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public final class Constants
{

    /**
     * CAN ID's
     */
    public static final int TITAN_ID                    = 42;

    public static final int SHARP_1           = 0;
    public static final int SONIC_TRIGG_1     = 8;
    public static final int SONIC_ECHO_1      = 9;
    public static final int SHARP_2           = 1;
    public static final int SONIC_TRIGG_2     = 10;
    public static final int SONIC_ECHO_2      = 11;

    /**
     * Drive Base Constants
     */
        /**
         * Motors
         */
        
        public static final int RIGHT_MOTOR                      = 1; //Right Motor
        public static final int BACK_MOTOR                      = 2; //Back Motor
        public static final int LEFT_MOTOR                      = 0; //Left Motor

        /**
         * Encoders
         */
        
        // Omni wheel radius
        public static final double wheelRadius          = 55; //mm

        // Encoder pulse per revolution
        public static final double pulsePerRevolution   = 1440;

        // Gear Ratio between encoder and wheel 
        public static final double gearRatio            = 1/1;

        // Pulse per revolution of wheel
        public static final double wheelPulseRatio      = pulsePerRevolution * gearRatio;

        // Distance per tick
        public static final double WHEEL_DIST_PER_TICK  = (Math.PI * 2 * wheelRadius) / wheelPulseRatio;

    /**
     * Elevator Constants
     */
        /**
         * Motors
         */
        public static final int ELEVATOR_MOTOR                      = 3; //Elevator Motor
        public static final int DIF_SERVO               = 0; //Differential servo

        /**
         * Encoder
         */

        // Radius of the belt pully
        public static final double pulleyRadius         = 7.85; //mm

        // Encoder pulses per revolution
        public static final double pulsePerRevElevator  = 1440;

        // Gear ratio between encoder and pulley
        public static final double elevatorGearRatio    = 2/1;

        // Pulse per revolution of pulley
        public static final double pulleyPulseRatio     = pulsePerRevElevator * elevatorGearRatio;

        // Distance per tick
        public static final double ELEVATOR_DIST_TICK   = (Math.PI * 2 * pulleyRadius) / pulleyPulseRatio;
}
