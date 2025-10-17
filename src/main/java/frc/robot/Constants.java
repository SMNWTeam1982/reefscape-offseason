// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class DriveCANIDs{
        public static final int BR_TURN = 0;
        public static final int BR_DRIVE = 0;
        public static final int BR_ENCODER = 0;

        public static final int BL_TURN = 0;
        public static final int BL_DRIVE = 0;
        public static final int BL_ENCODER = 0;

        public static final int FR_TURN = 0;
        public static final int FR_DRIVE = 0;
        public static final int FR_ENCODER = 0;

        public static final int FL_TURN = 0;
        public static final int FL_DRIVE = 0;
        public static final int FL_ENCODER = 0;

        public static final int GYRO_DEVICE = 0;
    }

    // Set this to Mode.REPLAY for AdvantageKit Replay
    public static final Mode simMode = Mode.REAL;

    /** Constants for general configuration of the robot project */
    public static class OperatorConstants {
        public static final String PROJECT_NAME = "Swerve Template";
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        // public static final boolean ENABLE_QUESTNAV = false;
        // public static final boolean ENABLE_PHOTONLIB = false;
        public static final boolean REPLAY_LOGS = false;
    }

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }
}
