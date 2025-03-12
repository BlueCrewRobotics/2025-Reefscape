package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import org.json.simple.parser.ParseException;

import java.io.IOException;

/**
 * Contains all the robot constants
 */
public final class Constants {
    public static final double stickDeadband = 0.1;

    /**
     * All the Swerve Subsystem constants
     */
    public static final class Swerve {

        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        public static final COTSTalonFXSwerveConstants chosenModule =
        COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21);
        public static final double wheelBase = Units.inchesToMeters(21);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLowerLimit = 25;
        public static final int angleCurrentLimit = 40;
        public static final double angleCurrentLowerTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLowerLimit = 35;
        public static final int driveCurrentLimit = 60;
        public static final double driveCurrentLowerTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = 70;//chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.08403;
        public static final double driveKV = 2.2341;
        public static final double driveKA = 0.27385;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.9;
        /** Radians per Second */
        public static final double maxAngularVelocity = 12.1725; //TODO: This is theoretical!

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /** Constants for the Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 0;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.053223);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /** Constants for the Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.173340);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /** Constants for the Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-.053955);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /** Constants for the Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.218262);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class PhotonVision {
        public static final String Camera_Name_Back = "Camera_Intake_Side";
        public static final String camera_Name_Front = "Camera_Battery_Side";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d April_Tag_Front_pos =
            new Transform3d(new Translation3d(Units.inchesToMeters(11.5d), Units.inchesToMeters(-10.7d), Units.inchesToMeters(9.0d)), new Rotation3d(Math.toRadians(-33d), 0, 0/*Math.PI+3.125*/));
        public static final Transform3d April_Tag_Back_pos =
            new Transform3d(new Translation3d(Units.inchesToMeters(12.5d), Units.inchesToMeters(-5.5d), Units.inchesToMeters(35.3d)), new Rotation3d(Math.toRadians(65d), 0, Math.toRadians(180)/*3.125*/));
        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout tagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);


        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    /**
     * Contains all the Constants used by Path Planner
     */
    public static final class PathPlannerConstants {
        public static final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID
                new PIDConstants(2.0, 0, 0) // Rotation PID
        );

        public static final RobotConfig robotConfig;

        static {
            try {
                robotConfig = RobotConfig.fromGUISettings();
            } catch (IOException e) {
                throw new RuntimeException(e);
            } catch (ParseException e) {
                throw new RuntimeException(e);
            }
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    //elevator constants 
    public static final int ELEVATOR_MOTOR_LEFT_ID = 10;
    public static final int ELEVATOR_MOTOR_RIGHT_ID = 11;
    public static final double ELEVATOR_LOWER_LIMIT = 0.4375; //.403809; //TODO: Placeholder
    //public static final double ELEVATOR_UPPER_LIMIT = 43; //TODO: Placeholder 
    public static final double ELEVATOR_MAX_ROTATIONS_PER_SEC = 80;
    public static final double ELEVATOR_CAN_TO_MOTOR_RATIO = 5.24954/0.916748;
    public static final int ELEVATOR_CAN_CODER_ID = 4;
    public static final double L4REEFPOSITION = 7.332764; //40.575195;
    public static final double L3REEFPOSITION = 4.114502;
    public static final double L2REEFPOSITION = 2.605713;
    public static final double L1REEFPOSITION = 2;
    public static final double DEEPCAGE = 999;
    public static final double PROCESSOR = 9;
    public static final double CORALSTATION = 0.898193; //-10.97

    //intake constants 
    public static final int INTAKE_MOTOR_ID = 8; 

    //wrist constants
    public static final int WRIST_MOTOR_ID = 9;
    public static final double WRIST_UPPER_LIMIT = 19.274414; //-7.884277
    public static final double WRIST_LOWER_LIMIT = -4.093750;//18.414551
    public static final double WRIST_MAX_ROTATIONS_PER_SEC = 100;
    public static final double WIRST_MAX_VOLTAGE = 40;
    public static final double WRIST_INTAKE_POSITION = 2.067871;//2.512207;
    public static final double WRIST_L1_POSITION = 4.617188;
    public static final double WRIST_LMID_POSITION = 10.134766;
    public static final double WRIST_L4_POSITION = 14.634766;
    public static final double WIRST_BARGE_POSITION = -4.212402;
}




