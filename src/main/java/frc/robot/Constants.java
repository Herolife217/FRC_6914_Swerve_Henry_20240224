package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 13;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule = 
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.25);
        public static final double wheelBase = Units.inchesToMeters(25); 
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
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
        
        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot
 
        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int angleMotorID = 1;
            public static final int driveMotorID = 2;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(306.48);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int angleMotorID = 3;
            public static final int driveMotorID = 4;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(86.05);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int angleMotorID = 5;
            public static final int driveMotorID = 6;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(190.99);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int angleMotorID = 7;
            public static final int driveMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(9.76);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
               
                
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

//* arm constants
            
    }
    public static final class ArmConstants {
        public static final int kLeftID = 14;
        public static final int kRightID = 15;
        public static final double kStowPos = 0.5;
    
        //these 2 should be the same
        public static final double kIntakePos = 0.06;
        public static final double kSubwooferPos = 0.06;
    
        public static final double kFrontAmpPos = 0.15;
    
        // 3m position
        public static final double k3mPos = 0.12;
    
        public static final double kBackAmpPos = 0.27;
        public static final double kUpperLimit = 0.3;
        public static final double kLowerLimit = 0.01;
        public static final double kOverrunLimit = 0;
    
        public static final double kP = 1.5; 
        public static final double kI = 0;
        public static final double kD = 0; 
        public static final double kIz = 0; 
        public static final double kFF = 0; 
        public static final double kMaxOutput = 0.7; 
        public static final double kMinOutput = -0.7;
        public static final double kMaxAccel = 0.18;
        public static final double kMaxVel = 0.85;
    
        public static final double kTolearance = 0.002;
        public static final double kManualSpeed = 0.3;

//*  shooter constants

      }
      public static final class ShooterConstants {
        public static final int kTopID = 8;
        public static final int kBottomID = 9;
    
        // init v should be 6.7 m/s for subwoofer at intake pos
        public static final double kSubwooferSpeed = 1259;
    
        // init v should be 8.2 m/s
        public static final double k3mSpeed = 1541;
    
        public static final double kCompenstion = 1.13;
    
        public static final double kIdleSpeed = 0.3;
        public static final double kAmpSpeed = 0.2;
    
        public static final double kP = 6e-5;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kFF = 0.000015;
        public static final double kIz = 0;
        public static final double kMinOutput = -1;
        public static final double kMaxOutput = 1;
        public static final double kMaxRPM = 5676;
        public static final double kLaunchTime = 0.5;

//* Climber constants 

      }
    public static final class ClimberConstants {
    public static final int kPortID = 15;
    public static final int kStarboardID = 16;
    public static final int kPortDIO = 1;
    public static final int kStarboardDIO = 2;
    public static final double kP = 0.1; 
    public static final double kI = 1e-4;
    public static final double kD = 1; 
    public static final double kIz = 0; 
    public static final double kFF = 0; 
    public static final double kMaxOutput = 1; 
    public static final double kMinOutput = -1;

    public static final double kHomeSpeed = -0.4;
    
    public static final double kUpperLimit = 10;
    public static final double kLowerLimit = 0;
    public static final double kManualSpeed = 0;
}

// Cannot remember if the Extreme 3D PRO is 0 index or not.
public static final class DriverConstants {
  public static final double kDefaultSpeed = 0.8;
  public static final int kSetX = 5;
  public static final int kIntake = 0;
  public static final int kHoldArmDown = 6;
  public static final int kAutoIntake = 3; 
  public static final int kAutoAmp = 4;
  public static final int kSelfDestruct = 9;
  public static final int kTurbo = 1;

//* intake constants

}
 public static final class IntakeConstants {
            public static final int kIntakeID = 10;
            public static final int kSensorDIOPort = 0;
            public static final double kIntakeSpeed = 3098;
            public static final double kFeedSpeed = 3083;
            public static final double kReverseSpeed = 0.4;
            public static final double kCompenstion = 1.83;
        
            public static final double kP = 6e-5;
            public static final double kI = 0.0;
            public static final double kD = 0.0;
            public static final double kFF = 0.000015;
            public static final double kIz = 0;
            public static final double kMinOutput = -1;
            public static final double kMaxOutput = 1;
            public static final double kMaxRPM = 11000;    
          }

          public static final class OIConstants {
            public static final int kDriverControllerPort = 0;
            public static final double kDriveDeadband = 0.1;
            public static final int kOperatorControllerPort = 1;
             }

             // operator constants

            public static final class OperatorConstants {
                public static final double kManoeuvreSpeed = 0.4; 
            }
    }

  
