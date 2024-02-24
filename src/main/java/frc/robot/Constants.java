package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3.92;
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
        
        public static final double kSVolts = 0;
        public static final double kGVolts = 0.335;
        public static final double kVVoltSecondPerRad = 6.24;
        public static final double kAVoltSecondSquaredPerRad = 0.04;
//*  shooter constants

      }
      public static final class ShooterConstants {
        public static final int kTopID = 17;
        public static final int kBottomID = 18;
    
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
    public static final int kPortID = 19;
    public static final int kStarboardID = 20;
    public static final int kPortDIO = 3;
    public static final int kStarboardDIO = 4;
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
            public static final int kIntakeID = 16;
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

  
