
package frc.robot;

//import com.pathplanner.lib.config.PIDConstants;
//import edu.wpi.first.wpilibj.XboxController.Axis;
//import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController.Axis;

public final class Constants {
    

    public static final class ControllerConstants {
        
        public static final int resetNavX = 8;
        public static final int intakeUp = 6; 
        public static final int intakeDown = 5; 
        public static final int intake = 4;
         
        public static final int shooterRev = 3; 
        public static final int indexer = 1; 
       // public static final int k_algaeArmUp = 4;  //Y button
       // public static final int k_algaeArmDown = 1; //A button
       // public static final int k_algaeDisplaceButton = 2; // B button (right)
       // public static final int k_troughRollerForwardButton = 6; //Right bumper
       // public static final int k_troughRollerBackwardButton = 5; //Left bumper
       public final static int k_righttrig = Axis.kRightTrigger.value; // Right Trig
       public final static int k_lefttrig = Axis.kLeftTrigger.value; // Left Trig
    }
    public static final class canID {
        public static final int k_indexCID = 20;
        
    }

    public static final class AutoConstants {
        //public static final PIDConstants k_translationPID = new PIDConstants(0.7, 0, 0);
        //public static final PIDConstants k_anglePID = new PIDConstants(0.4, 0, 0.01);

       // public static final double spinUpAutoTime = 8; 
       // public static final double shootAutoTime = 3; 
        
  }  
  public static final class IndexerConstants{
        public static final double k_IndexerSpeed = .5;
        public static final double k_ShooterIntakeSpeed = .7;

  }
    public static final class ShooterCIDConstants{
        public static final int k_shooterShooter1 = 31; 
        public static final int k_shooterShooter2 = 32; 
        public static final int k_shooterIntake = 30; 
        
    }
    public static final class IndexCIDConstants{
        public static final int k_indexMotor = 20; 
    }
    
    public static final class ShooterConstants{
        public static final double k_shooter1Speed = .65;
        public static final double k_shooter2Speed = -0.65;
       
        
    }
     public static final class IntakeChannelConstants{
        public static final int k_Elavator = 1;
        public static final int k_MainIntake = 2;
       
        
    }
    public static final class IntakeConstants{
        public static final double k_ElivatorMoveSpeedDown = -.5;
        public static final double k_ElivatorMoveSpeedUp = .5;

        public static final double k_MainIntakeSpeed = 1;
       
        
    }

    /** Vision pose fusion (Limelight MegaTag2). Everything here is PLACEHOLDER - TUNE ON ROBOT. */
    public static final class VisionConstants {
        /**
         * Master switch for feeding Limelight poses into the drivetrain pose estimator.
         * Default false so existing odometry/auto behavior is unchanged. Set true only after
         * verifying camera names, camera pose in the Limelight UI, and field map.
         */
        public static final boolean FUSION_ENABLED = false;

        public static final String CAMERA_1 = "limelight";
        /** Optional second camera. Leave "" to disable. */
        public static final String CAMERA_2 = "";

        /** PLACEHOLDER - TUNE ON ROBOT: reject frames while spinning faster than this (deg/s). */
        public static final double MAX_YAW_RATE_DEG_PER_SEC = 720.0;
        /** PLACEHOLDER - TUNE ON ROBOT: reject frames whose timestamp is older than this (s). */
        public static final double MAX_FRAME_AGE_SEC = 0.5;
        /** PLACEHOLDER - TUNE ON ROBOT: base xy std dev (m) at zero distance. */
        public static final double BASE_XY_STD_DEV_M = 0.5;
        /** PLACEHOLDER - TUNE ON ROBOT: distance (m^2) scale; std = base * (1 + d^2 / k). */
        public static final double DISTANCE_SCALE_K = 4.0;
        /** MegaTag2 heading is not trusted; the gyro is. Huge std dev ignores vision rotation. */
        public static final double THETA_STD_DEV_RAD = 9999999.0;
    }

    /** Hold-to-align-to-peg feature. */
    public static final class AlignConstants {
        /**
         * MASTER ENABLE. When false the align button does nothing (not bound).
         * WARNING: PEG_TAG_IDS and the field layout JSON are UNKNOWN placeholders and MUST be
         * verified against the game manual / field before setting this true.
         */
        public static final boolean ALIGN_ENABLED = false;

        /**
         * Custom Stow and Seal field layout, in src/main/deploy. Shipped as an EMPTY placeholder;
         * with no tags, alignment and vision fusion stay disabled at runtime.
         */
        public static final String FIELD_LAYOUT_FILE = "stow_and_seal_tags.json";

        /** INFERRED from CAD part names, confirm with Limelight on field */
        public static final int[] PEG_TAG_IDS = {4, 8};

        /** PLACEHOLDER - TUNE ON ROBOT: robot-center distance in front of the tag face (m). */
        public static final double OFFSET_DISTANCE_M = 0.6;
        /** PLACEHOLDER - TUNE ON ROBOT: lateral offset, +left when looking at the tag from the front (m). */
        public static final double OFFSET_LATERAL_M = 0.0;

        // PLACEHOLDER - TUNE ON ROBOT: gains, caps, tolerances
        public static final double XY_KP = 2.0, XY_KI = 0.0, XY_KD = 0.0;
        public static final double THETA_KP = 3.0, THETA_KI = 0.0, THETA_KD = 0.0;
        public static final double MAX_SPEED_MPS = 1.0;
        public static final double MAX_ACCEL_MPS2 = 1.5;
        public static final double MAX_ANGULAR_RAD_PER_SEC = Math.PI;
        public static final double MAX_ANGULAR_ACCEL_RAD_PER_SEC2 = 2 * Math.PI;
        public static final double XY_TOLERANCE_M = 0.03;
        public static final double THETA_TOLERANCE_RAD = Math.toRadians(2.0);
        /** Consecutive in-tolerance loops (with fresh vision) before isAligned() is true. */
        public static final int ALIGNED_LOOPS = 10;
        /** Command ends NOT aligned after this long. */
        public static final double TIMEOUT_SEC = 5.0;
        /** Command ends NOT aligned if no accepted vision frame for this long. */
        public static final double LOST_TARGET_SEC = 0.5;

        /** Driver controller bindings (CommandXboxController). */
        // Align: hold right trigger. Score: left trigger.

        /**
         * Score binding is disabled: there is NO beam break sensor in this codebase.
         * TODO: add a beam break (DigitalInput) subsystem, wire it into
         * RobotContainer.pieceHeld, then set this true.
         */
        public static final boolean SCORE_BINDING_ENABLED = false;
    }
}

