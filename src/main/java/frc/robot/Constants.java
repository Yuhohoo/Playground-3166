
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
}

