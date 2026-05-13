// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.IndexForSecsCommand;
import frc.robot.commands.IntakeDownCommand;
import frc.robot.commands.ShootForSecsCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.IntakeUpCommand;
import frc.robot.commands.Intake;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final CommandXboxController joystick = new CommandXboxController(0);
    

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem();
    public final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();

    
	private SendableChooser<Command> m_chooser = new SendableChooser<>();
    
    
    public RobotContainer() {
        configureBindings();
        //m_chooser = AutoBuilder.buildAutoChooser("SAS");
        //dont change, it boot loops!!!!!
        

        //NamedCommands.registerCommand("SAS", getAutonomousCommand());
        //m_chooser.addOption("SAS", getAutonomousCommand());
        NamedCommands.registerCommand("ShootCommand",
            new ShootForSecsCommand(m_ShooterSubsystem, 4)
        );
         NamedCommands.registerCommand("IndexCommand",
            new IndexForSecsCommand(m_IndexerSubsystem, 4)
        );
        NamedCommands.registerCommand("IntakeDownCommand",
            new IntakeDownCommand(m_IntakeSubsystem, 1.5)
        );
        NamedCommands.registerCommand("RevCommand",
            new ShootForSecsCommand(m_ShooterSubsystem, 2)
        );
        NamedCommands.registerCommand("UpCommand",
            new IntakeUpCommand(m_IntakeSubsystem, 2)
        );
        NamedCommands.registerCommand("Intake",
            new Intake(m_IntakeSubsystem, 2)
        );
        NamedCommands.registerCommand("Shoot", 
           new ParallelCommandGroup(
            new IndexForSecsCommand(m_IndexerSubsystem, 4),
            new ShootForSecsCommand(m_ShooterSubsystem, 4)

        )
        );
        NamedCommands.registerCommand("Rev and Shoot", 
        shootCommandGroup());
        //add named command that calls the sequential command group method below, add this to pathplanner as named command
		m_chooser.addOption("SAS", drivetrain.getAutonomousCommand("SAS"));
        SmartDashboard.putData("Automode", m_chooser);

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->  
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        
        new JoystickButton(joystick.getHID(), 7)
		.onTrue(
				new InstantCommand(() -> drivetrain.resetGyro(), drivetrain)
		);
       //index
        new JoystickButton(joystick.getHID(), ControllerConstants.indexer)
        .onTrue(new InstantCommand(
            () -> m_IndexerSubsystem.startIndexerCommand(),
            m_IndexerSubsystem));
        new JoystickButton(joystick.getHID(), ControllerConstants.indexer)
        .onFalse(new InstantCommand(
            () -> m_IndexerSubsystem.stopIndexerCommand(),
            m_IndexerSubsystem));
        //rev
        new JoystickButton(joystick.getHID(), ControllerConstants.shooterRev)
        .onTrue(new InstantCommand(
            () -> m_ShooterSubsystem.startShooterCommand(),
            m_ShooterSubsystem));
        new JoystickButton(joystick.getHID(), ControllerConstants.shooterRev)
        .onFalse(new InstantCommand(
            () -> m_ShooterSubsystem.stopShooterCommand(),
            m_ShooterSubsystem));
        
        //upIntake
        new JoystickButton(joystick.getHID(), ControllerConstants.intakeUp)
        .onTrue(new InstantCommand(
            () -> m_IntakeSubsystem.MoveUpIntakeCommand(),
            m_IntakeSubsystem));
        new JoystickButton(joystick.getHID(), ControllerConstants.intakeUp)
        .onFalse(new InstantCommand(
            () -> m_IntakeSubsystem.StopIntakeElavator(),
            m_IntakeSubsystem));
       
        //downIntake
        new JoystickButton(joystick.getHID(), ControllerConstants.intakeDown)
        .onTrue(new InstantCommand(
            () -> m_IntakeSubsystem.MoveDownIntakeCommand(),
            m_IntakeSubsystem));
        new JoystickButton(joystick.getHID(), ControllerConstants.intakeDown)
        .onFalse(new InstantCommand(
            () -> m_IntakeSubsystem.StopIntakeElavator(),
            m_IntakeSubsystem));
        //intake run
        new JoystickButton(joystick.getHID(), ControllerConstants.intake)
        .onTrue(new InstantCommand(
            () -> m_IntakeSubsystem.startIntakeCommand(),
            m_IntakeSubsystem));
        new JoystickButton(joystick.getHID(), ControllerConstants.intake)
        .onFalse(new InstantCommand(
            () -> m_IntakeSubsystem.stopIntakeCommand(),
            m_IntakeSubsystem));
        

        

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        //final var idle = new SwerveRequest.Idle();
        // m_chooser.getSelected();
            return m_chooser.getSelected();
        // return Commands.sequence(
        //     // Reset our field centric heading to match the robot
        //     // facing away from our alliance station wall (0 deg).
        //     drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        //     // Then slowly drive forward (away from us) for 5 seconds.
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(0.5)
        //             .withVelocityY(0)
        //             .withRotationalRate(0)
        //     )
        //     .withTimeout(5.0),
        //     // Finally idle for the rest of auton
        //     drivetrain.applyRequest(() -> idle)
            
        // );
              //return m_chooser.getSelected();

    }

    public SequentialCommandGroup shootCommandGroup(){
        return  new SequentialCommandGroup(
            new ShootForSecsCommand(m_ShooterSubsystem, 1.5),
            new ParallelCommandGroup(
                new ShootForSecsCommand(m_ShooterSubsystem, 4),
                new IndexForSecsCommand(m_IndexerSubsystem, 4)
            )
        );

    }
}
