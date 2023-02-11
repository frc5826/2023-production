// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.ArmLockCommand;
import frc.robot.commands.ArmMoveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.GrabbinSubsystem;
import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    ArmSubsystem armSubsystem = new ArmSubsystem();
    VisionSubsystem visionSubsystem = new VisionSubsystem();

    ArmMoveCommand armMoveCommand = new ArmMoveCommand(armSubsystem);
    ArmLockCommand armLockCommand = new ArmLockCommand(armSubsystem);
    FieldOrientedDriveCommand fieldOrientedDriveCommand = new FieldOrientedDriveCommand(driveSubsystem);
    AutoBalanceCommand autoBalanceCommand = new AutoBalanceCommand(driveSubsystem);
    AutoAlignCommand autoAlignCommand = new AutoAlignCommand(driveSubsystem, visionSubsystem);

    GrabbinSubsystem grabbinSubsystem = new GrabbinSubsystem();
    GrabbinCommand grabbinCommand = new GrabbinCommand(grabbinSubsystem);

    JoystickButton trigger = new JoystickButton(cJoystick, 1);

    public RobotContainer()
    {
        // Configure the trigger bindings
        configureBindings();
        CommandScheduler.getInstance().setDefaultCommand(armSubsystem, armMoveCommand);
        CommandScheduler.getInstance().setDefaultCommand(driveSubsystem, fieldOrientedDriveCommand);
    }
    
    
    /** Use this method to define your trigger->command mappings. */
    private void configureBindings()
    {
        trigger.whileTrue(armLockCommand);

        Constants.zeroGyro.onTrue(new InstantCommand(() -> {
            driveSubsystem.zeroGyroYaw();
            driveSubsystem.zeroGyroRollPitch();
        }));

        Constants.autoBalance.whileTrue(autoBalanceCommand);

        Constants.align.whileTrue(autoAlignCommand);
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // TODO: Implement properly
        return driveSubsystem.followTrajectoryCommand(PathPlanner.loadPath("New Path", 1, 1));
    }
}
