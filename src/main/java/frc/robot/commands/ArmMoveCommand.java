package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

import static frc.robot.Constants.*;

public class ArmMoveCommand extends CommandBase {

    private ArmSubsystem armSubsystem;

    public ArmMoveCommand(ArmSubsystem armSubsystem){

        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);

    }

    @Override
    public void execute() {

        if(cJoystick.getRawButton(5) || cJoystick.getRawButton(3)) {
            if (Math.abs(cJoystick.getY()) >= 0.05) {
                armSubsystem.setMastSpeed(cJoystick.getY());
            } else {
                armSubsystem.setMastSpeed(0);
            }
        } else {
            armSubsystem.setMastSpeed(0);
        }

        if(cJoystick.getRawButton(6) || cJoystick.getRawButton(3)) {
            if (Math.abs(cJoystick.getZ()) >= 0.05) {
                armSubsystem.setArmSpeed(cJoystick.getZ());
            } else {
                armSubsystem.setArmSpeed(0);
            }
        } else {
            armSubsystem.setArmSpeed(0);
        }

    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setMastSpeed(0);
        armSubsystem.setArmSpeed(0);
        super.end(interrupted);
    }
}
