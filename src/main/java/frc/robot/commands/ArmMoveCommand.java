package frc.robot.commands;

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

        if(Math.abs(cJoystick.getY()) >= 0.05){
            armSubsystem.setBaseSpeed(cJoystick.getY());
        }
        else{
            armSubsystem.setBaseSpeed(0);
        }

        if(Math.abs(cJoystick.getZ()) >= 0.05){
            armSubsystem.setMiddleSpeed(cJoystick.getZ());
        }
        else{
            armSubsystem.setMiddleSpeed(0);
        }

    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setBaseSpeed(0);
        armSubsystem.setMiddleSpeed(0);
        super.end(interrupted);
    }
}
