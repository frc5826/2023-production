package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class PlaceholderCommand extends CommandBase {

    ArmSubsystem armSubsystem;

    public PlaceholderCommand(ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();

        System.out.println("Placeholder Init");
    }

    @Override
    public void execute() {
        super.execute();
        armSubsystem.setBaseSpeed(0.4);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setBaseSpeed(0);

        System.out.println("Placeholder End");

        super.end(interrupted);
    }
}
