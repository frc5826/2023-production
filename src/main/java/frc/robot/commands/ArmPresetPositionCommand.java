package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.fabrik.Point;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPresetPositionCommand extends CommandBase {

    ArmSubsystem armSubsystem;
    Point target;

    public ArmPresetPositionCommand(ArmSubsystem armSubsystem, Point target){
        this.target = new Point();
        this.target.setCoords(target);

        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();

        armSubsystem.setArmGoal(target);
        armSubsystem.calculateArm();

        armSubsystem.setMastTargetRad(armSubsystem.getCalculatedMastAngle());
        armSubsystem.setArmTargetRad(armSubsystem.getCalculatedArmAngle());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
