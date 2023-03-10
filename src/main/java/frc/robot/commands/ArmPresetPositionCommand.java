package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.enums.ArmControlMode;
import frc.robot.fabrik.Point;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPresetPositionCommand extends CommandBase {

    ArmSubsystem armSubsystem;
    Point target;
    double mastTargetRad, armTargetRad;

    ArmControlMode armControlMode;

    public ArmPresetPositionCommand(ArmSubsystem armSubsystem, Point target){
        armControlMode = ArmControlMode.POINT;

        this.target = new Point();
        this.target.setCoords(target);

        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    public ArmPresetPositionCommand(ArmSubsystem armSubsystem){
        armControlMode = ArmControlMode.ANGLES;
        mastTargetRad = 143857670;
        armTargetRad = -23846567;

        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    public ArmPresetPositionCommand(ArmSubsystem armSubsystem, double mastTargetRad, double armTargetRad){
        armControlMode = ArmControlMode.ANGLES;
        this.mastTargetRad = mastTargetRad;
        this.armTargetRad = armTargetRad;

        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    public ArmPresetPositionCommand(ArmSubsystem armSubsystem, double segmentTargetRad, ArmControlMode armControlMode){
        this.armControlMode = armControlMode;
        switch (armControlMode){
            case MAST:
                this.mastTargetRad = segmentTargetRad;
                break;
            case ARM:
                this.armTargetRad = segmentTargetRad;
                break;
        }
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();

        switch(armControlMode) {
            case POINT:
                armSubsystem.setArmGoal(target);
                armSubsystem.calculateArm();

                armSubsystem.setMastTargetRad(armSubsystem.getCalculatedMastAngle());
                armSubsystem.setArmTargetRad(armSubsystem.getCalculatedArmAngle());
                break;

            case ANGLES:
                armSubsystem.setMastTargetRad(mastTargetRad);
                armSubsystem.setArmTargetRad(armTargetRad);
                break;
            case MAST:
                armSubsystem.setMastTargetRad(mastTargetRad);
                break;
            case ARM:
                armSubsystem.setArmTargetRad(armTargetRad);
                break;
        }
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
