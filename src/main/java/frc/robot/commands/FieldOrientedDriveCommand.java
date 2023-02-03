package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class FieldOrientedDriveCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    public FieldOrientedDriveCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    public void execute() {
        double[] input = getJoystickInput();

        ChassisSpeeds speeds =  ChassisSpeeds.fromFieldRelativeSpeeds(input[0], input[1], input[2], driveSubsystem.getRotation());

        driveSubsystem.drive(speeds);
    }

    public double[] getJoystickInput() {
        double joystickY = Constants.cJoystick.getY() * Math.abs(Constants.cJoystick.getY()) * -1;
        double joystickX = Constants.cJoystick.getX() * Math.abs(Constants.cJoystick.getX()) * -1;
        double joystickZ = Constants.cJoystick.getZ() * Math.abs(Constants.cJoystick.getZ()) * -1;

        if  (Math.abs(joystickY) <= Constants.cJoystickMin) {
            joystickY = 0.0; }
        if (Math.abs(joystickX) <= Constants.cJoystickMin) {
            joystickX = 0.0; }
        if (Math.abs(joystickZ) <= Constants.cJoystickMin) {
            joystickZ = 0.0; }

        double[] joystickReturn = new double[] {joystickY, joystickX, joystickZ};

        return joystickReturn;
    }
}
