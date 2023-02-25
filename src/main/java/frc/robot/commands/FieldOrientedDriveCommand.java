package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PID;
import frc.robot.subsystems.VisionSubsystem;

public class FieldOrientedDriveCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private PID pidTurn = new PID(0.075, 0, 0.004, 1.75, 0, 1);
    private AHRS gyro;

    public FieldOrientedDriveCommand(DriveSubsystem driveSubsystem) {
        pidTurn.setGoal(0); //once again might be wrong if targeting the opposite side of the field
        gyro = driveSubsystem.gyro;

        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    public void execute() {
        double[] input = new double[3];
        if (DriverStation.isJoystickConnected(0)) {
            input = getJoystickInput();
        } else {
            input = getXboxInput();
        }

        if (Constants.cXbox.getRightBumperPressed()) { //TODO ask drivers which would be best :D
            input[2] = pidTurn.calculate(driveSubsystem.getRotation().getDegrees());
        }

        ChassisSpeeds speeds =  ChassisSpeeds.fromFieldRelativeSpeeds(input[0], input[1], input[2] * Constants.cTurnSpeed,
                driveSubsystem.getRotation().minus(Rotation2d.fromDegrees(driveSubsystem.driveGyroOffset)));
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

    public double[] getXboxInput() {
        double xboxLeftY = Constants.cXbox.getLeftY() * -1;
        double xboxLeftX = Constants.cXbox.getLeftX() * -1;
        double xboxRightX = Constants.cXbox.getRightX() * -1;

        if  (Math.abs(xboxLeftY) <= Constants.cXboxMin) {
            xboxLeftY = 0.0; }
        if (Math.abs(xboxLeftX) <= Constants.cXboxMin) {
            xboxLeftX = 0.0; }
        if (Math.abs(xboxRightX) <= Constants.cXboxMin) {
            xboxRightX = 0.0; }

        double[] xboxReturn = new double[] {xboxLeftY, xboxLeftX, xboxRightX};

        return xboxReturn;
    }

}
