package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.revrobotics.CANSparkMax;
import com.swervedrivespecialties.swervelib.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.commands.SetupGyroCommand;

import java.util.HashMap;

public class DriveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    public SwerveDriveKinematics kinematics;
    public static SwerveDriveOdometry odometry = null;

    private double gyroPitchOffset;
    private double gyroRollOffset;

    public double driveGyroOffset = 0;

    public AHRS gyro = new AHRS(SPI.Port.kMXP);

    public DriveSubsystem(SwerveDriveKinematics kinematics, double[] offsets, int[] speedControllers) {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

        /* How to adjust steer offsets:
        Set offset to zero and deploy and enable the robot
        Disable the robot and unplug the target cancoder
        Plug it back in and re-enable the robot and wait for the angle to reset its self
        Add the negative of the current angle from the shuffleboard to the steer offset
        Once again enable the robot and disable it and unplug the cancoder and plug it in
         */
        MkModuleConfiguration configuration = MkModuleConfiguration.getDefaultSteerNEO();
        configuration.setDriveCurrentLimit(50);

        frontLeftModule = new MkSwerveModuleBuilder(configuration).withLayout(shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withSize(2, 2)
                .withPosition(0, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L1)
                .withDriveMotor(MotorType.NEO, speedControllers[0])
                .withSteerMotor(MotorType.NEO, speedControllers[1])
                .withSteerEncoderPort(50)
                .withSteerOffset(Math.toRadians(-offsets[0]))
                .build();

        frontRightModule = new MkSwerveModuleBuilder(configuration).withLayout(shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 2)
                        .withPosition(2, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L1)
                .withDriveMotor(MotorType.NEO, speedControllers[2])
                .withSteerMotor(MotorType.NEO, speedControllers[3])
                .withSteerEncoderPort(51)
                .withSteerOffset(Math.toRadians(-offsets[1]))
                .build();

        backRightModule = new MkSwerveModuleBuilder(configuration).withLayout(shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 2)
                        .withPosition(4, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L1)
                .withDriveMotor(MotorType.NEO, speedControllers[4])
                .withSteerMotor(MotorType.NEO, speedControllers[5])
                .withSteerEncoderPort(52)
                .withSteerOffset(Math.toRadians(-offsets[2]))
                .build();

        backLeftModule = new MkSwerveModuleBuilder(configuration).withLayout(shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 2)
                        .withPosition(6, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L1)
                .withDriveMotor(MotorType.NEO, speedControllers[6])
                .withSteerMotor(MotorType.NEO, speedControllers[7])
                .withSteerEncoderPort(53)
                .withSteerOffset(Math.toRadians(-offsets[3]))
                .build();

        setRampRates(frontLeftModule, frontRightModule, backLeftModule, backRightModule);

        this.kinematics = kinematics;

        odometry = new SwerveDriveOdometry(
                kinematics,
                Rotation2d.fromDegrees(gyro.getFusedHeading()),
                new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() } );

        shuffleboardTab.addNumber("Gyroscope Angle", () -> getRotation().getDegrees());
        shuffleboardTab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
        shuffleboardTab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());
        shuffleboardTab.addNumber("Roll" , () -> getGyroRoll());
        shuffleboardTab.addNumber("Pitch", () -> getGyroPitch());

        shuffleboardTab.addNumber("Gyro Offset", () -> driveGyroOffset);

    }

    private void setRampRates(SwerveModule... swerveModules) {
        for(var s : swerveModules)
        ((CANSparkMax) s.getDriveMotor()).setOpenLoopRampRate(Constants.rampRate);
    }

    public void zeroDriveGyro() {
        driveGyroOffset = getRotation().getDegrees();
    }

    public void invertZeroDriveGyro() {driveGyroOffset = getRotation().getDegrees() - 180;}

    public void zeroGyroYaw() {
        odometry.resetPosition(
                Rotation2d.fromDegrees(-gyro.getFusedHeading()),
                new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() },
                new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0))
        );
    }

    public void invertGyroYaw() {
        odometry.resetPosition(
                Rotation2d.fromDegrees(-gyro.getFusedHeading() + 180),
                new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() },
                new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0))
        );
    }

    public Rotation2d getRotation() {
        return odometry.getPoseMeters().getRotation();
    }

    public void drive(ChassisSpeeds speeds) {
        odometry.update(
                Rotation2d.fromDegrees(-gyro.getFusedHeading()),
                new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() }
        );

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        frontLeftModule.set(states[0].speedMetersPerSecond * Constants.cDriveSpeed, states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond * Constants.cDriveSpeed, states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond * Constants.cDriveSpeed, states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond * Constants.cDriveSpeed, states[3].angle.getRadians());
    }

    public static final MechanicalConfiguration MK4Inverted = new MechanicalConfiguration(
            0.10033,
            (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0),
            false,
            (15.0 / 32.0) * (10.0 / 60.0),
            true
    );

    public double getGyroPitch() { return gyro.getPitch() - gyroPitchOffset; }

    public double getGyroRoll() { return gyro.getRoll() - gyroRollOffset; }

    public void zeroGyroRollPitch() {
        gyroRollOffset = gyro.getRoll();
        gyroPitchOffset = gyro.getPitch();
    }

    public void resetOdometry(PathPlannerTrajectory path) {
        zeroGyroYaw();
        odometry.resetPosition(gyro.getRotation2d(),
                new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition()} ,path.getInitialHolonomicPose());
    }

    public void resetOdometryButCool(double[] pose) {
        odometry.resetPosition(getRotation(),
                new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition()} ,new Pose2d(pose[0], pose[1], getRotation()));
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory path) {
        return new SequentialCommandGroup(
                //new InstantCommand(() -> resetOdometry(path) ),
                new PPSwerveControllerCommand(
                        path,
                        odometry::getPoseMeters,
                        new PIDController(1, 0, 0),
                        new PIDController(1, 0, 0), //make sure this is the same at the one above it (unless you doin somethin silly)
                        new PIDController(2, 0, 0),
                        this::drive,
                        false,
                        this)
        );
    }

    public SwerveAutoBuilder getAutoBuilder(HashMap<String, Command> eventMap) {
        return new SwerveAutoBuilder(
                odometry::getPoseMeters,
                pose2d -> {},
                new PIDConstants(1, 0, 0),
                new PIDConstants(2, 0, 0),
                this::drive,
                eventMap,
                false,
                this
        );


    }


}
