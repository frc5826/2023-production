package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    public SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    public AHRS gyro = new AHRS(SPI.Port.kMXP);

    public DriveSubsystem() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

        frontLeftModule = new MkSwerveModuleBuilder().withLayout(shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L1)
                .withDriveMotor(MotorType.NEO, 2)
                .withSteerMotor(MotorType.NEO, 1)
                .withSteerEncoderPort(51)
                .withSteerOffset(Math.toRadians(179.3889))
                .build();

        frontRightModule = new MkSwerveModuleBuilder().withLayout(shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0))
                .withGearRatio(MK4Inverted)
                .withDriveMotor(MotorType.NEO, 4)
                .withSteerMotor(MotorType.NEO, 3)
                .withSteerEncoderPort(49)
                .withSteerOffset(Math.toRadians(357.1037))
                .build();

        backLeftModule = new MkSwerveModuleBuilder().withLayout(shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0))
                .withGearRatio(SdsModuleConfigurations.MK4_L1)
                .withDriveMotor(MotorType.NEO, 8)
                .withSteerMotor(MotorType.NEO, 7)
                .withSteerEncoderPort(50)
                .withSteerOffset(Math.toRadians(354.7767))
                .build();

        backRightModule = new MkSwerveModuleBuilder().withLayout(shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0))
                .withGearRatio(MK4Inverted)
                .withDriveMotor(MotorType.NEO, 6)
                .withSteerMotor(MotorType.NEO, 5)
                .withSteerEncoderPort(52)
                .withSteerOffset(Math.toRadians(178.5477))
                .build();

        Translation2d frontLeftWheel = new Translation2d(0.257, 0.2794);
        Translation2d frontRightWheel = new Translation2d(0.257, -0.2794);
        Translation2d backLeftWheel = new Translation2d(-0.257, 0.2794);
        Translation2d backRightWheel = new Translation2d(-0.257, -0.2794);

        kinematics = new SwerveDriveKinematics(frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel);

        odometry = new SwerveDriveOdometry(
                kinematics,
                Rotation2d.fromDegrees(gyro.getFusedHeading()),
                new SwerveModulePosition[]{ frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition() } );

        shuffleboardTab.addNumber("Gyroscope Angle", () -> getRotation().getDegrees());
        shuffleboardTab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
        shuffleboardTab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());

    }

    public void zeroGyro() {
        odometry.resetPosition(
                Rotation2d.fromDegrees(-gyro.getFusedHeading()),
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



}
