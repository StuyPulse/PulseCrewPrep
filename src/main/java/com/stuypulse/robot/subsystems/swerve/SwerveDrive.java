package com.stuypulse.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    
    public final static SwerveDrive instance;

    static {
        instance = new SwerveDrive(
            new SwerveModule(Swerve.FrontRight.ID, Swerve.FrontRight.MODULE_OFFSET, Swerve.FrontRight.ABSOLUTE_OFFSET, Swerve.FrontRight.TURN, Swerve.FrontRight.DRIVE, Swerve.FrontRight.ENCODER),
            new SwerveModule(Swerve.FrontLeft.ID, Swerve.FrontLeft.MODULE_OFFSET, Swerve.FrontLeft.ABSOLUTE_OFFSET, Swerve.FrontLeft.TURN, Swerve.FrontLeft.DRIVE, Swerve.FrontLeft.ENCODER),
            new SwerveModule(Swerve.BackLeft.ID, Swerve.BackLeft.MODULE_OFFSET, Swerve.BackLeft.ABSOLUTE_OFFSET, Swerve.BackLeft.TURN, Swerve.BackLeft.DRIVE, Swerve.BackLeft.ENCODER),
            new SwerveModule(Swerve.BackRight.ID, Swerve.BackRight.MODULE_OFFSET, Swerve.BackRight.ABSOLUTE_OFFSET, Swerve.BackRight.TURN, Swerve.BackRight.DRIVE, Swerve.BackRight.ENCODER)
        );
    }

    public static SwerveDrive getInstance() {
        return instance;
    }
    
    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;
    private final AHRS gyro;
    private final FieldObject2d[] modules2ds;
    private final SwerveDriveOdometry odometry;
    private final Field2d field;
    
    public SwerveDrive(SwerveModule... modules) {
        this.modules = modules;
        kinematics = new SwerveDriveKinematics(getModuleOffsets());
        gyro = new AHRS(SPI.Port.kMXP);
        modules2ds = new FieldObject2d[modules.length];
        field = new Field2d();
        odometry = new SwerveDriveOdometry(kinematics, getGyroAngle(), new SwerveModulePosition[modules.length]);

        initModule2ds(field);
        reset(new Pose2d());
    }

    // Constants    
    private interface Swerve {
        //TODO: Change these values to match the actual robot
        double WIDTH = Units.inchesToMeters(26);
        double LENGTH = Units.inchesToMeters(26);

        double MODULE_VELOCITY_DEADBAND = 0.02;
        double MAX_MODULE_SPEED = 5.06;
        double MAX_MODULE_TURN = 6.28;

        public interface FrontRight {
            //XXX: Constants to be updated
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(65.566406);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * -0.5);

            // Ports
            int DRIVE = 10;
            int TURN = 11;
            int ENCODER = 2;
        }
        
        public interface FrontLeft {
            //XXX: Constants to be updated
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(47.197266);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * +0.5);

            // Ports
            int DRIVE = 12;
            int TURN = 13;
            int ENCODER = 1;
        }

        public interface BackLeft {
            //XXX: Constants to be updated
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(208.212891);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * +0.5);

            // Ports
            int DRIVE = 14;
            int TURN = 15;
            int ENCODER = 4;
        }

        public interface BackRight {
            //XXX: Constants to be updated
            String ID = "Back Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(154.511719);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * -0.5);
            
            // Ports
            int DRIVE = 16;
            int TURN = 17;
            int ENCODER = 3;
        }    
    }

    public void initModule2ds(Field2d field) {
        for(int i=0; i<modules.length; i++) {
            modules2ds[i] = field.getObject(modules[i].getID() + "-2d");
        }
    }

    public Translation2d[] getModuleOffsets() {
        Translation2d[] offsets = new Translation2d[modules.length];

        for(int i=0; i<modules.length; i++) {
            offsets[i] = modules[i].getOffset();
        }

        return offsets;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        
        for(int i=0; i<modules.length; i++) {
            states[i] = modules[i].getState();
        }

        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];

        for(int i=0; i<modules.length; i++) {
            positions[i] = modules[i].getModulePosition();
        }

        return positions;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public SwerveModuleState filterModuleState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) > Swerve.MODULE_VELOCITY_DEADBAND) {
            return state;
        }

        return new SwerveModuleState(0, state.angle);
    }

    public void setModuleStates(SwerveModuleState... states) {
        if (states.length != modules.length) {
            throw new IllegalArgumentException(
                String.format("State count mismatch error: %d states does not equal %d modules", states.length, modules.length)
            );
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Swerve.MAX_MODULE_SPEED);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(filterModuleState(states[i]));
        }
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    public void drive(Translation2d velocity, double rotation) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(velocity.getX(), -velocity.getY(), -rotation, getPose().getRotation());
        
        Pose2d robotVel = new Pose2d(
            0.02 * speeds.vxMetersPerSecond,
            0.02 * speeds.vyMetersPerSecond,
            Rotation2d.fromRadians(0.02 * speeds.omegaRadiansPerSecond));

        Twist2d twistVel = new Pose2d().log(robotVel);
        
        setChassisSpeeds(new ChassisSpeeds(
            twistVel.dx / 0.02,
            twistVel.dy / 0.02,
            twistVel.dtheta / 0.02
        ));
    }
    
    public void stop() {
        setChassisSpeeds(new ChassisSpeeds());
    }

    public Rotation2d getGyroAngle() {
        return gyro.getRotation2d();
    }

    public Rotation2d getGyroYaw() {
        return getGyroAngle();
    }

    public Rotation2d getGyroPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    public Rotation2d getGyroRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll());
    }

    
    public double getForwardAccelerationGs() {
        return gyro.getWorldLinearAccelY();
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void setXMode() {
        setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(225)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(315)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        });
    } 

    public Field2d getField() {
        return field;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void reset(Pose2d pose) {
        odometry.resetPosition(getGyroAngle(), getModulePositions(), pose);
    }

    public void updateOdomentry() {
        odometry.update(getGyroAngle(), getModulePositions());
    }

    @Override
    public void periodic() {
         // need to complete writing this without using StuyLib
        updateOdomentry();
        Pose2d pose = getPose();

        for (int i = 0; i < modules.length; i++) {
            modules2ds[i].setPose(new Pose2d(
                pose.getTranslation().plus(modules[i].getOffset().rotateBy(pose.getRotation())),
                modules[i].getAngle().plus(pose.getRotation())
            ));
        }

        SmartDashboard.putNumber("Swerve/Gyro/Angle (deg)", getGyroAngle().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro/Pitch (deg)", getGyroPitch().getDegrees());
        SmartDashboard.putNumber("Swerve/Gyro/Roll (deg)", getGyroRoll().getDegrees());

        SmartDashboard.putNumber("Swerve/Forward Acceleration  (Gs)", getForwardAccelerationGs());
        SmartDashboard.putNumber("Swerve/X Acceleration (Gs)", gyro.getWorldLinearAccelX());
        SmartDashboard.putNumber("Swerve/Y Acceleration (Gs)", gyro.getWorldLinearAccelY());
        SmartDashboard.putNumber("Swerve/Z Acceleration (Gs)", gyro.getWorldLinearAccelZ());

        SmartDashboard.putNumber("Swerve/Chassis X Speed", getChassisSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Chassis Y Speed", getChassisSpeeds().vyMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Chassis Rotation", getChassisSpeeds().omegaRadiansPerSecond);
    }
}
