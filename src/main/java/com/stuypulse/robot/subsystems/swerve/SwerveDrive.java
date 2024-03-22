package com.stuypulse.robot.subsystems.swerve;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
    
    public SwerveDrive(SwerveModule... modules) {
        this.modules = modules;
        kinematics = new SwerveDriveKinematics(getModuleOffsets());
        gyro = new AHRS(SPI.Port.kMXP);
        modules2ds = new FieldObject2d[modules.length];
    }

    // Constants    
    private interface Swerve {
        double WIDTH = Units.inchesToMeters(26);
        double LENGTH = Units.inchesToMeters(26);

        double MODULE_VELOCITY_DEADBAND = 0.02;
        double MAX_MODULE_SPEED = 5.06;
        double MAX_MODULE_TURN = 6.28;

        public interface FrontRight {
            // Constants
            String ID = "Front Right";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(65.566406);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * -0.5);

            // Ports
            int DRIVE = 10;
            int TURN = 11;
            int ENCODER = 2;
        }
        
        public interface FrontLeft {
            // Constants
            String ID = "Front Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(47.197266);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * +0.5, LENGTH * +0.5);

            // Ports
            int DRIVE = 12;
            int TURN = 13;
            int ENCODER = 1;
        }

        public interface BackLeft {
            // Constants
            String ID = "Back Left";
            Rotation2d ABSOLUTE_OFFSET = Rotation2d.fromDegrees(208.212891);
            Translation2d MODULE_OFFSET = new Translation2d(WIDTH * -0.5, LENGTH * +0.5);

            // Ports
            int DRIVE = 14;
            int TURN = 15;
            int ENCODER = 4;
        }

        public interface BackRight {
            // Constants
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

    // IMPLEMENT DRIVE METHOD WITHOUT USING VECTOR 2D FROM STUYLIB
    
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

    public Rotation2d getBalanceAngle() {
        Rotation2d yaw = getGyroYaw(), pitch = getGyroPitch(), roll = getGyroRoll();
        
        double facingSlope = pitch.getTan() * yaw.getCos() + roll.getTan() * yaw.getSin();
        double maxSlope = Math.sqrt(Math.pow(roll.getTan(), 2) + Math.pow(pitch.getTan(), 2));

        SmartDashboard.putNumber("Swerve/Facing Slope", facingSlope);
        SmartDashboard.putNumber("Swerve/Max Slope", maxSlope);

        return Rotation2d.fromRadians(Math.signum(facingSlope) * Math.atan(maxSlope));
    }

    public void setXMode() {
        setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromDegrees(135)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(225)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(315)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        });
    } 

    @Override
    public void periodic() {
         // need to complete writing this without using StuyLib
    }
}
