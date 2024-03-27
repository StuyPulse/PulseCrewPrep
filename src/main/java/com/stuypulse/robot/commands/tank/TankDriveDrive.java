package com.stuypulse.robot.commands.tank;

import com.stuypulse.robot.commands.tank.TankDriveDrive.Driver.Turn;
import com.stuypulse.robot.subsystems.TankDrive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class TankDriveDrive extends Command {

    private TankDrive tank;
    private XboxController driver;

    private final SlewRateLimiter driveLimiter;
    private final SlewRateLimiter turnLimiter;

    double speed;
    double turn;
        
    public TankDriveDrive(TankDrive TankDrive, XboxController driver) {
        this.tank = TankDrive;
        this.driver = driver;

        driveLimiter = new SlewRateLimiter(0.5);
        turnLimiter = new SlewRateLimiter(0.5);

        speed = 0;
        turn = 0; 

        addRequirements(TankDrive);
    }

    public interface Drivetrain {
        // If speed is below this, use quick turn
        double BASE_TURNING_SPEED = 0.45; //Driver Settings/Base Turn Speed

        // Low Pass Filter and deadband for Driver Controls
        double SPEED_DEADBAND = 0.00; //Driver Settings/Speed Deadband
        double ANGLE_DEADBAND = 0.00; //Driver Settings/Turn Deadband

        double SPEED_FILTER = 0.125; //Driver Settings/Speed Filtering
        double ANGLE_FILTER = 0.005; //Driver Settings/Turn Filtering

        double SPEED_POWER = 2.0; //Driver Settings/Speed Power
        double ANGLE_POWER = 1.0; //Driver Settings/Turn Power

        // Width of the robot
        double TRACK_WIDTH = Units.inchesToMeters(26.9); //CHANGE
    }

    public interface Driver {
        
        public interface Turn {
            double DEADBAND = 0.03; //"Driver Settings/Turn/Deadband"

            double RC = 0.05; //"Driver Settings/Turn/RC"
            double POWER = 2; //"Driver Settings/Turn/Power"

            double MAX_TELEOP_TURNING = 6.0; //"Driver Settings/Turn/Max Turning"
        }
    }

    public interface Motion {

        DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(Drivetrain.TRACK_WIDTH);

        SimpleMotorFeedforward MOTOR_FEED_FORWARD =
                new SimpleMotorFeedforward(FeedForward.kS, FeedForward.kV, FeedForward.kA);

        double MAX_VELOCITY = 2.0;
        double MAX_ACCELERATION = 3.0;

        public interface FeedForward {
            double kS = 0.20094;
            double kV = 1.6658;
            double kA = 0.4515;
        }

        public interface PID {
            double kP = 1.0;
            double kI = 0;
            double kD = 0;
        }
    }

    public double filterSpeed(double speed) {        
        if (Math.abs(speed) < Drivetrain.SPEED_DEADBAND) {
            return 0; 
        } else {
            return pow(driveLimiter.calculate(speed), Drivetrain.SPEED_POWER);
        }  
    }

    public double filterTurn(double turn) {
        if (Math.abs(turn) < Turn.DEADBAND) {
            return 0;
        } else {
            return pow(turnLimiter.calculate(turn), Drivetrain.ANGLE_POWER);
        }
    }

    public double pow(double value, double expo){
        if (value < 0) return Math.pow(value, expo) * -1.0;
        else           return Math.pow(value, expo);
    }

    @Override
    public void execute() {
        tank.curvatureDrive(
            filterSpeed(driver.getLeftX()),
            filterTurn(driver.getRightX()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}