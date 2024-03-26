package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Settings.Swerve.Driver.Turn;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive.Swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;



public class SwerveDriveDrive extends Command {

    private SwerveDrive swerve;

    Translation2d speed;
    double turn;

    public SwerveDriveDrive(XboxController driver) {
        swerve = SwerveDrive.getInstance();

        speed = filterSpeed(new Translation2d(driver.getLeftX(), driver.getLeftY()));
        turn = filterTurn(driver.getRightX()); 

        addRequirements(swerve);
    }

    public interface Driver {
        
        public interface Turn {
            double DEADBAND = 0.03; //"Driver Settings/Turn/Deadband"

            double RC = 0.05; //"Driver Settings/Turn/RC"
            double POWER = 2; //"Driver Settings/Turn/Power"

            double MAX_TELEOP_TURNING = 6.0; //"Driver Settings/Turn/Max Turning"
        }
    }

    @Override
    public void execute() {
        swerve.drive(speed, turn); 
    }
    
    public Translation2d filterSpeed(Translation2d speed) {
        
        SlewRateLimiter filter = new SlewRateLimiter(0.5); //limits controller inputs to 0.5units/second

        if (speed.getNorm() < Swerve.MODULE_VELOCITY_DEADBAND) {
            return new Translation2d();
        }
        else {
            return new Translation2d(filter.calculate(speed.getX()), filter.calculate(speed.getY())).times(Swerve.MAX_MODULE_SPEED);
        }        
    }

    public double filterTurn(double turn) {
        SlewRateLimiter filter = new SlewRateLimiter(0.5);

        if (Math.abs(turn) < Turn.DEADBAND) {
            return 0;
        }
        else {
            return filter.calculate(turn) * Turn.MAX_TELEOP_TURNING;
        }
    }
}

