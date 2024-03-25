import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.sensors.BasePigeon;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//TODO: fix this :C

public class IntakePigeonIMU extends SubsystemBase{
    private final int DEVICE_NUMBER = 0;
    private final int ACQUIRE_SPEED = 0;

    private final WPI_PigeonIMU motor;
    private final PowerDistribution powerDistribution;

    public Intake getInstance(){
        return new Intake();
    }
    
    public IntakePigeonIMU() {
        motor = new WPI_PigeonIMU(DEVICE_NUMBER);
        powerDistribution = new PowerDistribution();
    }

    public void acquire() {
        motor.set
    }

    public void deacquire() {
        motor.set;
    }

    public void stop() {
        motor.set;
    }
    
    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Intake/Speed", motor.get());
        SmartDashboard.putNumber("Intake/Current", powerDistribution.getCurrent(DEVICE_NUMBER));
    }
}
