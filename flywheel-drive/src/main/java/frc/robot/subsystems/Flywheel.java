package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.simulators.SimulatedFlywheel;

/**
 * Flywheel subsystem
 */
public class Flywheel extends SubsystemBase {
    private MotorController m_motor = new Talon(1);
    private Encoder m_encoder = new Encoder(1, 2);

    /**
     * Simulates the flywheel subsystem
     * 
     * @return The simulated flywheel
     */
    public SimulatedFlywheel simulate() {
        EncoderSim simulatedEncoder = new EncoderSim(m_encoder);
        return new SimulatedFlywheel(m_motor, simulatedEncoder);
    }

    /**
     * Activate flywheel
     */
    public void turnOn() {
        m_motor.setVoltage(12);
    }

    /**
     * Deactivate flywheel
     */
    public void turnOff() {
        m_motor.setVoltage(0);
    }
}
