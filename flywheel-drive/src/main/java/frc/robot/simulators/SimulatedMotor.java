package frc.robot.simulators;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimulatedMotor extends SubsystemBase {
    MotorController m_motor;

    private NetworkTableEntry m_motorStallTorqueNM;
    private NetworkTableEntry m_motorFreeSpinRPM;

    double m_motorSpeedRpm = 0;

    /**
     * Construct simulated motor and its connections to
     * Shuffleboard 
     * @param motor The motor controller that controls this motor
     * @param tab The Shuffleboard tab on which the motor's controls should appear
     * @param name Name of this motor in the Shuffleboard UI
     * @param positionX The X position of the motor controller in the Shuffleboard UI
     * @param positionY The Y position of the motor controller in the Shuffleboard UI
     */
    SimulatedMotor(
        MotorController motor,
        ShuffleboardTab tab,
        String name,
        int positionX,
        int positionY
    ) {
        m_motor = motor;
        // Default values are for a REV Robotics NEO brushless motor
        // https://www.revrobotics.com/rev-21-1650/
        var motorCharacteristics = tab.getLayout("Motor", BuiltInLayouts.kList)
        .withPosition(positionX, positionY)
        .withSize(2,3);
        m_motorStallTorqueNM = motorCharacteristics.add("Motor stall torque (Newton-meters)", 2.6).getEntry();
        m_motorFreeSpinRPM = motorCharacteristics.add("Motor unloaded free spin (RPM)", 5676.0).getEntry();
    }

    /**
     * Get torque produced by the motor
     * @return The torque, in newton-meters
     */
    public double getMotorTorque() {
        // Motor characteristic caclculations based on https://pages.mtu.edu/~wjendres/ProductRealization1Course/DC_Motor_Calculations.pdf
        
        // Calculate back-EMF constant K_c and Q (resistance / K_m)
        var k_c = 12.0 / Units.rotationsPerMinuteToRadiansPerSecond(m_motorFreeSpinRPM.getDouble(1.0)); 
        var q = 12.0 / m_motorStallTorqueNM.getDouble(1.0);

        // Calculate torque applied by motor
        var motorVoltage = Math.max(-1.0, Math.min(1.0, m_motor.get())) * RobotController.getBatteryVoltage();
        var motorSpeedRadsS = Units.rotationsPerMinuteToRadiansPerSecond(m_motorSpeedRpm);
        var motorTorque = (motorVoltage - motorSpeedRadsS * k_c) / q;
        
        // Simulate motor controller operating in "coast mode..." if motor voltage is 0, controller opens the circuit and
        // so current is 0, leading to no motor torque
        if (motorVoltage == 0) {
            motorTorque = 0;
        }
        return motorTorque;
    }

    /**
     * Set speed of motor. This is needed for proper back-EMF calculation
     * 
     * @param rpm Motor speed, In revolutions per minute
     */
    public void setMotorSpeedRpm(double rpm) {
        m_motorSpeedRpm = rpm;
    }

    /**
     * Get the speed of the motor
     * 
     * @return Motor speed, in revolutions per minute
     */
    public double getMotorSpeedRpm() {
        return m_motorSpeedRpm;
    }
}
