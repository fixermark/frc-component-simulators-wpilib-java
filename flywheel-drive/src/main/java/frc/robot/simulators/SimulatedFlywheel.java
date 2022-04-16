package frc.robot.simulators;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A flywheel controlled by one motor
 */
public class SimulatedFlywheel extends SubsystemBase {
    /**
     * Designed to run in TimedRobot, where periodic time-stamp is fixed
     */
    private static final double TIMESTEP_SECS = 0.02;

    private static final double GRAVITATIONAL_ACCELERATION_M_PER_SEC_SQUARED = 9.8;

    private MotorController m_motor;
    private EncoderSim m_encoder;

    private NetworkTableEntry m_motorStallTorqueNM;
    private NetworkTableEntry m_motorFreeSpinRPM;

    private NetworkTableEntry m_flywheelMassKg;
    private NetworkTableEntry m_flywheelMomentOfInertiaKgMSquared;
    private NetworkTableEntry m_flywheelStaticFrictionConstant;
    private NetworkTableEntry m_flywheelDynamicFrictionConstant;
    private NetworkTableEntry m_motorToFlywheelGearRatio;

    private NetworkTableEntry m_flywheelSpeedRpm;

    private double m_motorSpeedRpm = 0;

    public SimulatedFlywheel(MotorController motor, EncoderSim encoder) {
        m_motor = motor;
        m_encoder = encoder;

        // Default values are for a REV Robotics NEO brushless motor
        // https://www.revrobotics.com/rev-21-1650/
        var motorCharacteristics = Shuffleboard.getTab("Flywheel").getLayout("Motor", BuiltInLayouts.kList)
        .withSize(2,3);
        m_motorStallTorqueNM = motorCharacteristics.add("Motor stall torque (Newton-meters)", 2.6).getEntry();
        m_motorFreeSpinRPM = motorCharacteristics.add("Motor unloaded free spin (RPM)", 5676.0).getEntry();

        // Default values are for an AndyMark SmoothGrip wheel (6" diameter)
        // https://www.andymark.com/products/6-in-smoothgrip-wheel-1
        // moment of inertia calculated at http://hyperphysics.phy-astr.gsu.edu/hbase/icyl.html
        // by assuming the SmoothGrip wheel is a perfect 3" radius cylinder of uniform density
        // (not true, but close enough for simulation fun)
        var flywheelCharacteristics = Shuffleboard.getTab("Flywheel").getLayout("Flywheel", BuiltInLayouts.kList)
        .withPosition(2,0)
        .withSize(2,3);
        m_flywheelMassKg = flywheelCharacteristics.add("Mass (kg)", 0.23).getEntry();
        m_flywheelMomentOfInertiaKgMSquared = flywheelCharacteristics.add("Moment of inertia (kg * m^2)", 0.007548372000000001).getEntry();
        m_flywheelStaticFrictionConstant = flywheelCharacteristics.add("Static friction constant", 0.6).getEntry();
        m_flywheelDynamicFrictionConstant = flywheelCharacteristics.add("Dynamic friction constant", 0.29).getEntry();
        m_motorToFlywheelGearRatio = flywheelCharacteristics.add("Motor-to-flywheel gear ratio", 1.0).getEntry();

        m_flywheelSpeedRpm = Shuffleboard.getTab("Flywheel").add("Flywheel speed (RPM)", 1.0)
        .withPosition(0,3)
        .withSize(3,3)
        .withWidget(BuiltInWidgets.kGraph)
        .getEntry();
    }

    @Override
    public void periodic() {
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

        var gearRatio = m_motorToFlywheelGearRatio.getDouble(1.0);

        var advantagedTorque = motorTorque * gearRatio;

        var frictionCoefficient = m_motorSpeedRpm == 0 ? 
          m_flywheelStaticFrictionConstant.getDouble(1.0) :
          m_flywheelDynamicFrictionConstant.getDouble(1.0);

        var counterTorque = Math.signum(m_motorSpeedRpm) * m_flywheelMassKg.getDouble(1.0) * GRAVITATIONAL_ACCELERATION_M_PER_SEC_SQUARED * frictionCoefficient;

        var totalTorque = advantagedTorque - counterTorque;
        if (m_motorSpeedRpm == 0 && Math.abs(advantagedTorque) < Math.abs(counterTorque)) {
            // stalled motor does not overcome static friction
            totalTorque = 0;
        }

        var angularAccelerationRadsPerSec = totalTorque / m_flywheelMomentOfInertiaKgMSquared.getDouble(1.0);
        var flywheelAngularVelocityRps = Units.rotationsPerMinuteToRadiansPerSecond(m_motorSpeedRpm) / gearRatio;

        // Update angular velocity, parceling the change by the delta-T
        var newAngularVelocityRps = flywheelAngularVelocityRps + angularAccelerationRadsPerSec * TIMESTEP_SECS;

        if (Math.signum(newAngularVelocityRps) != Math.signum(flywheelAngularVelocityRps) && flywheelAngularVelocityRps != 0 && newAngularVelocityRps != 0) {
            // Crossed zero boundary; if torque is too low to overcome friction, motor seizes
            if (Math.abs(advantagedTorque) < Math.abs(counterTorque)) {
                newAngularVelocityRps = 0;
            }
        }

        m_motorSpeedRpm = Units.radiansPerSecondToRotationsPerMinute(newAngularVelocityRps * gearRatio);

        // multiply by timestep and ratio of rotations to radians to get new flywheel position
        var flywheelSpeedRotationsPerSec = newAngularVelocityRps / (2 * Math.PI);
        var flywheelDelta = flywheelSpeedRotationsPerSec * TIMESTEP_SECS;

        // encoder values are relative to flywheel, not motor
        m_encoder.setDistance(m_encoder.getDistance() + flywheelDelta);
        m_encoder.setRate(newAngularVelocityRps);

        m_flywheelSpeedRpm.setDouble(flywheelSpeedRotationsPerSec * 60);
    }
}
