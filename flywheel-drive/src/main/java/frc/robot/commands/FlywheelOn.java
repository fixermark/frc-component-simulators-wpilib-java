package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;

public class FlywheelOn extends CommandBase {
    private Flywheel m_flywheel;

    public FlywheelOn(Flywheel flywheel) {
        m_flywheel = flywheel;
    }

    @Override
    public void initialize() {
        m_flywheel.turnOn();
    }

    @Override
    public void end(boolean interrupted) {
        m_flywheel.turnOff();
    }   
}
