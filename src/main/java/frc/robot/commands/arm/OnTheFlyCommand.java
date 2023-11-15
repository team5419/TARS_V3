package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Arrays;
import java.util.Set;
import java.util.concurrent.Callable;

/**
 * This class allows you to use a command that freezes configuration on construction that you would
 * like to be deferred until initialization.
 *
 * @author G
 */
public class OnTheFlyCommand extends CommandBase {

  Callable<Command> m_commandCallable;
  Command m_command;

  public OnTheFlyCommand(Callable<Command> commandCallable, Subsystem... requirements) {
    m_commandCallable = commandCallable;
    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    try {
      m_command = m_commandCallable.call();
    } catch (Exception e) {
      System.err.println(
          "[ON THE FLY COMMAND] - Issue calling the passed in callable, aborting function. Stack trace: "
              + e);
      this.cancel();
      return;
    }

    m_command.initialize();

    Set<Subsystem> s = m_command.getRequirements();
    addRequirements(Arrays.copyOf(s.toArray(), s.size(), Subsystem[].class));
  }

  @Override
  public void execute() {
    // if(m_command == null) return;
    m_command.execute();
  }

  @Override
  public boolean isFinished() {
    // if(m_command == null) return true;
    return m_command.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (m_command == null) return;
    m_command.end(interrupted);
    m_command = null;
  }
}
