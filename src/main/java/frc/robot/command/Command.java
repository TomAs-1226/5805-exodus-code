package frc.robot.command;

import java.util.function.Function;

/**
 * command interface; it runs, it does things (occasionally)
 * @author florpy
 */
@FunctionalInterface
public interface Command {

    /**
     * executes this command
     * @param sys a reference to the command system executing this command
     * @return result log of this command
     * @implSpec the {@link CommandResult} returned by this method should describe what this command did
     */
    CommandResult execute(CommandSystem sys);

    /**
     * returns a composition of this command and another command if the specified condition is met
     * @param condition a function which defines whether the specified command should execute after or not
     * @param command command to run if the condition is matched
     * @return composed commands
     */
    default Command andIf(Function<CommandResult, Boolean> condition, Command command) {
        Command self = this;
        return sys -> {
            CommandResult result = self.execute(sys);
            if (condition.apply(result)) {
                result = command.execute(sys);
                return result;
            }
            return result;
        };
    }

    /**
     * returns a composition of this command and another command if the specified condition is met or not met
     * @param condition a function which defines whether the specified command should execute after or not
     * @param cmdIf command to run if the condition is matched
     * @param cmdElse command to run if the condition is not matched
     * @return composed commands
     */
    default Command andIfElse(Function<CommandResult, Boolean> condition, Command cmdIf, Command cmdElse) {
        Command self = this;
        return sys -> {
            CommandResult result = self.execute(sys);
            if (condition.apply(result)) {
                result = cmdIf.execute(sys);
            } else {
                result = cmdElse.execute(sys);
            }
            return result;
        };
    }

}
