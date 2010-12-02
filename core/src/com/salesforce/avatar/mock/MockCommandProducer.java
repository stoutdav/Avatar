package com.salesforce.avatar.mock;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

import com.salesforce.avatar.Command;
import com.salesforce.avatar.CommandProducer;

public class MockCommandProducer implements CommandProducer {
	private static final String HELP_COMMAND = "help";
	
	private BufferedReader reader = new BufferedReader(new InputStreamReader(System.in));
	
	@Override
	public Command getCommand() {
		Command command = null;
		try {
			synchronized (System.out) {
				System.out.print("MockProducer> ");
				String input = reader.readLine();
				if (input.equals(HELP_COMMAND)) {
					System.out.println("[MockProducer] Supported commands:");
					for (Command cmd : Command.values()) {
						System.out.printf("[MockProducer] %14s - %s\n", cmd.name(), cmd.getDescription());
					}
				} else {
					command = Command.valueOf(input);
				}
			}
		}
		catch (IOException e) {
			// do nothing
		}
		return command;
	}

}
