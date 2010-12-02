package com.salesforce.avatar.skype;

import java.util.Properties;

import com.salesforce.avatar.Command;
import com.salesforce.avatar.CommandProducer;

public class SkypeCommandProducer extends SkypeClient implements CommandProducer {

	/**
	 * Constructs the SkypeCommandProducer and configure it.
	 * @param config The configuration.
	 * @param prefix The prefix to use in the configuration.
	 */
	public SkypeCommandProducer(Properties config, String prefix) {
		super(config, prefix);
	}
	
	@Override
	public Command getCommand() throws InterruptedException {
		Command command = null;
		do {
			// Read a datagram from the remote client
			String datagram = getDatagram();
			
			// Parse the datagram into a Command object
			try {
				Command.valueOf(datagram);
			}
			catch (Exception e) {
				System.err.println("Received invalid command '" + datagram + "'");
			}
		}
		while (command == null);
		return command;
	}

	/* TODO: Here's what we need to do here.
	 * 
	 * ON INIT
	 * 1) connect to Skype (will be handled by the superclass)
	 * 2) create an application for A2A (will be handled by the superclass)
	 * 3) start monitoring incoming calls until we detect a call
	 * 4) when a call is established initiate the A2A connection
	 * 
	 * NOTE: we can't process commands until the A2A connection is established 
	 * 
	 * ON GET COMMAND
	 * 1) wait for a datagram to come in through A2A and decode it. If the message
	 *    is a valid command, then build a command object for it and send it down.
	 */

}
