package com.salesforce.avatar.skype;

import java.util.Properties;

import com.salesforce.avatar.Command;
import com.salesforce.avatar.CommandConsumer;

public class SkypeCommandConsumer extends SkypeClient implements CommandConsumer {
	
	/**
	 * Constructs the SkypeCommandConsumer and configure it.
	 * @param config The configuration.
	 * @param prefix The prefix to use in the configuration.
	 */
	public SkypeCommandConsumer(Properties config, String prefix) {
		super(config, prefix);
	}

	@Override
	public void handleCommand(Command command) {
		sendCommand(command);
	}
	
	/* TODO: Here's what we need to do here.
	 * 
	 * ON INIT
	 * 1) connect to Skype (will be handled by the superclass)
	 * 2) create an application for A2A (will be handled by the superclass)
	 * 3?) start monitoring calls until we detect a call to skype.target 
	 * 4?) when a call is established with skype.target, initiate the A2A connection
	 * 
	 * NOTE: we can't process commands until the A2A connection is established
	 * NOTE: 3 & 4 could be done by the producer 
	 * 
	 * ON HANDLE COMMAND
	 * 1) relay the message to the peer client via A2A
	 */

}
