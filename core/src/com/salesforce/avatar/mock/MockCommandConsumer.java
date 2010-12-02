package com.salesforce.avatar.mock;

import com.salesforce.avatar.Command;
import com.salesforce.avatar.CommandConsumer;

public class MockCommandConsumer implements CommandConsumer {

	@Override
	public void handleCommand(Command command) {
		synchronized (System.out) {
			System.out.printf("[MockConsumer] Received: %s\n", command);
		}
	}

}
