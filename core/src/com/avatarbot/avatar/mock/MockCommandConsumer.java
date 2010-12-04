package com.avatarbot.avatar.mock;

import com.avatarbot.avatar.Command;
import com.avatarbot.avatar.CommandConsumer;

public class MockCommandConsumer implements CommandConsumer {

	@Override
	public void handleCommand(Command command) {
		synchronized (System.out) {
			System.out.printf("[MockConsumer] Received: %s\n", command);
		}
	}

}
