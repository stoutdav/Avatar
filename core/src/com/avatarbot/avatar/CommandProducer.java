package com.avatarbot.avatar;

public interface CommandProducer {
	public Command getCommand() throws InterruptedException;
}
