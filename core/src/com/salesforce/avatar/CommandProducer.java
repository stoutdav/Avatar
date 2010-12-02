package com.salesforce.avatar;

public interface CommandProducer {
	public Command getCommand() throws InterruptedException;
}
