package com.salesforce.avatar.skype;

import java.text.MessageFormat;
import java.util.Iterator;
import java.util.Properties;
import java.util.ServiceLoader;
import java.util.concurrent.atomic.AtomicInteger;

import com.salesforce.avatar.Command;

@SuppressWarnings("unused")
public abstract class SkypeClient {
	
	private static final String CONFIG_SKYPE_PREFIX = "skype.";
	private static final String CONFIG_NAME_SUFFIX = "name";
	private static final String CONFIG_TARGET_SUFFIX = "target";
	
	private static final String PROTOCOL_CREATE_REQUEST_FORMAT = "#{0} CREATE APPLICATION avatar";
	private static final String PROTOCOL_CREATE_RESPONSE_FORMAT = "#{0} CREATE APPLICATION avatar";
	private static final String PROTOCOL_CONNECT_REQUEST_FORMAT = "#{0} ALTER APPLICATION avatar CONNECT {1}";
	private static final String PROTOCOL_CONNECT_RESPONSE_FORMAT = "APPLICATION avatar STREAMS {0}:{1}";
	private static final String PROTOCOL_DATAGRAM_REQUEST_FORMAT = "#{0} ALTER APPLICATION avatar DATAGRAM {1}:{2} {3}";
	private static final String PROTOCOL_DATAGRAM_RESPONSE_FORMAT = "#{0} ALTER APPLICATION avatar DATAGRAM {1}:{2}={3}";
	
	private SkypePlatformIntegration integration;
	private int messageNumber;
	private String name;
	private String target;
	
	protected SkypeClient(Properties config, String prefix) {
		// Read some configuration parameters
		name = config.getProperty(CONFIG_SKYPE_PREFIX + CONFIG_NAME_SUFFIX);
		target = config.getProperty(CONFIG_SKYPE_PREFIX + CONFIG_TARGET_SUFFIX);
		
		// Instantiate the platform-specific integration
		ServiceLoader<SkypePlatformIntegration> loader = ServiceLoader.load(SkypePlatformIntegration.class);
		Iterator<SkypePlatformIntegration> iterator = loader.iterator();
		if (iterator.hasNext()) {
			integration = iterator.next();
			integration.setDelegate(new Delegate());
		} else {
			throw new RuntimeException("No skype platform integration available on the classpath");
		}
	}
	
	public void sendCommand(Command command) {	
		try {
			synchronized (integration) {
				integration.sendCommand(MessageFormat.format("#{0} {1}", messageNumber, command.name()));
				integration.wait();
			}
		} 
		catch (InterruptedException e) {
			System.out.println("SkypeClient.sendCommand() Interrupted!");
		}
	}
	
	public String getDatagram() throws InterruptedException {
		Thread.sleep(1000);
		return null;
	}
	
	/**
	 * Implementation of the platform integration callback interface.
	 */
	private class Delegate implements SkypePlatformIntegrationDelegate {

		@Override
		public String getClientApplicationName() {
			return SkypeClient.this.name;
		}

		@Override
		public void handleNotification(String notification) {
			System.out.println("DEBUG: Received notification '" + notification + "'");
		}
	}
}
