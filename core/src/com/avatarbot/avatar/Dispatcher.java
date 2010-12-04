package com.avatarbot.avatar;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.lang.reflect.Constructor;
import java.util.Properties;

public class Dispatcher {
	
	private static final String CONFIG_FILENAME_PROPERTY = "config";
	private static final String CONFIG_FILENAME_DEFAULT = "conf/avatar.conf";
	private static final String CONFIG_PRODUCER_PREFIX = "producer.";
	private static final String CONFIG_CONSUMER_PREFIX = "consumer.";
	private static final String CONFIG_CLASS_SUFFIX = "class";
	
	private CommandProducer producer;
	private CommandConsumer consumer;
	
	/**
	 * Constructs a command dispatcher.
	 * @param producer The command producer.
	 * @param consumer The command consumer.
	 */
	public Dispatcher(CommandProducer producer, CommandConsumer consumer) {
		this.producer = producer;
		this.consumer = consumer;
	}
	
	/**
	 * Dispatches commands between a producer and a consumer.
	 * @throws InterruptedException 
	 */
	public void dispatch() throws InterruptedException {
		while (true) {
			consumer.handleCommand(producer.getCommand());
		}
	}
	
	public static void main(String[] args) throws Exception {
		try {
			// Load the configuration file
			String fileName = System.getProperty(CONFIG_FILENAME_PROPERTY, CONFIG_FILENAME_DEFAULT);
			Properties config = new Properties();
			config.load(new BufferedInputStream(new FileInputStream(fileName)));
			
			// Instantiate the command producer and consumer
			CommandProducer producer = instantiate(config, CONFIG_PRODUCER_PREFIX);
			CommandConsumer consumer = instantiate(config, CONFIG_CONSUMER_PREFIX);
			
			// Get to work
			Dispatcher dispatcher = new Dispatcher(producer, consumer);
			dispatcher.dispatch();
		}
		catch (FileNotFoundException e) {
			System.err.println("FATAL: Unable to locate the configuration file!");
			System.err.println("Configuration should be in " + CONFIG_FILENAME_DEFAULT +
				" or specified using -D" + CONFIG_FILENAME_PROPERTY + "=<filename>");
		}
	}

	/**
	 * Utility method to instantiate an object and, optionally, configure it.
	 * @param <T> The type of object to instantiate.
	 * @param config The configuration.
	 * @param prefix The prefix to use in the configuration.
	 * @return The object.
	 * @throws Exception If something goes wrong.
	 */
	@SuppressWarnings("unchecked")
	private static <T> T instantiate(Properties config, String prefix) throws Exception {
		// Load the implementation class
		String className = config.getProperty(prefix + CONFIG_CLASS_SUFFIX);
		Class<?> clazz = Class.forName(className);
		
		// Find the most relevant constructor. We hunt for a constructor that takes
		// a Properties and a String. If we don't find it, we use the empty constructor
		// instead.
		Constructor<T> emptyConstructor = null;
		T instance = null;
		for (Constructor<T> constructor : (Constructor<T>[]) clazz.getConstructors()) {
			Class<?>[] paramTypes = constructor.getParameterTypes();
			if (paramTypes.length == 0) {
				emptyConstructor = constructor;
			} else {
				if (paramTypes.length == 2 && paramTypes[0] == Properties.class && paramTypes[1] == String.class) {
					// Instantiate the object using the configuration-aware constructor
					instance = constructor.newInstance(config, prefix);
					break;
				}
			}
		}
		if (instance == null && emptyConstructor != null) {
			// Instantiate the object using the empty constructor
			instance = emptyConstructor.newInstance();
		}
		return instance;
	}
	
}
