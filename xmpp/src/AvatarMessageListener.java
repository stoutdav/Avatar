import org.jivesoftware.smack.Chat;
import org.jivesoftware.smack.MessageListener;
import org.jivesoftware.smack.XMPPException;
import org.jivesoftware.smack.packet.Message;

import java.io.IOException;

/**
 *
 */
public class AvatarMessageListener implements MessageListener {
    private AvatarSerialInterface serialInterface;
    private Chat chat;

    public void processMessage(Chat chat, Message message) {
        Message response = new Message();
        try {
            String messageBody = message.getBody();
            response.setBody("You said: " + messageBody);
            chat.sendMessage(response);
            serialInterface.getOutput().write(messageBody.getBytes());
        } catch (XMPPException ex) {
            ex.printStackTrace();
        } catch (IOException e) { //To change body of catch statement use File | Settings | File Templates.
        }
    }

    public void processSerialEvent(String arduinoResponse) {
        Message response = new Message();
        try {
            response.setBody("Arduino Said: " + arduinoResponse);
            chat.sendMessage(response);
        } catch (XMPPException ex) {
            ex.printStackTrace();
        }
    }

    public void setChat(Chat chat) {
        this.chat = chat;
    }

    public void setSerialInterface(AvatarSerialInterface serialInterface) {
        this.serialInterface = serialInterface;
    }
}