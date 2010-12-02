import org.jivesoftware.smack.ChatManager;
import org.jivesoftware.smack.XMPPConnection;
import org.jivesoftware.smack.XMPPException;
import org.jivesoftware.smack.packet.Presence;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 */
public class Avatar {

    private AvatarSerialInterface serialInterface;
    private XMPPConnection gTalkConnection;
    private static final String AVATAR_JUID = "force.avatar@gmail.com";
    private static final String AVATAR_PASSWORD = "pxJUOuk8wGFa4ty2dpV0";

    public static void main(String args[]) {
        final Avatar avatar = new Avatar();
        avatar.connect();
        while (avatar.isConnected()) {
        }
        // TODO(pantaki): Disconnect?
    }

    private void connect() {
        AvatarChatManagerListener chatManagerListener = createChatListener();
        serialInterface = createMicroControllerInterface();

        serialInterface.setMessageListener(chatManagerListener.getMessageListener());
        chatManagerListener.setSerialInterface(serialInterface);
    }

    public AvatarChatManagerListener createChatListener() {
        gTalkConnection = new XMPPConnection("gmail.com");
        try {
            gTalkConnection.connect();
        } catch (XMPPException e) {
            e.printStackTrace();
        }
        try {
            gTalkConnection.login(AVATAR_JUID, AVATAR_PASSWORD);
        } catch (XMPPException e) {
            e.printStackTrace();
        }
        ChatManager chatManager = gTalkConnection.getChatManager();
        AvatarChatManagerListener avatarChatManagerListener = new AvatarChatManagerListener();
        chatManager.addChatListener(avatarChatManagerListener);

        setPresence();
        return avatarChatManagerListener;
    }

    private AvatarSerialInterface createMicroControllerInterface() {
        AvatarSerialInterface avatarSerialInterface = new AvatarSerialInterface();
        avatarSerialInterface.initialize();
        return avatarSerialInterface;
    }

    private void disconnect() {
        gTalkConnection.disconnect();
        serialInterface.close();
    }

    private boolean isConnected() {
        return gTalkConnection.isConnected() && gTalkConnection.isAuthenticated();
    }

    private void setPresence() {
        Presence presence = new Presence(Presence.Type.available);
        presence.setStatus(getDateTime() +"Awaiting your command.");
        presence.setPriority(127);
        presence.setMode(Presence.Mode.available);
        gTalkConnection.sendPacket(presence);
    }

     private String getDateTime() {
        DateFormat dateFormat = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss");
        Date date = new Date();
        return dateFormat.format(date);
    }
}
