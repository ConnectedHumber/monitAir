# borrowed from cayenne#
import time
from ssl import PROTOCOL_TLSv1_2
import paho.mqtt.client as mqtt
import json

# Topics
COMMAND_TOPIC = "cmd"
DATA_TOPIC = "data"
RESPONSE_TOPIC = "response"


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, monitair, flags, rc):
    if rc != 0:
        # MQTT broker error codes
        broker_errors = {
            1 : 'unacceptable protocol version',
            2 : 'identifier rejected',
            3 : 'server unavailable',
            4 : 'bad user name or password',
            5 : 'not authorized',
        }
        error = "Connection failed, " + broker_errors.get(rc, "result code " + str(rc))
        raise Exception(error)
    else:
        print("Connected with result code "+str(rc))
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        monitair.connected = True
        monitair.reconnect = False
        command_topic = monitair.getCommandTopic();
        print("SUB %s\n" % command_topic)
        client.subscribe(command_topic)
        monitair.mqttPublish("%s/sys/model" % monitair.rootTopic, "Python")
        monitair.mqttPublish("%s/sys/version" % monitair.rootTopic, __version__)

# The callback for when the client disconnects from the server.
def on_disconnect(client, monitair, rc):
    print("Disconnected with result code "+str(rc))
    monitair.connected = False
    monitair.reconnect = True
    
# The callback for when a PUBLISH message is received from the server.
def on_message(client, monitair, msg):
    print(msg.topic+" "+str(msg.payload))
    if monitair.on_message:
        message = monitairMessage(msg)
        error = monitair.on_message(message)
        if not error:
            # If there was no error, we send the new channel state, which should be the command value we received.
            monitair.virtualWrite(message.channel, message.value)
        # Send a response showing we received the message, along with any error from processing it.
        monitair.responseWrite(message.msg_id, error)
        
class monitairMessage:
    """ This is a class that describes an incoming monitair message. It is
    passed to the on_message callback as the message parameter.
    Members:
    client_id : String. Client ID that the message was published on.
    topic : String. Topic that the message was published on.
    channel : Int. Channel that the message was published on.
    msg_id : String. The message ID.
    value : String. The message value.
    """
    def __init__(self, msg):
        topic_tokens = msg.topic.split('/')
        self.client_id = topic_tokens[3]
        self.topic = topic_tokens[4]
        self.channel = int(topic_tokens[5])
        if msg.payload is str:
            payload_tokens = msg.payload.split(',')
        else:
            payload_tokens = msg.payload.decode().split(',')
        self.msg_id = payload_tokens[0]
        self.value = payload_tokens[1]
        
    def __repr__(self):
        return str(self.__dict__)
        
class monitairMQTTClient:
    """monitair MQTT Client class.
    
    This is the main client class for connecting to monitair and sending and receiving data.
    
    Standard usage:
    * Set on_message callback, if you are receiving data.
    * Connect to monitair using the begin() function.
    * Call loop() at intervals (or loop_forever() once) to perform message processing.
    * Send data to monitair using write functions: virtualWrite(), celsiusWrite(), etc.
    * Receive and process data from monitair in the on_message callback.
    The on_message callback can be used by creating a function and assigning it to monitairMQTTClient.on_message member.
    The callback function should have the following signature: on_message(message)
    The message variable passed to the callback is an instance of the monitairMessage class.
    """
    client = None
    rootTopic = ""
    connected = False
    reconnect = False
    on_message = None
    
    def begin(self, username, password, clientid, hostname, port=1883):
        """Initializes the client and connects to monitair.
        
        username is the monitair username.
        password is the monitair password.
        clientID is the monitair client ID for the device.
        hostname is the MQTT broker hostname.
        port is the MQTT broker port. Use port 8883 for secure connections.
        """
        self.rootTopic = "v1/%s/things/%s" % (username, clientid)
        self.client = mqtt.Client(client_id=clientid, clean_session=True, userdata=self)
        self.client.on_connect = on_connect
        self.client.on_disconnect = on_disconnect
        self.client.on_message = on_message
        self.client.username_pw_set(username, password)
        if port == 8883:
            self.client.tls_set(tls_version=PROTOCOL_TLSv1_2)
        self.client.connect(hostname, port, 60)        
        print("Connecting to {}:{}".format(hostname, port))

    def loop(self):
        """Process monitair messages.
        
        This should be called regularly to ensure monitair messages are sent and received.
        """
        self.client.loop()
        if not self.connected and self.reconnect:
            try:
                self.client.reconnect()
                self.reconnect = False
            except:
                print("Reconnect failed, retrying")
                time.sleep(5)
    
    def loop_forever(self):
        """Process monitair messages in a blocking loop that runs forever."""
        self.client.loop_forever()
    
    def getDataTopic(self, channel):
        """Get the data topic string.
        
        channel is the channel to send data to.
        """
        return "%s/%s/%s" % (self.rootTopic, DATA_TOPIC, channel)
    
    def getCommandTopic(self):
        """Get the command topic string."""
        return "%s/%s/+" % (self.rootTopic, COMMAND_TOPIC)

    def getResponseTopic(self):
        """Get the response topic string."""
        return "%s/%s" % (self.rootTopic, RESPONSE_TOPIC)

    def virtualWrite(self, channel, value, dataType="", dataUnit=""):
        """Send data to monitair.
        
        channel is the monitair channel to use.
        value is the data value to send.
        dataType is the type of data.
        dataUnit is the unit of the data.
        """
        if (self.connected):
            topic = self.getDataTopic(channel)
            if dataType:
                payload = "%s,%s=%s" % (dataType, dataUnit, value)
            else:
                payload = value
            self.mqttPublish(topic, payload)

    def jsonWrite(self, channel, msg):
        """Send data to monitair.
        
        channel is the monitair channel to use.
        msg is the payload to send.
        """
        if (self.connected):
            topic = channel
            payload = json.dumps(msg) # encode object to JSONvalue
            self.mqttPublish(topic, payload)

    def responseWrite(self, msg_id, error_message):
        """Send a command response to monitair.
        
        This should be sent when a command message has been received.
        msg_id is the ID of the message received.
        error_message is the error message to send. This should be set to None if there is no error.
        """
        if (self.connected):
            topic = self.getResponseTopic()
            if error_message:
                payload = "error,%s=%s" % (msg_id, error_message)
            else:
                payload = "ok,%s" % (msg_id)
            self.mqttPublish(topic, payload)            
            
    def mqttPublish(self, topic, payload):
        """Publish a payload to a topic
        
        topic is the topic string.
        payload is the payload data.
        """
        print("PUB %s\n%s\n" % (topic, payload))
        self.client.publish(topic, payload, 0, False) 
