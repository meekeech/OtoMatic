
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
 
#ZERO_SERVER = "localhost"
#ZERO_PATH = "zero_channel"
#m= ''
PI4_SERVER = "192.168.0.75"
#Topic order: sender-receiver
#topicNames: "piz1-piz2","piz1-pi4","piz2-pi4"
#Piz1: Publisher, 
    #sends piz1-piz2 and piz1-piz4
#Piz2: Publisher,Subscriber
    #sends "piz2-pi4"
    #receives "piz1-piz2"
#Pi4: Subscriber
    #receives piz1-pi4 and piz2-pi4

#MQTT_PATH = [("pi4_channel",0),("zero_channel",0)]
PIZ_PATH = "piz1-piz2"
PI4_PATH = "piz2-pi4"

 
# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
 
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(PIZ_PATH)
 
# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print("Signal Received")
    
    
    f = open("image.png","rb")
    fileContent = f.read()
    byteArr = bytearray(fileContent)
    publish.single(PI4_PATH,byteArr,hostname = PI4_SERVER)
    # more callbacks, etc
 
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
 
client.connect_async(PI4_SERVER, 1883, 60)
client.loop_start()

while True:
    pass
    

client.loop_stop()
