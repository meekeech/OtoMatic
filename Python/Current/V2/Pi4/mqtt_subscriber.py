import paho.mqtt.client as mqtt
 
PI4_SERVER = "localhost"
PI4_PATH = [("piz1-pi4",0),("piz2-pi4",0)]
#m = ''

 
# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
 
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(PI4_PATH)
 
# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    if (msg.topic == "piz1-pi4"):
        f = open('output1.jpg',"wb")
        f.write(msg.payload)
        print("Image1 received")
        f.close()
    elif (msg.topic == "piz2-pi4"):
        f2 = open('output2.jpg',"wb")
        f2.write(msg.payload)
        print("Image2 received")
        f2.close()
    

 
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
 
client.connect_async(PI4_SERVER, 1883, 60)
 
# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
client.loop_start()

while True:
    pass
    #if m is 'x':
        #break
client.loop_stop()
