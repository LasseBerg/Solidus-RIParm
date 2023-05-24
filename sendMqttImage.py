import base64
import paho.mqtt.client as mqtt

client = mqtt.Client()
client.connect("192.168.50.210", 1883, 60)




with open ("thermal-image.png", "rb") as img_file:
    my_string = base64.b64encode(img_file.read())
my_string = my_string.decode("utf-8")
#print(my_string)





def on_connect(client, userdata, flags, rc):
    client.publish("solidus/arm/thermal/image", my_string)

client.on_connect = on_connect

client.loop_forever()
