#!/bin/bash

# Set the MQTT broker IP address
broker_ip="192.168.50.210"
topic="solidus/arm/thermal/button"

echo "MQTT SERVICE RUNNING"
# This script subscribes to a MQTT topic using mosquitto_sub.
# On each message received, you can execute whatever you want.

while true; do
    mosquitto_sub -h "$broker_ip" -t "$topic" | while read -r payload
    do
        if [ "$payload" == "thermal snap" ]; then
            # Execute the thermal_image command
            echo "taking snapshot, please wait"
            seek_snapshot --rotate 90 -o thermal-image.png
            mosquitto_pub -h "$broker_ip" -t "$topic" -m "thermal snap done"

            # Send the image as a base64 encoded string via MQTT
            image_base64=$(base64 -w 0 /home/robot/thermal-image.png)
            mosquitto_pub -h "$broker_ip" -t "solidus/arm/thermal/image" -m "$image_base64"
        fi
    done
    sleep 10  # Wait 10 seconds until reconnection
done

