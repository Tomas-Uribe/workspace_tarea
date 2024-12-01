import cv2
import gmqtt
import asyncio
import numpy as np
import base64

from cv_bridge import CvBridge, CvBridgeError

class MQTTClient(gmqtt.Client):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.bridge = CvBridge()

    async def on_connect(self, client, flags, rc, properties):
        print("Connected with result code " + str(rc))
        await self.subscribe("/image_topic", qos=1)

    async def on_message(self, client, topic, payload, qos, properties):
        try:
            # Decode the base64 string to bytes
            img_data = base64.b64decode(payload)
            # Convert bytes to numpy array
            np_arr = np.frombuffer(img_data, np.uint8)
            # Decode numpy array to OpenCV image
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print("CvBridge Error: {0}".format(e))

        # Display the image
        cv_image = self.bridge.imgmsg_to_cv2(cv_image, "bgr8")
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(3)

async def main():
    client = MQTTClient("client_id")
    client.set_auth_credentials("username", "password")
    await client.connect("mqtt_broker_address", 1883)

    await client.subscribe("/image_topic", qos=1)
    await client.loop_forever()

if __name__ == '__main__':
    asyncio.run(main())