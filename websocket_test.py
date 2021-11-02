import asyncio;
import websockets;
import json
import base64
import numpy as np
import cv2

async def accept(websocket, path):
	while True:
		data = await websocket.recv()
		#print(json.dumps(data,indent=4))
		#if len(data) != 0:
		r= base64.b64decode(data) # byte 248832
		#print('s',data)
		hg_qi = np.frombuffer(r, dtype=np.uint8)
		hg_q = hg_qi.reshape((288,512,3))
		hg_r=hg_q[:, :, :3]
		print("enf : ")
		await websocket.send("ok")
		cv2.imshow("3",hg_r)
		cv2.waitKey(1)
		#else:

			#await websocket.send("no")
		#print("type : ",type(data))
		#await websocket.send("echo : ")
		
start_server = websockets.serve(accept, "localhost", 9998)
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
