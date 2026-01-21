## Mini Reciver python script to recieve from trasmitter decices via strings
# Primary goal is to establish Sockets between Linux ran Kernels between devices that have and have no screens
# python script pairs with 'miniTransmitter.py' so it can recieve and spit back out responses based on key words.

#This code is a reciever, mainly recieves strings as commands or messages, processes them or reveals them, and then returns anything that is encoded into it.

#VERSION 0.01
#	Release. edit/patch
##Jayven A Aguirre Jan 21, 2026
##Mini Test Assisted using DeepSeek generative AI (mainly used for establishing the socket)
import socket
import time
import threading

#colors,for highlighting
RED = '\033[31m'
GREEN = '\033[32m'
YELLOW = '\033[33m'
BLUE = '\033[34m'
MAGENTA = '\033[35m'
CYAN = '\033[36m'
WHITE = '\033[37m'
RESET = '\033[0m'

KEYWORD = {
	"respond": "System Acknowledged",
	"status": "System Status: " ,
	"time" : "" ,
	"help" : "This unit can..: " ,
	"id"   : "This is a test device",
	"quit" : "Terminating Socket"
	}

def handle_client(client_socket,client_address):
	#Single Client Commns
	print(f"[+] New Connection from "+{client_address})
	#Receive protocols
	try:
		while True:
			data = client_socket.recv(1024).decode().strip()

			if not data:
				print(YELLOW+f"[{client_address}] disconnected..."+RESET)
				break
			#prints the data if we have any
			print(f"[{client_address}] >>>: {data}")

			response = ""

			if data.lower() in KEYWORD:
				#dumb variable
				keyin = data.lower()

				#if statement tree for specific commands, uses inserted value as "key in" or "keyin"

			if keyin == "time":
				import datetime
				response = f"Current Time: {datetime.datetime.now().strfime('%H:%M:%S:')}"
			elif keyin == "quit":
				response = KEYWORD[keyin]
				client_socket.sendall((response + '\n').encode())
				print("[ ! ] Quit Command Received! Terminating")
				break
			else:
				response = f"Echo: {data}"

		#any response we send it back, ez
		if response:
			client_socket.sendall((response + '\n').encode())
			print(f"[{client_address}] Sent: {response}")
	except ConnectionResetError:
		print(f"[-]{client_address} connection lost")
	except Exception as e:
		print(f"[!!!] Error woth {client_address}: {e}")
	finally:
		client_socket.close()
#end of CLIENT HANDLE

#Initializes server!
def start_server(host = '0.0.0.0' , port=8888):
	server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
	#helps reuse same address
	server_socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)

	try:
		server_socket.bind((host,port))
		server_socket.listen(3) #3 Queued connections,

		print(f"Receiver started on {host}:{port}")
		print("Waiting for connection requests")

		while True:
			client_socket,client_address = server_socket.accept()
			#code here manages 1 client per thread, try not to exceed 1
			client_thread = threading.Thread(
			target=handle_client,
			args=(client_socket,client_address)
			)
			client_thread.daemon = True
			client_thread.start()

	except KeyboardInterrupt:
		print(RED+"\n[ ! ] Server Shutting Down"+RESET)
	except Exception as e:
		print(f"[ ! ] Server Error! : {e}")
	finally:
		server_socket.close()


if __name__=="__main__":
	start_server('0.0.0.0',8888)
	#ports and addresses can be changed later



