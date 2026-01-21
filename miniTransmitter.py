## "Mini" Transmitter python script for transmitting data via strings
#  Primary goal is to establish foothold of Sockets between Linux ran Kernels to send and recieve data for messages,
# mainly used for devices and machines with no screen onboard, must have active IP's for this to work,
# This python script pairs with 'miniReciever.py' so the reciever can use its screen to display data or wait for button presses to send to the transmitter

#This code is supposed to be the Transmitter, which we will roleplay as the "device" we wish to send data to a "reciever" with a screen

#VERSION 0.01
#	Release. edit/patch
##Jayven A Aguirre Jan 21, 2026
##Mini Test Assisted using DeepSeek generative AI (mainly used for establishing sockets)
import socket
import time


# Define color codes (For ansi, I like to use colors to highlight events)
RED = '\033[31m'
GREEN = '\033[32m' 
CYAN = '\033[36m'
RESET = '\033[0m'

def connect_to_receiver(host='no socket, please insert', port= 8888):
	print("Attempting Connection to {host}:{port}")

	client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
	#attempts to create established connection, try means it works, should show green
	try:
		client_socket.connect(host,port)
		print(GREEN+"Connection Established to Machine"+RESET)
		return client_socket
	#Connection did not go through!
	except ConnectionRefusedError:
		print(RED+"ERROR: Reciever not running or incorrect IP/Port!!!"+RESET)
		return None
	except Exception as e:
		print(RED+"CONNECTION ERROR: {e}") 
		return None
##END of connect_to_reciever






#MAIN LOOP, Code runs until 1 side quits, should close the socket!
def main():
	#make this a config file that is encrypted please, other than that for doc please keep blank
	RECEIVER_IP = '100.100.1.067' ##Dummy Value, change b4 you use
	PORT = 8888 ##selected port, please explore better ports when you can

	#attempt to connect to reciever
	##classic function implementation here
	sock = connect_to_receiver(RECEIVER_IP,PORT)
	if not sock:
		return

	print(RESET+"\nType messages to send or 'quit' to exit program")
	print("This program has special key phrases an established socket can react to, please consult manual/documentation")

	#Main Repeating loop, does this until you ask it to stop
	try:
		while True:
			#classic input here!
			message = input("Message to send: ").strip()

			if message.lower() == 'quit':
				print("Closing connection")
				break

			#no message no problem!
			if not message:
				continue

			#send da message
			sock.sendall(message+'\n')
			sock.settimeout(2.0) #2 second timeout here,
			try:
				response = sock.recv(1024).decode().strip()
				if response:
					print("Response: {response}")
			except socket.timeout:
				print(CYAN+"No response recieved..."+RESET)
				pass
			#resets timeout for da next message
			sock.settimeout(None)

	except KeyboardInterrupt:
		print("\n\nProcess Intterupted by user")
	except Exception as e:
		print(RED+"PROCESS ERROR: {e}"+RESET)
	finally:
		sock.close()
		print(GREEN+"Socket network closed, exiting"+RESET)

if __name__=="__main__":
	main()
