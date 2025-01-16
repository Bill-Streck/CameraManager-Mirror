import socket
import asyncio
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

BROADCAST_ADDR = '255.255.255.255'

port = 12345

send_message = b'Hello, network!'

# Create a UDP socket
socklisten = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 

# Bind the socket to the port and any IP address
socklisten.bind(('', port))

async def listen():
    while True:
        # Receive a message
        message, address = socklisten.recvfrom(1024)
        print(f"Received message from {address}: {message.decode('utf-8')}")
        if message == send_message:
            break
        await asyncio.sleep(0)

async def send():
    print("trying")
    sock.sendto(send_message, (BROADCAST_ADDR, port))
    print("sent message")

async def main():
    # send finishes immediately, then listen gets to run
    await asyncio.gather(send(), listen())


start_t = time.time()
asyncio.run(main())
elapse = time.time() - start_t
print(f"elapsed: {elapse}")

sock.close()
socklisten.close()
