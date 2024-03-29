import socket
import struct

def main():
    # Set up the UDP socket
    UDP_IP = "0.0.0.0"  # Listen on all interfaces
    UDP_PORT = 12345    # Replace with your port number

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    print("Listening on port:", UDP_PORT)

    try:
        while True:
            data, addr = sock.recvfrom(1024)  # buffer size

            # Unpack the data according to the structure
            unpacked_data = struct.unpack('q d fff ffff i', data)

            serialNumber = unpacked_data[0]
            timestamp = unpacked_data[1]
            position = unpacked_data[2:5]
            quaternion = unpacked_data[5:9]
            toolId = unpacked_data[9]

            print(f"Received from {addr}:")
            print(f"Serial number: {serialNumber}")
            print(f"Timestamp: {timestamp}")
            print(f"Position: {position}")
            print(f"Quaternion: {quaternion}")
            print(f"Tool ID: {toolId}")

    except KeyboardInterrupt:
        print("\nInterrupt received, stopping...")
    finally:
        sock.close()
        print("Socket closed.")

if __name__ == "__main__":
    main()
