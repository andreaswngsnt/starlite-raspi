import socket
import threading

# Server configuration
HOST = '0.0.0.0'  # Listen on all available network interfaces
PORT = 12345      # Port to listen on
USERS = {'user1': 'password1', 'user2': 'password2'}  # Set the username and password

def authenticate(username, password):
    return USERS.get(username) == password

def handle_client(client_socket):
    client_address = client_socket.getpeername()
    print("Connection established with", client_address)

    try:
        while True:
            username = client_socket.recv(1024).decode('utf-8')
            password = client_socket.recv(1024).decode('utf-8')

            if not username or not password:
                break  # Empty credentials, disconnect the client

            if authenticate(username, password):
                client_socket.send(b"Correct")
            else:
                client_socket.send(b"Wrong")
    except Exception as e:
        print("Error:", e)
    finally:
        client_socket.close()
        print("Connection closed with", client_address)

def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen(5)

    print("Server is listening on...")
    print("IP Address: %s" % HOST)
    print("Port: %d" % PORT)

    while True:
        client_socket, _ = server_socket.accept()
        client_thread = threading.Thread(target=handle_client, args=(client_socket,))
        client_thread.start()

if __name__ == "__main__":
    start_server()
