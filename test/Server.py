import socket

# Server configuration
HOST = '127.0.0.1' # Server IP address
PORT = 12345       # Port to listen on
USERS = {'username1':'password1','username2':'password2'}  # Replace with your own user credentials

# Function to validate user login
def authenticate(client_socket):
    username = client_socket.recv(1024).decode('utf-8')
    password = client_socket.recv(1024).decode('utf-8')
    if USERS.get(username) == password:
        return True
    else:
        return False

def main():
    # Create a socket object
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Bind the socket to the address and port
    server_socket.bind((HOST, PORT))

    print("\nThe Starlite Network is initializing.")
    print("Server is listening on...")
    print("IP Address: %s" %HOST)
    print("Port: %d" %PORT)

    while True:
        # Listen for incoming connections (max 5 clients in the queue)
        server_socket.listen(5)

        # Accept a client connection
        client_socket, client_address = server_socket.accept()
        print("\nStatus Alert: A connection request has been received.")

        # Authenticate the client
        if (authenticate(client_socket) == False):
            client_socket.send("\nInvalid credentials...\nPlease try again later.\n".encode('utf-8'))
            print("Status Alert: A user's login attempt was unsucessful.")
        elif (authenticate(client_socket) == True):
            print("Status Alert: A user is logging into the server.")
            client_socket.send("Your login attempt was successful.\n\nWelcome to the Starlite Network!\n".encode('utf-8'))

            # Prompt the client for a destination address
            client_socket.send("To use a nearby Starlite delivery robot, a location must first be selected.".encode('utf-8'))
            destination = client_socket.recv(1024).decode('utf-8')
            print("Status Alert: A destination address from client has been received.")
        else:
            client_socket.send("Connection timed out.".encode('utf-8'))

if __name__ == "__main__":
    #Main program
    main()
    
    #Closing the sockets
    client_socket.close()
    server_socket.close()
