import socket
import os
import threading
import json

MAIN_SOCKET = "/tmp/my_socket"
NUM_WORKERS = 8


def forward_to_main(data):

    try:
        client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        client.connect(MAIN_SOCKET)

        client.sendall(data)
        resp = client.recv(4096)

        client.close()

        if not resp:
            return "ERROR"

        return resp.decode()

    except Exception as e:
        print("Forward error:", e)
        return "ERROR"


def worker(socket_path):

    if os.path.exists(socket_path):
        os.remove(socket_path)

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(socket_path)
    server.listen()

    print("Worker listening:", socket_path)

    while True:

        conn, _ = server.accept()

        try:
            data = conn.recv(4096)

            if not data:
                conn.close()
                continue

            print(socket_path, "received:", data.decode().strip()[:145])

            response = forward_to_main(data)

            conn.sendall(response.encode())

        except Exception as e:
            print("Worker error:", e)

        finally:
            conn.close()


for i in range(NUM_WORKERS):
    path = f"/tmp/my_socket{i}"
    threading.Thread(target=worker, args=(path,), daemon=True).start()

print("Workers running")
input("Press Enter to exit\n")
