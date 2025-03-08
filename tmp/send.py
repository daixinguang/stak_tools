import socket
import json
 
port = 12345

data_dict = {
    "state": "off",
    "name" : "test",
    "max_length" : 130000,
    "input_data": "null"
}

data_dict2 = {
    "state": "off",
    "name" : "test",
    "max_length" : 130000,
    "input_data": None
}
Message = json.dumps(data_dict, ensure_ascii=False)

print(len(Message))

# client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
 
# host = socket.gethostname()
 
# client.connect((host, port))
 
# # msg = json.dumps(Message)
# client.send(Message.encode('utf-8'))
 
# Result=client.recv(1024).decode()
# print(Result)
 
 
 