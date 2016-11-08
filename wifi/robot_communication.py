import server
import json

s = server.server()
s.start()
s.connect()


data_string = json.dumps({"test": "test"}) #data serialized


s.client.send(data_string.encode())
s.close()