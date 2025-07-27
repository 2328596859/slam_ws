import requests
import json

url = "http://117.72.41.60:8080/robot/info/status/editBySn"


payload = {
    "robotSn": "CS202504170001",
    "status": "0"
}


headers = {
    "Content-Type": "application/json"
}


response = requests.put(url, data=json.dumps(payload), headers=headers)


print("状态码:", response.status_code)
print("响应内容:", response.text)
