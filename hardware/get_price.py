import requests
from requests.auth import HTTPBasicAuth

ROOT = "https://partdb.franks.dascheese.online"

PROJECT = "/api/projects"
ID = "/2"

APIKEY = "tcp_84b2289bf32d46756be8595c97daa18fa2636ca39a35eb53b6a46d0c38a7de5a"

auth = HTTPBasicAuth('apiKey', APIKEY)

req = requests.get(url=f"{ROOT}{PROJECT}{ID}", auth=auth)
print(req.content)