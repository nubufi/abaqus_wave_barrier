import json
from model import Input

def read_json(file_name)->Input:
    with open(file_name) as f:
        data = json.load(f)
    
    return Input(**data)