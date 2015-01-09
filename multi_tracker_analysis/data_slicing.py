
def get_keys_with_attributes(data, attributes, values):
    if type(attributes) is not list:
        attributes = [attributes]
        values = [values]
    
    keys = []
    for key, trajec in data.items():
        key_ok = True
        for a, attribute in enumerate(attributes):
            if trajec.__getattribute__(attribute) == values[a]:
                pass
            else:
                key_ok = False
                break
        if key_ok:
            keys.append(key)
        
    return keys
        
def get_keys_of_length_greater_than(data, length):
    keys = []
    for key, trajec in data.items():
        if trajec.length > length:
            keys.append(key)
    return keys
