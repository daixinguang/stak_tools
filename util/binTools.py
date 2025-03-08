import struct
import binascii

def unpackUint8(data):
    res_tuple = struct.unpack('B', data)
    return res_tuple[0]

def unpackUint16(data):
    res_tuple = struct.unpack('H', data)
    return res_tuple[0]

def unpackUint32(data):
    res_tuple = struct.unpack('I', data)
    return res_tuple[0]

def unpackUint64(data):
    res_tuple = struct.unpack('Q', data)
    return res_tuple[0]

def unpackInt32(data):
    res_tuple =  struct.unpack('i', data)
    return res_tuple[0]

def unpackInt16(data):
    res_tuple = struct.unpack('h', data)
    return res_tuple[0]

def unpackInt8(data):
    res_tuple = struct.unpack('b', data)
    return res_tuple[0]

def unpackFloat(data):
    res_tuple = struct.unpack('f', data)
    return res_tuple[0]

def unpackChar(data):
    res = binascii.hexlify(data)
    return res