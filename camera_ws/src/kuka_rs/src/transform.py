import struct

def binary(num):
    packed = struct.pack('!f', num)
    integers = [ord(c) for c in packed]
    return integers

#def equat_line(point1, point2):
