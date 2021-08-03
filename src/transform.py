import struct


def binary(num):
    """
    calculate R, G, B components from bytes number
    :param num: int
    :return: list
    """
    packed = struct.pack('!f', num)
    integers = [ord(c) for c in packed]
    return integers
