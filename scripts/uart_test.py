import serial
import struct
import time

sbus_port=serial.Serial(
    port='/dev/ttyAMA2',
    baudrate=100000,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_EVEN,
    stopbits=serial.STOPBITS_TWO,
    timeout=0.01
)

def read_sbus_packet():
    while True:
        if sbus_port.read() == b'\x0F':
            packet = sbus_port.read(24)
            if len(packet) == 24:
                return b'\x0F' + packet

def decode_sbus_packet(packet):
    channels = []

    channels.append((packet[1] | packet[2] << 8) & 0x07FF)
    channels.append((packet[2] >> 3 | packet[2] << 5) & 0x07FF)
    channels.append((packet[3] >> 6 | packet[4] << 2 | packet[5] << 10) & 0x07FF)
    channels.append((packet[5] >> 1 | packet[6] << 7) & 0x07FF)
    channels.append((packet[6] >> 4 | packet[7] << 4) & 0x07FF)
    channels.append((packet[7] >> 7 | packet[8] << 1 | packet[9] << 9) & 0x07FF)
    channels.append((packet[9] >> 2 | packet[10] << 6) & 0x07FF)
    channels.append((packet[10] >> 5 | packet[11] << 3) & 0x07FF)
    channels.append((packet[12] | packet[13] << 8) & 0x07FF)
    channels.append((packet[13] >> 3 | packet[14] << 5) & 0x07FF)
    channels.append((packet[14] >> 6 | packet[15] << 2 | packet[16] << 10) & 0x07FF)
    channels.append((packet[16] >> 1 | packet[17] << 7) & 0x07FF)
    channels.append((packet[17] >> 4 | packet[18] << 4) & 0x07FF)
    channels.append((packet[18] >> 7 | packet[19] << 1 | packet[20] << 9) & 0x07FF)
    channels.append((packet[20] >> 2 | packet[21] << 6) & 0x07FF)
    channels.append((packet[21] >> 5 | packet[22] << 3) & 0x07FF)

    channel17 = packet[23] & 0x01
    channel18 = packet[23] & 0x02
    frameLost = packet[23] & 0x04
    failsafe = packet[23] & 0x08

    return channels, channel17, channel18, frameLost, failsafe

def main():
    while True:
        packet = read_sbus_packet()
        if packet:
            channels, channel17, channel18, frameLost, failsafe = decode_sbus_packet(packet)
            print(channels)

if __name__ == "__main__":
    main()
