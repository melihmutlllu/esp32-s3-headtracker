import socket
import struct
import time


PORT = 4242
PACKET = struct.Struct("<6d")


def main() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", PORT))
    sock.settimeout(1.0)

    print(f"Listening for OpenTrack UDP pose packets on 0.0.0.0:{PORT}")
    last_print = 0.0
    packets = 0

    while True:
        try:
            data, addr = sock.recvfrom(256)
        except socket.timeout:
            continue

        if len(data) != PACKET.size:
            print(f"{addr[0]}:{addr[1]} invalid packet size={len(data)}")
            continue

        tx, ty, tz, yaw, pitch, roll = PACKET.unpack(data)
        packets += 1
        now = time.monotonic()

        if now - last_print >= 0.2:
            last_print = now
            print(
                f"{addr[0]}:{addr[1]} packets={packets} "
                f"yaw={yaw:8.2f} pitch={pitch:8.2f} roll={roll:8.2f}"
            )


if __name__ == "__main__":
    main()
