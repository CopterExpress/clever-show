import socket
import logging
import platform

def str_peername(peername):
    if peername is None:
        return "None"
    return f"{peername[0]}:{peername[1]}"


def get_ip_address():  # dodo async
    """
    Returns the IP address of current computer or `localhost` if no network connection present.

    Returns:
        string: IP address of current computer or `localhost` if no network connection present
    """
    ip_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # TODO IPv6
    try:
        ip_socket.connect(("8.8.8.8", 80))
        ip = ip_socket.getsockname()[0]
    except OSError as e:
        logging.warning(f"No network connection detected, using localhost: {e}")
        ip = "localhost"
    finally:
        ip_socket.close()
    return ip


def get_ntp_time(ntp_host, ntp_port):
    """
    Gets and returns time from specified host and port of NTP server.

    Args:
        ntp_host (string): hostname or address of the NTP server.
        ntp_port (int): port of the NTP server.

    Returns:
        int: Current time recieved from the NTP server
    """
    NTP_DELTA = 2208988800  # 1970-01-01 00:00:00
    NTP_QUERY = b'\x1b' + bytes(47)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as ntp_socket:
        ntp_socket.sendto(NTP_QUERY, (ntp_host, ntp_port))
        msg, _ = ntp_socket.recvfrom(1024)
    return int.from_bytes(msg[-8:], 'big') / 2 ** 32 - NTP_DELTA


def set_keepalive(sock, after_idle_sec=1, interval_sec=3, max_fails=5):
    """
    Sets `keepalive` parameters of given socket.

    Args:
        sock (socket): Socket which parameters will be changed.
        after_idle_sec (int, optional): Start sending keepalive packets after this amount of seconds. Defaults to 1.
        interval_sec (int, optional): Interval of keepalive packets in seconds. Defaults to 3.
        max_fails (int, optional): Count of fails leading to socket disconnect. Defaults to 5.

    Raises:
        NotImplementedError: for unknown platform.
    """
    current_platform = platform.system()  # could be empty

    if current_platform == "Linux":
        return _set_keepalive_linux(sock, after_idle_sec, interval_sec, max_fails)
    if current_platform == "Windows":
        return _set_keepalive_windows(sock, after_idle_sec, interval_sec)
    if current_platform == "Darwin":
        return _set_keepalive_osx(sock, interval_sec)

    raise NotImplementedError


def _set_keepalive_linux(sock, after_idle_sec, interval_sec, max_fails):
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, after_idle_sec)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, interval_sec)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, max_fails)


def _set_keepalive_windows(sock, after_idle_sec, interval_sec):
    sock.ioctl(socket.SIO_KEEPALIVE_VALS, (1, after_idle_sec * 1000, interval_sec * 1000))


def _set_keepalive_osx(sock, interval_sec):
    TCP_KEEPALIVE = 0x10
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    sock.setsockopt(socket.IPPROTO_TCP, TCP_KEEPALIVE, interval_sec)
