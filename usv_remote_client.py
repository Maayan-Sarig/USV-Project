#!/usr/bin/env python3
import argparse
import json
import socket


def send_command(server_host, server_port, command, timeout=5.0):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        sock.sendto(command.encode('utf-8'), (server_host, server_port))
        response, _ = sock.recvfrom(8192)
        return response.decode('utf-8', errors='ignore')
    finally:
        sock.close()


def interactive_mode(server_host, server_port):
    print('USV remote client interactive mode. Type exit to quit.')
    while True:
        try:
            line = input('> ').strip()
        except EOFError:
            break
        if not line:
            continue
        if line.lower() in ('exit', 'quit'):
            break
        response = send_command(server_host, server_port, line)
        try:
            parsed = json.loads(response)
            print(json.dumps(parsed, indent=2))
        except Exception:
            print(response)


def main():
    parser = argparse.ArgumentParser(description='USV remote client for sending commands to the Pi server')
    parser.add_argument('--host', default='192.168.1.100', help='Pi server IP address')
    parser.add_argument('--port', type=int, default=15000, help='Pi server UDP port')
    parser.add_argument('--interactive', action='store_true', help='Run an interactive command prompt')
    parser.add_argument('command', nargs='*', help='Command text to send')
    args = parser.parse_args()

    if args.interactive or not args.command:
        interactive_mode(args.host, args.port)
        return

    command_text = ' '.join(args.command)
    response = send_command(args.host, args.port, command_text)
    try:
        parsed = json.loads(response)
        print(json.dumps(parsed, indent=2))
    except Exception:
        print('Response:', response)


if __name__ == '__main__':
    main()
