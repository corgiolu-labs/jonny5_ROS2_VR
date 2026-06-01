import argparse
import select
import socket
import socketserver


class _ProxyHandler(socketserver.BaseRequestHandler):
    target_host = "127.0.0.1"
    target_port = 8443
    buffer_size = 65536

    def handle(self):
        upstream = socket.create_connection((self.target_host, self.target_port), timeout=10.0)
        upstream.setblocking(False)
        self.request.setblocking(False)
        sockets = [self.request, upstream]
        try:
            while True:
                readable, _, exceptional = select.select(sockets, [], sockets, 1.0)
                if exceptional:
                    break
                if not readable:
                    continue
                for src in readable:
                    dst = upstream if src is self.request else self.request
                    try:
                        data = src.recv(self.buffer_size)
                    except BlockingIOError:
                        continue
                    if not data:
                        return
                    view = memoryview(data)
                    while view:
                        sent = dst.send(view)
                        view = view[sent:]
        finally:
            try:
                upstream.close()
            except Exception:
                pass
            try:
                self.request.close()
            except Exception:
                pass


class _ThreadingTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    parser = argparse.ArgumentParser(description="Simple TCP proxy for exposing JONNY5 HTTPS on 443")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=443)
    parser.add_argument("--target-host", default="127.0.0.1")
    parser.add_argument("--target-port", type=int, default=8443)
    args = parser.parse_args()

    handler = type(
        "J5ProxyHandler",
        (_ProxyHandler,),
        {"target_host": args.target_host, "target_port": args.target_port},
    )
    with _ThreadingTCPServer((args.bind, args.port), handler) as server:
        print(f"TCP proxy {args.bind}:{args.port} -> {args.target_host}:{args.target_port}")
        server.serve_forever()


if __name__ == "__main__":
    main()
