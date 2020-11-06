import asyncio
import asyncio.constants as constants

async def _accept_connection2(
        self, protocol_factory, conn, extra,
        sslcontext=None, server=None,
        ssl_handshake_timeout=constants.SSL_HANDSHAKE_TIMEOUT):
    protocol = None
    transport = None
    try:
        protocol = protocol_factory(conn.getpeername())
        waiter = self.create_future()
        if sslcontext:
            transport = self._make_ssl_transport(
                conn, protocol, sslcontext, waiter=waiter,
                server_side=True, extra=extra, server=server,
                ssl_handshake_timeout=ssl_handshake_timeout)
        else:
            transport = self._make_socket_transport(
                conn, protocol, waiter=waiter, extra=extra,
                server=server)

        try:
            await waiter
        except BaseException:
            transport.close()
            raise
            # It's now up to the protocol to handle the connection.

    except (SystemExit, KeyboardInterrupt):
        raise
    except BaseException as exc:
        if self._debug:
            context = {
                'message':
                    'Error on transport creation for incoming connection',
                'exception': exc,
            }
            if protocol is not None:
                context['protocol'] = protocol
            if transport is not None:
                context['transport'] = transport
            self.call_exception_handler(context)

asyncio.SelectorEventLoop._accept_connection2 = _accept_connection2
