import asyncio
import logging

import lib.protocols as p
import lib.messages as messages

logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(name)-7.7s] [%(threadName)-19.19s] [%(levelname)-7.7s]  %(message)s",
        handlers=[
            logging.StreamHandler()
        ])

async def server():
    loop = asyncio.get_event_loop()

    server = await loop.create_server(lambda: p.PeerProtocol(None, None), host='', port=8181)

    await server.wait_closed()

async def client():
    loop = asyncio.get_event_loop()

    transport, protocol = await loop.create_connection(lambda: p.PeerProtocol(None, None),
                                                       host='', port=8181)
    await protocol.send(messages.Request("greetings"))

    await protocol.closed

async def main():
    s = asyncio.create_task(server())
    c = asyncio.create_task(client())
    await asyncio.gather(s, c)

asyncio.run(main(), debug=True)
