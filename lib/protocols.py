import logging
import asyncio

import messages
import exceptions

from network import str_peername

logger = logging.getLogger(__name__)


class BroadcastProtocol:
    def __init__(self, on_broadcast):
        self.transport = None
        self._on_broadcast = on_broadcast

        self._closed = asyncio.Event()

    @property
    def closed(self):
        return self._closed.wait()

    def connection_made(self, transport):
        self.transport = transport
        logger.info("Broadcast connection established")

    def connection_lost(self, exc):
        logger.info(f"Broadcast connection lost: {'closed' if exc is None else exc}")
        self._closed.set()

    def datagram_received(self, data, addr):
        message = message.MessageDecoder(data)
        message.process_message()
        content = message.content

        is_ip_broadcast = (content is not None and message.header["action"] == "server_ip")

        if is_ip_broadcast:
            logger.debug(f"Got broadcast message from {addr}: {content}")
            asyncio.get_event_loop().call_soon(self._on_broadcast, message)
            self.transport.close()
        else:
            logger.warning(f"Got wrong broadcast message from {addr}")

    def error_received(self, exc):
        logger.warning(f"Error on broadcast connection received: {exc}")

class PeerProtocol(asyncio.Protocol):
    def __init__(self, connected_callback, callbacks):
        self._callbacks = callbacks

        self.transport: asyncio.Transport = None

        self._connected = asyncio.Event()
        self._closed = asyncio.Event()
        self._closed.set()

        self._recv_buffer = bytearray()
        self._recv_queue = asyncio.Queue()
        self._current_msg = None
        self._recv_process_task = None

        self._send_queue = asyncio.Queue()  # holds messages to send
        self._send_task = None

        self._can_write = asyncio.Event()

        self._recv_waiter = None

    @property
    def peername(self):
        return self.transport.get_extra_info('peername')

    @property
    def is_connected(self):
        return self._connected.is_set()

    @property
    def connected(self):
        return self._connected.wait()

    @property
    def closed(self):
        return self._closed.wait()

    # Drain control

    def pause_writing(self) -> None:
        self._can_write.clear()

    def resume_writing(self) -> None:
        self._can_write.set()

    async def drain(self) -> None:
        await self._can_write.wait()

    async def send(self, msg: messages.PendingMessage):
        if not self.is_connected:  # and not send_disconnected
            msg.sent.cancel("Peer is disconnected, can't send")
            raise RuntimeError("Peer is disconnected, can't send")

        await self._send_queue.put(msg)
        if self._send_task is None:
            self._send_task = asyncio.create_task(self._send())

        logger.debug(f"Queued sending of message {msg} to {str_peername(self.peername)}")
        await msg.sent

    async def _send(self):
        while self.is_connected:
            msg = None
            try:
                msg: messages.PendingMessage = await self._send_queue.get()
                self.transport.write(msg.encode())
                await self.drain()
            except asyncio.CancelledError:
                if msg is not None:
                    msg.sent.set_exception(exceptions.ConnectionClosedError)
                raise
            else:
                msg.sent.set_result(None)
                logger.debug(f"Sent message {msg} to {str_peername(self.peername)}")

    def connection_made(self, transport):
        self.transport = transport
        logger.info(f"Connected to {str_peername(self.peername)}")

        self._connected.set()
        self._closed.clear()

        self._can_write.set()

    def connection_lost(self, exc):
        logger.info(f"Lost connection to {str_peername(self.peername)}: {'closed' if exc is None else exc}")
        self._connected.clear()
        self._closed.set()

        if self._recv_waiter is not None:
            self._recv_waiter.cancel()

        if self._recv_process_task is not None:
            self._recv_process_task.cancel()

        if self._send_task is not None:
            self._send_task.cancel()

        self._can_write.set()

        self._recv_buffer = bytearray()
        self._current_msg = None

    def error_received(self, exc):
        logger.warning(f"Error on {str_peername(self.peername)} connection received: {exc}")

    def data_received(self, data):
        self._recv_buffer += data
        logger.debug(f"Received {len(data)} bytes from {str_peername(self.peername)}")

        if self._recv_process_task is None:
            self._recv_process_task = asyncio.create_task(self._proceess_received())

    async def _proceess_received(self):
        try:
            while self._recv_buffer:
                if self._current_msg is None:
                    self._current_msg = messages.MessageDecoder(self._recv_buffer)
                else:
                    self._current_msg.set_buffer(self._recv_buffer)

                self._current_msg.process_message()
                self._recv_buffer = self._current_msg.get_buffer()
                # self._current_msg.reset_buffer()
                if self._current_msg.processed:
                    logger.debug(f"Recieved message {self._current_msg.content} from {str_peername(self.peername)}")
                    await self._recv_queue.put(self._current_msg)
                    self._current_msg = None
                else:
                    await asyncio.sleep(0)
        except Exception as e:
            logger.error(f"Error during message processing: {e}")
        else:
            logger.debug("All data processed")
        finally:
            self._recv_process_task = None

    async def receive_message(self):
        if self._recv_waiter is not None:
            raise RuntimeError("receive_message() is already being awaited")

        self._recv_waiter = asyncio.create_task(self._recv_queue.get())

        try:
            return await self._recv_waiter
        except asyncio.CancelledError:
            raise exceptions.ConnectionClosedError
        finally:
            self._recv_waiter = None
