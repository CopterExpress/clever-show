import collections
import asyncio

logger_format = "%(asctime)s [%(name)-11.11s] [%(levelname)-7.7s]  %(message)s"

def b_partial(func, *args, **kwargs):  # call argument blocker partial
    return lambda *a: func(*args, **kwargs)


class Callback:
    def __init__(self, *callbacks):
        self._callbacks = set()
        self._add_callbacks(callbacks)

    @property
    def all(self):
        return self._callbacks.copy()

    def add(self, callback):
        self._add_callback(callback)

    def remove(self, callback):
        self._callbacks.remove(callback)

    def _add_callbacks(self, callbacks):
        for callback in callbacks:
            self._add_callback(callback)

    def _add_callback(self, callback):
        if callback is None:
            return
        elif isinstance(callback, Callback):
            self._callbacks.update(callback.all)
        elif isinstance(callback, (list, set)):
            self._add_callbacks(callback)
        elif callable(callback):
            self._callbacks.add(callback)
        else:
            raise ValueError(f"Callback {callback} is not callable object!")

    def __call__(self, *args, **kwargs):
        return list(self.iter(*args, **kwargs))

    def iter(self, *args, **kwargs):
        for callback in self._callbacks:
            yield callback(*args, **kwargs)

    def gather_bool(self, *args, **kwargs):
        if self.all:
            return all(self(*args, **kwargs))
        return True

class KeyQueue:
    """A queue, useful for coordinating producer and consumer coroutines.
    If maxsize is less than or equal to zero, the queue size is infinite. If it
    is an integer greater than 0, then "await put()" will block when the
    queue reaches maxsize, until an item is removed by get().
    Unlike the standard library Queue, you can reliably know this Queue's size
    with qsize(), since your single-threaded asyncio application won't be
    interrupted between calling qsize() and doing an operation on the Queue.
    """

    def __init__(self, maxsize=0):
        self._loop = asyncio.events.get_event_loop()

        self._maxsize = maxsize

        # Futures.
        self._getters = collections.deque()
        # Futures.
        self._putters = collections.deque()
        self._unfinished_tasks = 0
        self._finished = asyncio.Event(loop=self._loop)
        self._finished.set()
        self._init(maxsize)

    # These three are overridable in subclasses.

    def _init(self, maxsize):
        self._queue = collections.deque()

    def _get(self, key=None):
        if key is None:
            return self._queue.popleft()[1]

        for pair in self._queue:
            k, item = pair
            if k == key:
                self._queue.remove(pair)
                return item
        else:
            raise KeyError(f"No item with key {key} in queue")

    def _put(self, item, key=None):
        self._queue.append((key, item))

    # End of the overridable methods.

    def _wakeup_next(self, waiters):
        # Wake up the next waiter (if any) that isn't cancelled.
        while waiters:
            waiter = waiters.popleft()
            if not waiter.done():
                waiter.set_result(None)
                break

    def _wakeup_get(self, key=None):
        if key is None:
            self._wakeup_next(self._getters)

        for pair in self._getters:
            k, waiter = pair
            if k == key and not waiter.done():
                waiter.set_result(None)
                self._getters.remove(pair)
                break

    def __repr__(self):
        return f'<{type(self).__name__} at {id(self):#x} {self._format()}>'

    def __str__(self):
        return f'<{type(self).__name__} {self._format()}>'

    def __class_getitem__(cls, type):
        return cls

    def _format(self):
        result = f'maxsize={self._maxsize!r}'
        if getattr(self, '_queue', None):
            result += f' _queue={list(self._queue)!r}'
        if self._getters:
            result += f' _getters[{len(self._getters)}]'
        if self._putters:
            result += f' _putters[{len(self._putters)}]'
        if self._unfinished_tasks:
            result += f' tasks={self._unfinished_tasks}'
        return result

    def qsize(self):
        """Number of items in the queue."""
        return len(self._queue)

    @property
    def maxsize(self):
        """Number of items allowed in the queue."""
        return self._maxsize

    def empty(self):
        """Return True if the queue is empty, False otherwise."""
        return not self._queue

    def full(self):
        """Return True if there are maxsize items in the queue.

        Note: if the Queue was initialized with maxsize=0 (the default),
        then full() is never True.
        """
        if self._maxsize <= 0:
            return False
        else:
            return self.qsize() >= self._maxsize

    async def put(self, item, key=None):
        """Put an item into the queue.

        Put an item into the queue. If the queue is full, wait until a free
        slot is available before adding item.
        """
        while self.full():
            putter = self._loop.create_future()
            self._putters.append(putter)
            try:
                await putter
            except:
                putter.cancel()  # Just in case putter is not done yet.
                try:
                    # Clean self._putters from canceled putters.
                    self._putters.remove(putter)
                except ValueError:
                    # The putter could be removed from self._putters by a
                    # previous get_nowait call.
                    pass
                if not self.full() and not putter.cancelled():
                    # We were woken up by get_nowait(), but can't take
                    # the call.  Wake up the next in line.
                    self._wakeup_next(self._putters)
                raise
        return self.put_nowait(item, key)

    def put_nowait(self, item, key=None):
        """Put an item into the queue without blocking.

        If no free slot is immediately available, raise QueueFull.
        """
        if self.full():
            raise asyncio.QueueFull
        self._put(item, key)
        self._unfinished_tasks += 1
        self._finished.clear()
        self._wakeup_get(key)

    async def get(self, key=None):
        """Remove and return an item from the queue.

        If queue is empty, wait until an item is available.
        """
        while self.empty():
            getter = self._loop.create_future()
            key_getter = key, getter
            self._getters.append(key_getter)
            try:
                await getter
            except:
                getter.cancel()  # Just in case getter is not done yet.
                try:
                    # Clean self._getters from canceled getters.
                    self._getters.remove(key_getter)
                except ValueError:
                    # The getter could be removed from self._getters by a
                    # previous put_nowait call.
                    pass
                if not self.empty() and not getter.cancelled():
                    # We were woken up by put_nowait(), but can't take
                    # the call.  Wake up the next in line.
                    self._wakeup_get(key)
                raise
        return self.get_nowait(key)

    def get_nowait(self, key=None):
        """Remove and return an item from the queue.

        Return an item if one is immediately available, else raise QueueEmpty.
        """
        if self.empty():
            raise asyncio.QueueEmpty
        item = self._get(key)
        self._wakeup_next(self._putters)
        return item

    def task_done(self):
        """Indicate that a formerly enqueued task is complete.

        Used by queue consumers. For each get() used to fetch a task,
        a subsequent call to task_done() tells the queue that the processing
        on the task is complete.

        If a join() is currently blocking, it will resume when all items have
        been processed (meaning that a task_done() call was received for every
        item that had been put() into the queue).

        Raises ValueError if called more times than there were items placed in
        the queue.
        """
        if self._unfinished_tasks <= 0:
            raise ValueError('task_done() called too many times')
        self._unfinished_tasks -= 1
        if self._unfinished_tasks == 0:
            self._finished.set()

    async def join(self):
        """Block until all items in the queue have been gotten and processed.

        The count of unfinished tasks goes up whenever an item is added to the
        queue. The count goes down whenever a consumer calls task_done() to
        indicate that the item was retrieved and all work on it is complete.
        When the count of unfinished tasks drops to zero, join() unblocks.
        """
        if self._unfinished_tasks > 0:
            await self._finished.wait()

    def cancel(self):
        # self._finished.set()
        # self._unfinished_tasks = 0
        while self._getters:
            pair = self._getters.popleft()
            key, getter = pair
            getter.cancel()
