import heapq
import time
import logging
import threading
import collections

logger = logging.getLogger(__name__)
Task = collections.namedtuple("Task", ["func", "args", "kwargs", "delayable", ])


def wait(end, interrupter=None, maxsleep=1):  # Added features to interrupter sleep and set max sleeping interval
    def interrupted():
        if interrupter is None:
            return False
        else:
            return interrupter.is_set()

    while not interrupted():  # Basic implementation of pause module until()
        now = time.time()
        diff = min(end - now, maxsleep)
        if diff <= 0:
            break
        else:
            time.sleep(diff / 2)


class TaskManager(object):
    def __init__(self):
        self.task_queue = []
        self._active_task = None

        self._processor_thread = threading.Thread(target=self._task_processor, name="Task processing thread")
        self._processor_thread.daemon = True

        self._timeout_thread = threading.Thread(target=self._task_time_interrupter, name="Task timeouts thread")
        self._processor_thread.daemon = True

        self._task_lock = threading.RLock()
        self._queue_lock = threading.RLock()

        self._running_event = threading.Event()
        self._interrupt_event = threading.Event()
        self.shutdown_event = threading.Event()

    def add_task(self, timestamp, priority, task_function,
                 task_args=(), task_kwargs=None, task_delayable=False):
        if task_kwargs is None:
            task_kwargs = {}

        heapq.heappush(self.task_queue, (timestamp, priority,
                                         Task(task_function, task_args, task_kwargs, task_delayable)
                                         ))
        logger.debug(self.task_queue)
        if self._processor_thread.is_alive():
            self._update_queue()

    def _remove_task(self, task):
        with self._queue_lock:
            self.task_queue.remove(task)
            heapq.heapify(self.task_queue)

    def start(self, timeouts=False):
        logger.info("Starting")
        self._processor_thread.start()
        if timeouts:
            self._timeout_thread.start()
        self.resume()

    def stop(self):
        self.pause(interrupt=True)
        with self._queue_lock:
            self.task_queue = []

    def shutdown(self):
        self.stop()
        self.shutdown_event.set()
        self._running_event.clear()
        self._processor_thread.join(timeout=5)

    def pause(self, interrupt=False):
        if interrupt:
            self.interrupt()
        self._running_event.clear()
        logger.info("Task queue paused")

    def resume(self):
        self._running_event.set()
        logger.info("Task queue resumed")

    def reset(self):
        self.stop()
        self.resume()

    def _update_queue(self):
        logger.info("Queue updated")
        with self._queue_lock, self._task_lock:
            if self.task_queue:
                if self._active_task is None:
                    self._active_task = self.task_queue[0]
                elif self.task_queue[0] is not self._active_task:
                    if self.task_queue[0] < self._active_task:
                        self.change_active_task(self.task_queue[0])

    def interrupt(self):
        self._interrupt_event.set()
        while self._interrupt_event.is_set():
            pass
        logger.info("Task execution successfully interrupted")

    def change_active_task(self, task):
        self.pause(interrupt=True)

        with self._task_lock:
            if not self._active_task[2].delayable:
                self._remove_task(self._active_task)
            self._active_task = task

        self.resume()

    def execute_task(self):
        with self._task_lock:
            task = self._active_task[2]
            start_time = self._active_task[0]

        logger.info("Waiting util task execution time:{}".format(start_time))
        wait(start_time, self._interrupt_event, 1)

        if not self._interrupt_event.is_set():
            logger.info("Executing task {}".format(task))
            try:
                task.func(*task.args, interrupter=self._interrupt_event, **task.kwargs)
            except Exception as e:
                logger.error("Error '{}' occurred in task {}".format(e, task))
        else:
            logger.warning("Task interrupted before execution")

        self._interrupt_event.clear()

        try:
            logger.debug("Removing task")
            self._remove_task(self._active_task)
        except ValueError:
            logger.warning("Task already removed, probably task changed")
        else:
            with self._task_lock:
                self._active_task = None

            self._update_queue()
            logger.info("Execution done")

    def _task_processor(self):
        logger.info("Tasking thread started")
        self._update_queue()  # Initial tick if tasks added before start
        while not self.shutdown_event.is_set():
            self._running_event.wait()
            if self._active_task is not None:
                self.execute_task()

    def _task_time_interrupter(self):  # TODO revork; temporary disabled
        raise NotImplementedError
        logger.info("Timeouts thread started")
        while not self.shutdown_event.is_set():
            self._running_event.wait()
            try:
                if self.task_queue[1] is not self._active_task:  # If pending task is more important than current
                    if self.task_queue[1] < self._active_task:  # TODO look at timeout time
                        logger.warning("Changing low-priority task due timeout")
                        self.change_active_task(self.task_queue[1])
            except IndexError:
                time.sleep(0.01)


if __name__ == "__main__":
    logger.addHandler(logging.StreamHandler())
    logger.setLevel(logging.DEBUG)

    def printer(stri, interrupter, *args, **kwargs):
        logger.info("String: {}, timenow: {}".format(stri, time.time()))
        wait(time.time()+30, interrupter)

    tasker = TaskManager()  # Lower priority first!
    tasker.start()

    tasker.add_task(0, 10, printer, ("Task1 ", ))
    time.sleep(1)
    tasker.add_task(time.time(), 10, printer, ("Lol ", ))
    tasker.add_task(time.time()+10, 5, printer, ("Kek ", ))
    tasker.add_task(time.time()+7, 1, printer, ("Dededededee", ))
    time.sleep(3)
    tasker.add_task(time.time()+7, 0, printer, ("Iiiiiii", ))

    while True:
        pass
