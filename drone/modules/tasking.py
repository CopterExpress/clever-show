import heapq
import time
import logging
import threading
import collections
import itertools

logger = logging.getLogger(__name__)
Task = collections.namedtuple("Task", ["func", "args", "kwargs", "delayable", ])

INTERRUPTER = threading.Event()


def wait(end, interrupter=INTERRUPTER, maxsleep=0.1):
    # Added features to interrupter sleep and set max sleeping interval

    while not interrupter.is_set():  # Basic implementation of pause module until()
        now = time.time()
        diff = min(end - now, maxsleep)
        if diff <= 0:
            break
        else:
            time.sleep(diff / 2)
    else:
        logger.warning("Waiting was interrupted!")
        #print("Waiting was interrupted!")


class TaskManager(object):
    def __init__(self):
        self.task_queue = []
        self._counter = itertools.count()     # unique sequence count

        self._processor_thread = threading.Thread(target=self._task_processor, name="Task processing thread")
        self._processor_thread.daemon = True
        self._task_queue_lock = threading.RLock()

        self._running_event = threading.Event()
        self._reset_event = threading.Event()
        self._wait_interrupt_event = threading.Event()
        self._task_interrupt_event = threading.Event()
        self._shutdown_event = threading.Event()

        self._last_task = None

        self._timeshift = 0.0

    def add_task(self, timestamp, priority, task_function,
                 task_args=(), task_kwargs=None, task_delayable=False):

        if task_kwargs is None:
            task_kwargs = {}

        self._wait_interrupt_event.set()
        self._running_event.clear()

        task = Task(task_function, task_args, task_kwargs, task_delayable)

        count = next(self._counter)
        entry = (timestamp, priority, count, task)

        with self._task_queue_lock:
            if self.task_queue:
                entry_old = self.task_queue[0]
            else:
                entry_old = entry

            heapq.heappush(self.task_queue, entry)

            if self.task_queue[0] != entry_old:
                self._task_interrupt_event.set()
                #print("Task queue updated with more priority task")

            if self._reset_event.is_set():
                self._task_interrupt_event.set()
                self._reset_event.clear()
                #print("Task queue updated after reset")

        self._wait_interrupt_event.clear()
        self._running_event.set()

        # #print(self.task_queue)

    def pop_task(self):
        with self._task_queue_lock:
            if self.task_queue:
                return heapq.heappop(self.task_queue)
            raise KeyError('Pop from an empty priority queue')

    def get_last_task_name(self):
        try:
            name = self._last_task.func.__name__
            if name == 'execute_frame':
                return name.kwargs["frame"].action
            return name
        except AttributeError:
            return None

    def get_current_task(self):
        try:
            start_time, priority, count, task = self.task_queue[0]
        except IndexError as e:
            logger.debug("Task queue checking exception: {}".format(e))
            return "No task"
        else:
            if self._running_event.is_set():
                time_to_start = start_time - time.time()
                if time_to_start > 0:
                    return "{} in {:.1f} s".format(task.func.__name__,time_to_start)
                return task.func.__name__
            else:
                return "paused"

    def start(self):
        #print("Task manager is started")
        logger.info("Task manager is started")
        self._processor_thread.start()
        self.resume()

    def stop(self):
        self._timeshift = 0.0
        self.pause(interrupt=True)
        with self._task_queue_lock:
            del self.task_queue[:]

    def shutdown(self, timeout=5.0):
        self.stop()
        self._shutdown_event.set()
        self._processor_thread.join(timeout=timeout)

    def pause(self, interrupt=True):
        if interrupt:
            self._wait_interrupt_event.set()
            self._task_interrupt_event.set()
        self._running_event.clear()
        logger.info("Task queue paused")
        #print("Task queue paused")

    def resume(self, time_to_start_next_task=0.0):
        if self.task_queue:
            next_task_time = self.task_queue[0][0]
            if time_to_start_next_task > next_task_time:
                self._timeshift = time_to_start_next_task - next_task_time
        self._wait_interrupt_event.clear()
        self._task_interrupt_event.clear()
        self._running_event.set()
        logger.info("Task queue resumed with timeshift {}".format(self._timeshift))
        #print("Task queue resumed with timeshift {}".format(self._timeshift))

    def reset(self, interrupt_next_task=True):
        self.stop()
        self.resume()
        if interrupt_next_task:
            self._reset_event.set()

    def execute_task(self):
        delta = 0.1

        with self._task_queue_lock:
            try:
                start_time, priority, count, task = self.task_queue[0]
            except IndexError as e:
                logger.debug("Task queue checking exception: {}".format(e))
                self._timeshift = 0.0
                self._wait_interrupt_event.clear()
                self._task_interrupt_event.clear()
                self._running_event.clear()
                return

        task_start_time = start_time + self._timeshift
        logger.info("Executing task {}".format(task.func.__name__))
        logger.debug("Waiting util task execution time:{}".format(task_start_time))
        #print("Waiting until task execution time:{}".format(task_start_time))
        wait(task_start_time, self._wait_interrupt_event)

        if task_start_time - time.time() > 0.01:
            logger.error("Need for waiting more")
            self._wait_interrupt_event.clear()
            return

        if not self._wait_interrupt_event.is_set():
            #logger.info("Executing task {}".format(task))
            #print("{} Executing task {}".format(time.time(),task))
            #print("Interrupter is set: {}".format(self._task_interrupt_event.is_set()))
            try:
                task.func(*task.args, interrupter=self._task_interrupt_event, **task.kwargs)

            except Exception as e:
                try:
                    logger.error("Error '{}' occurred in task {}".format(e, task))
                #print("Error '{}' occurred in task {}".format(e, task))
                    if str(e) == 'STOP':
                        self.reset()
                        logger.error("Return after STOP exception, can't arm!")
                        return
                except (KeyError, TypeError):
                    logger.error(e)
        else:
            logger.error("Task interrupted before execution")
            #print("Task interrupted before execution")
            self._wait_interrupt_event.clear()
            return

        if time.time() > start_time:
            try:
                start_time_n, priority_n, count_n, task_n = self.task_queue[0]
            except IndexError as e:
                logger.warning("Timeout checking exception: {}".format(e))
                self._timeshift = 0.0
                self._wait_interrupt_event.clear()
                self._task_interrupt_event.clear()
                self._running_event.clear()
                return
            if (task_n == task) and (start_time_n == start_time):
                try:
                    self.pop_task()
                except KeyError as e:
                    logger.error(str(e))
                self._last_task = task
                #try:
                    #print("Pop {} function!".format(task.func.__name__))
                #except Exception as e:
                    #print("Pop something!")

        if self._task_interrupt_event.is_set():
            self._task_interrupt_event.clear()

        logger.info("Execution done")
        #print("Execution done")

    def _task_processor(self):
        logger.info("Tasking thread started")
        # self._update_queue()  # Initial tick if tasks added before start
        while not self._shutdown_event.is_set():
            self._running_event.wait()
            self.execute_task()

if __name__ == "__main__":
    #logger.addHandler(logging.StreamHandler())
    #logger.setLevel(logging.DEBUG)

    def printer(stri, interrupter, *args, **kwargs):
        #logger.info("String: {}, timenow: {}".format(stri, time.time()))
        wait(time.time()+30, interrupter)

    tasker = TaskManager()  # Lower priority first!

    tasker.start()
    tasker.add_task(time.time(), 10, printer, ("Task1 ", ))
    tasker.add_task(time.time()+10, 5, printer, ("Task2 ", ))
    time.sleep(1)
    tasker.add_task(time.time()+3, 1, printer, ("Task3", ))
    tasker.pause()
    time.sleep(5)
    tasker.resume(time_to_start_next_task=time.time()+1)
    tasker.add_task(time.time()+7, 0, printer, ("Task4", ))

    while True:
        pass
