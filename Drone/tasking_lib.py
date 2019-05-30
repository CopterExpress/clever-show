import heapq
import time
import logging
import threading
import collections
import itertools

logger = logging.getLogger(__name__)
Task = collections.namedtuple("Task", ["func", "args", "kwargs", "delayable", ])

def wait(end, interrupter=None, maxsleep=0.1):  # Added features to interrupter sleep and set max sleeping interval

    interrupted = False

    while not interrupted:  # Basic implementation of pause module until()
        now = time.time()
        diff = min(end - now, maxsleep)
        if interrupter is None:
            interrupted = False
        else:
            interrupted = interrupter.is_set()
        if diff <= 0:
            break
        else:
            time.sleep(diff / 2)
    
    if interrupted:
        logger.warning("Waiting was interrupted!")


class TaskManager(object):
    def __init__(self):
        self.task_queue = []
        self._counter = itertools.count()     # unique sequence count

        self._processor_thread = threading.Thread(target=self._task_processor, name="Task processing thread")
        self._processor_thread.daemon = True
        self._task_queue_lock = threading.RLock()

        self._running_event = threading.Event()
        self._interrupt_event = threading.Event()
        self._shutdown_event = threading.Event()

    def add_task(self, timestamp, priority, task_function,
                 task_args=(), task_kwargs=None, task_delayable=False):

        self._interrupt_event.set()

        if task_kwargs is None:
            task_kwargs = {}

        task = Task(task_function, task_args, task_kwargs, task_delayable)

        count = next(self._counter)
        entry = (timestamp, priority, count, task)
        
        with self._task_queue_lock:
            heapq.heappush(self.task_queue, entry)
        
        print(self.task_queue)

    def pop_task(self):
        with self._task_queue_lock:
            if self.task_queue:
                return heapq.heappop(self.task_queue)
            raise KeyError('Pop from an empty priority queue')

    def start(self, timeouts=False):
        print("Task manager is started")
        logger.info("Task manager is started")
        self._processor_thread.start()
        self.resume()

    def stop(self):
        self.pause()
        with self._task_queue_lock:
            del self.task_queue[:]

    def shutdown(self):
        self.stop()
        self._shutdown_event.set()
        self._running_event.clear()
        self._processor_thread.join(timeout=5)

    def pause(self, interrupt=False):
#        if interrupt:
#            self.interrupt()
        self._running_event.clear()
        logger.info("Task queue paused")

    def resume(self):
        self._running_event.set()
        logger.info("Task queue resumed")

    def reset(self):
        self.stop()
        self.resume()

#    def interrupt(self):
#        self._interrupt_event.set()
#        while self._interrupt_event.is_set():
#            pass
#        logger.info("Task execution successfully interrupted")

    def execute_task(self):
        with self._task_queue_lock:
            if self.task_queue:
                start_time, priority, count, task = self.task_queue[0]
            else: 
                return

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
            return

        if time.time() > start_time:
            self.pop_task()       

        logger.info("Execution done")

    def _task_processor(self):
        logger.info("Tasking thread started")
        # self._update_queue()  # Initial tick if tasks added before start
        while not self._shutdown_event.is_set():
            self._running_event.wait()
            self.execute_task()

if __name__ == "__main__":
    logger.addHandler(logging.StreamHandler())
    logger.setLevel(logging.DEBUG)

    def printer(stri, interrupter, *args, **kwargs):
        logger.info("String: {}, timenow: {}".format(stri, time.time()))
        wait(time.time()+30, interrupter)

    tasker = TaskManager()  # Lower priority first!

    tasker.start()
    tasker.add_task(time.time(), 10, printer, ("Task1 ", ))
    tasker.add_task(time.time()+10, 5, printer, ("Task2 ", ))
    time.sleep(1)
    tasker.add_task(time.time()+7, 1, printer, ("Task3", ))
    time.sleep(3)
    tasker.add_task(time.time()+7, 0, printer, ("Task4", ))

    while True:
        pass
