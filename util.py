# Author: Shing-Yan Loo (lsyan@ualberta.ca)
# helper functions

import multiprocessing 

def destroy_queue(queue):
    """destroy the queue

    :param queue: queue object
    :type queue: multiprocessing.Queue or Queue
    """
    # Clear the queues
    while not queue.empty():
        queue.get()

        if type(queue) != multiprocessing.queues.Queue:
            queue.task_done()
    
    if type(queue) == multiprocessing.queues.Queue:
        queue.close()
        queue.join_thread()
    else:
        queue.join()
