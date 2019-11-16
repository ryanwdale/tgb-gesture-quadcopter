from multiprocessing import Process, Pipe
from time import sleep


def producer(conn):
    for i in range(100):
        conn.send([i, i, i])
    conn.close()


def consumer(conn):
    for i in range(100):
        res = conn.recv()
        print("text =", res)
    conn.close()


parent_conn, child_conn = Pipe()

producer_process = Process(target=producer, args=(child_conn,))
consumer_process = Process(target=consumer, args=(parent_conn,))

producer_process.start()
consumer_process.start()

producer_process.join()
consumer_process.join()
