# import logging
# import threading
# import time
# import tqdm
# import concurrent.futures


# # def thread_function(name):
# #     logging.info("Thread %s: starting", name)
# #     time.sleep(2)
# #     logging.info("Thread %s: finishing", name)


# # if __name__ == "__main__":
# #     format = "%(asctime)s: %(message)s"
# #     logging.basicConfig(format=format, level=logging.INFO,
# #                         datefmt="%H:%M:%S")

# #     with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
# #          executor.map(thread_function, range(3))
# # class FakeDatabase:
# #     def __init__(self):
# #         self.value = 0

# #     def update(self, name):
# #         logging.info("Thread %s: starting update", name)
# #         local_copy = self.value
# #         local_copy += 1
# #         time.sleep(0.1)
# #         self.value = local_copy
# #         logging.info("Thread %s: finishing update", name)

# # if __name__ == "__main__":
# #     format = "%(asctime)s: %(message)s"
# #     logging.basicConfig(format=format, level=logging.INFO,
# #                         datefmt="%H:%M:%S")

# #     database = FakeDatabase()
# #     logging.info("Testing update. Starting value is %d.", database.value)
# #     with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
# #         for index in range(2):
# #             executor.submit(database.update, index)
# #     logging.info("Testing update. Ending value is %d.", database.value)

# # class FakeDatabase:
# #     def __init__(self):
# #         self.value = 0
# #         self._lock = threading.Lock()

# #     def locked_update(self, name):
# #         logging.info("Thread %s: starting update", name)
# #         logging.debug("Thread %s about to lock", name)
# #         with self._lock:
# #             logging.debug("Thread %s has lock", name)
# #             local_copy = self.value
# #             local_copy += 1
# #             time.sleep(0.1)
# #             self.value = local_copy
# #             logging.debug("Thread %s about to release lock", name)
# #         logging.debug("Thread %s after release", name)
# #         logging.info("Thread %s: finishing update", name)

# # if __name__ == "__main__":
# #     format = "%(asctime)s: %(message)s"
# #     logging.basicConfig(format=format, level=logging.INFO,
# #                         datefmt="%H:%M:%S")
# #     logging.getLogger().setLevel(logging.DEBUG)

# #     database = FakeDatabase()
# #     logging.info("Testing update. Starting value is %d.", database.value)
# #     with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
# #         for index in range(2):
# #             executor.submit(database.locked_update, index)
# #     logging.info("Testing update. Ending value is %d.", database.value)

# # count=1
# # for i in tqdm.trange(0,5,position=1,desc="loop 1:"):
# #     for j in tqdm.trange(0,3,position=2,desc="loop 2:",leave=bool(i==4)):
# #         count +=1
# #         time.sleep(1)


# # pbar=tqdm.tqdm(total=5)

# # for i in range(0,5):
# #     time.sleep(1)
# #     pbar.update(1)

# # from time import sleep
# # from tqdm import trange, tqdm
# # from multiprocessing import Pool, RLock, freeze_support

# # L = list(range(9))

# # def progresser(n):
# #     interval = 0.001 / (n + 2)
# #     total = 5000
# #     text = "#{}, est. {:<04.2}s".format(n, interval * total)
# #     for _ in trange(total, desc=text, position=n):
# #         sleep(interval)

# # if __name__ == '__main__':
# #     freeze_support()  # for Windows support
# #     tqdm.set_lock(RLock())  # for managing output contention
# #     p = Pool(initializer=tqdm.set_lock, initargs=(tqdm.get_lock(),))
# #     p.map(progresser, L)

# from time import sleep
# from tqdm import tqdm, trange
# from concurrent.futures import ThreadPoolExecutor

# L = list(range(9))

# def progresser(n):
#     interval = 0.001 / (n + 2)
#     total = 5000
#     text = "#{}, est. {:<04.2}s".format(n, interval * total)
#     for _ in trange(total, desc=text):
#         sleep(interval)
#     if n == 6:
#         tqdm.write("n == 6 completed.")
#         tqdm.write("`tqdm.write()` is thread-safe in py3!")

# if __name__ == '__main__':
#     with ThreadPoolExecutor() as p:
#         p.map(progresser, L)
