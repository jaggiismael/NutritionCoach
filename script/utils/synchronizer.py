import time
import asyncio
import concurrent.futures

#Source: https://docs.luxai.com/docs/tutorials/python/python_ros_sync_robot_behaviors
class TaskSynchronizer():

    def __init__(self, max_workers=5):
        self.loop = asyncio.get_event_loop()
        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=max_workers)


    def __worker(self, *args):
        delay_exe = args[0][0]
        func = args[0][1]
        time.sleep(delay_exe)
        return func()
    
    async def __non_blocking(self, tasks):
        fs = []
        for task in tasks:
            fs.append(self.loop.run_in_executor(
                self.executor, self.__worker, task))
        done, pending = await asyncio.wait(fs=fs, return_when=asyncio.ALL_COMPLETED)
        results = [task.result() for task in done]
        return results
    
    def sync(self, tasks):
        results = self.loop.run_until_complete(self.__non_blocking(tasks))
        return results