import asyncio


class Timer:
    def __init__(self, timeout: float, callback, callback_args=(), callback_kwargs={}):
        self._timeout = timeout
        self._callback = callback
        self._task = asyncio.create_task(self._job())
        self._callback_args = callback_args
        self._callback_kwargs = callback_kwargs

    async def _job(self):
        await asyncio.sleep(self._timeout)
        await self._call_callback()

    async def _call_callback(self):
        if asyncio.iscoroutine(self._callback):
            await self._callback(*self._callback_args, **self._callback_kwargs)
        else:
            self._callback(*self._callback_args, **self._callback_kwargs)

    def cancel(self):
        """
        Cancels the timer. The callback will not be called.
        """
        self._task.cancel()

    def end_early(self):
        """
       Ends the timer early, and calls the callback coroutine.
       """
        self._task.cancel()
        asyncio.create_task(self._call_callback())
