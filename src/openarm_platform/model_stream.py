"""Latest model-input PNGs as one cancellable multipart browser connection."""
import asyncio


async def input_frames(path, active, *, period=.05):
    previous = None
    while True:
        try:
            stat = await asyncio.to_thread(path.stat)
            identity = (stat.st_mtime_ns, stat.st_ino)
            if stat.st_size > 4_000_000:
                return
            if identity != previous:
                data = await asyncio.to_thread(path.read_bytes)
                if len(data) > 4_000_000:
                    return
                previous = identity
                yield (b'--frame\r\nContent-Type: image/png\r\nContent-Length: '
                       + str(len(data)).encode('ascii')+b'\r\n\r\n'+data+b'\r\n')
        except OSError:
            return
        if not active():
            return
        await asyncio.sleep(period)
