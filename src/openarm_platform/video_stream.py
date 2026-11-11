"""Optional GUI factory: resilient multipart video without per-frame websocket IO."""
_stream_type = None


def video_stream(src, *, reconnect=True):
    global _stream_type
    if _stream_type is None:
        from nicegui.element import Element

        class VideoStream(Element, component='static/video_stream.js'):
            def __init__(self, src, reconnect):
                super().__init__()
                self._props.update(src=src, reconnect=reconnect)

            def set_reconnect(self, value):
                if self._props['reconnect'] != value:
                    self._props['reconnect'] = value
                    self.update()

        _stream_type = VideoStream
    return _stream_type(src, reconnect)
