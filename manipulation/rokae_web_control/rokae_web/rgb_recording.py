"""Dedicated JPEG-only recording store for the left wrist camera."""
from datetime import datetime, timedelta
from pathlib import Path
import threading


class RgbRecordingStore:
    def __init__(self, root):
        self.root = Path(root)
        self.lock = threading.Lock()

    def begin_scan(self):
        with self.lock:
            instant = datetime.now()
            while True:
                relative = Path(instant.strftime('%Y-%m-%d')) / ('scan_' + instant.strftime('%H%M%S%f')[:9])
                directory = self.root / relative
                directory.parent.mkdir(parents=True, exist_ok=True)
                try:
                    directory.mkdir()
                    return dict(directory=str(directory), relative_directory=relative.as_posix())
                except FileExistsError:
                    instant += timedelta(milliseconds=1)

    def save_scan(self, group, index, jpeg):
        if type(index) is not int or not 1 <= index <= 5:
            raise ValueError('扫码图片序号必须为1–5')
        directory = Path(group['directory']).resolve()
        relative = directory.relative_to(self.root.resolve())
        if len(relative.parts) != 2 or not relative.name.startswith('scan_') or not directory.is_dir():
            raise ValueError('无效的扫码图片目录')
        path = directory / f'扫码{index}.jpg'
        with self.lock:
            output = path.open('xb')
            try:
                with output:
                    output.write(jpeg)
            except Exception:
                path.unlink(missing_ok=True)
                raise
        return dict(camera_id='left_wrist', directory=str(directory), path=str(path),
                    relative_path=(relative / path.name).as_posix(), filename=path.name)

    def save(self, jpeg: bytes) -> dict:
        with self.lock:
            instant = datetime.now()
            while True:
                relative = Path(instant.strftime('%Y-%m-%d')) / (
                    instant.strftime('%H%M%S%f')[:9] + '.jpg')
                path = self.root / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                try:
                    output = path.open('xb')
                except FileExistsError:
                    instant += timedelta(milliseconds=1)
                    continue
                try:
                    with output:
                        output.write(jpeg)
                except Exception:
                    path.unlink(missing_ok=True)
                    raise
                return dict(camera_id='left_wrist', directory=str(path.parent),
                            path=str(path), relative_path=relative.as_posix(),
                            filename=path.name)
