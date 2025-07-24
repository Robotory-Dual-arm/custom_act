# Author: Chemin Ahn (chemx3937@gmail.com)
# Use of this source code is governed by the MIT, see LICENSE

# Camera serial 확인용

import pyrealsense2 as rs
import numpy as np
import cv2
import time

def get_device_serials():
    ctx = rs.context()
    serials = []
    for device in ctx.query_devices():
        serials.append(device.get_info(rs.camera_info.serial_number))
    if len(serials) < 2:
        raise RuntimeError("2개 이상의 Realsense 카메라가 연결되어 있어야 합니다.")
    print("Detected serials:", serials)
    return serials

def show_serials_with_index():
    serials = get_device_serials()
    serial_d405 = serials[0]
    serial_d435 = serials[1]

    pipelines = []
    for serial in [serial_d405, serial_d435]:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(config)
        pipelines.append((serial, pipeline))

    print(f"[INFO] serials[0] = {serial_d405}, serials[1] = {serial_d435}")
    print("[INFO] 카메라 화면 좌상단에 인덱스와 시리얼 번호가 표시됩니다. 'q'를 눌러 종료하세요.")

    font = cv2.FONT_HERSHEY_SIMPLEX

    while True:
        for idx, (serial, pipeline) in enumerate(pipelines):
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            image = np.asanyarray(color_frame.get_data())
            label = f"serials[{idx}]: {serial}"
            cv2.putText(image, label, (20, 40), font, 0.9, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow(f"Camera {idx}", image)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    for _, pipeline in pipelines:
        pipeline.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    show_serials_with_index()
