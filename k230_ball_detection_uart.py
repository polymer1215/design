import os
import sys
import ujson
import aicube
import gc
import time
import image

from media.sensor import *
from media.display import *
from media.media import *

import nncase_runtime as nn
import ulab.numpy as np
from machine import FPIOA, UART


# ============================================================
# 显示和摄像头尺寸
# ============================================================

display_mode = "lcd"

if display_mode == "lcd":
    # 亚博 K230 自带 LCD：640x480
    DISPLAY_WIDTH = ALIGN_UP(640, 16)
    DISPLAY_HEIGHT = 480
else:
    DISPLAY_WIDTH = ALIGN_UP(1920, 16)
    DISPLAY_HEIGHT = 1080

# AI 摄像头通道分辨率
# 使用明确支持的标准分辨率，避免原代码 1080 对齐后变成 1088
AI_FRAME_WIDTH = ALIGN_UP(640, 16)
AI_FRAME_HEIGHT = 480

UART_BAUD_RATE = 115200
SEND_NO_TARGET_FRAME = True


# ============================================================
# 颜色盘：ARGB
# 实际绘图时使用 [1:] 取 RGB
# ============================================================

color_four = [
    (255, 220, 20, 60),
    (255, 119, 11, 32),
    (255, 0, 0, 142),
    (255, 0, 0, 230),
    (255, 106, 0, 228),
    (255, 0, 60, 100),
    (255, 0, 80, 100),
    (255, 0, 0, 70),
    (255, 0, 0, 192),
    (255, 250, 170, 30),
    (255, 100, 170, 30),
    (255, 220, 220, 0),
    (255, 175, 116, 175),
    (255, 250, 0, 30),
    (255, 165, 42, 42),
    (255, 255, 77, 255),
    (255, 0, 226, 252),
    (255, 182, 182, 255),
    (255, 0, 82, 0),
    (255, 120, 166, 157),
    (255, 110, 76, 0),
    (255, 174, 57, 255),
    (255, 199, 100, 0),
    (255, 72, 0, 118),
    (255, 255, 179, 240),
    (255, 0, 125, 92),
    (255, 209, 0, 151),
    (255, 188, 208, 182),
    (255, 0, 220, 176),
    (255, 255, 99, 164),
    (255, 92, 0, 73),
    (255, 133, 129, 255),
    (255, 78, 180, 255),
    (255, 0, 228, 0),
    (255, 174, 255, 243),
    (255, 45, 89, 255),
    (255, 134, 134, 103),
    (255, 145, 148, 174),
    (255, 255, 208, 186),
    (255, 197, 226, 255),
    (255, 171, 134, 1),
    (255, 109, 63, 54),
    (255, 207, 138, 255),
    (255, 151, 0, 95),
    (255, 9, 80, 61),
    (255, 84, 105, 51),
    (255, 74, 65, 105),
    (255, 166, 196, 102),
    (255, 208, 195, 210),
    (255, 255, 109, 65),
    (255, 0, 143, 149),
    (255, 179, 0, 194),
    (255, 209, 99, 106),
    (255, 5, 121, 0),
    (255, 227, 255, 205),
    (255, 147, 186, 208),
    (255, 153, 69, 1),
    (255, 3, 95, 161),
    (255, 163, 255, 0),
    (255, 119, 0, 170),
    (255, 0, 182, 199),
    (255, 0, 165, 120),
    (255, 183, 130, 88),
    (255, 95, 32, 0),
    (255, 130, 114, 135),
    (255, 110, 129, 133),
    (255, 166, 74, 118),
    (255, 219, 142, 185),
    (255, 79, 210, 114),
    (255, 178, 90, 62),
    (255, 65, 70, 15),
    (255, 127, 167, 115),
    (255, 59, 105, 106),
    (255, 142, 108, 45),
    (255, 196, 172, 0),
    (255, 95, 54, 80),
    (255, 128, 76, 255),
    (255, 201, 57, 1),
    (255, 246, 0, 122),
    (255, 191, 162, 208)
]


# ============================================================
# 模型部署配置
# ============================================================

root_path = "/sdcard/mp_deployment_source/"
config_path = root_path + "deploy_config.json"

debug_mode = 1

# IoU 大于此值时合并检测框
MERGE_IOU_THRESH = 0.3


# ============================================================
# 性能计时
# ============================================================

class ScopedTiming:
    def __init__(self, info="", enable_profile=True):
        self.info = info
        self.enable_profile = enable_profile
        self.start_time = 0

    def __enter__(self):
        if self.enable_profile:
            self.start_time = time.time_ns()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.enable_profile:
            elapsed_time = time.time_ns() - self.start_time
            print(
                "{} took {:.2f} ms".format(
                    self.info,
                    elapsed_time / 1000000
                )
            )


# ============================================================
# 读取部署配置
# ============================================================

def read_deploy_config(path):
    config = None

    with open(path, "r") as json_file:
        try:
            config = ujson.load(json_file)
        except ValueError as e:
            print("JSON解析错误:", e)
            raise e

    return config


# ============================================================
# 检测框合并
# ============================================================

def merge_overlap_boxes(boxes):
    """
    合并重叠检测框。
    输入格式：
        [class_id, confidence, x1, y1, x2, y2]
    """

    if boxes is None or len(boxes) <= 1:
        return boxes

    merged = []
    used = [False] * len(boxes)

    for i in range(len(boxes)):
        if used[i]:
            continue

        x1 = boxes[i][2]
        y1 = boxes[i][3]
        x2 = boxes[i][4]
        y2 = boxes[i][5]

        best_conf = boxes[i][1]
        best_cls = boxes[i][0]

        used[i] = True
        changed = True

        while changed:
            changed = False

            for j in range(len(boxes)):
                if used[j]:
                    continue

                bx1 = boxes[j][2]
                by1 = boxes[j][3]
                bx2 = boxes[j][4]
                by2 = boxes[j][5]

                ix1 = max(x1, bx1)
                iy1 = max(y1, by1)
                ix2 = min(x2, bx2)
                iy2 = min(y2, by2)

                inter_w = max(0, ix2 - ix1)
                inter_h = max(0, iy2 - iy1)
                inter = inter_w * inter_h

                area_a = max(0, x2 - x1) * max(0, y2 - y1)
                area_b = max(0, bx2 - bx1) * max(0, by2 - by1)

                union = area_a + area_b - inter
                iou = inter / union if union > 0 else 0

                if iou > MERGE_IOU_THRESH:
                    x1 = min(x1, bx1)
                    y1 = min(y1, by1)
                    x2 = max(x2, bx2)
                    y2 = max(y2, by2)

                    if boxes[j][1] > best_conf:
                        best_conf = boxes[j][1]
                        best_cls = boxes[j][0]

                    used[j] = True
                    changed = True

        merged.append([
            best_cls,
            best_conf,
            x1,
            y1,
            x2,
            y2
        ])

    return merged


# ============================================================
# 坐标限制
# ============================================================

def clamp(value, min_value, max_value):
    if value < min_value:
        return min_value

    if value > max_value:
        return max_value

    return value


# ============================================================
# 主检测程序
# ============================================================

def init_ball_uart():
    """Configure the Yahboom K230 communication connector at 115200-8-N-1."""
    fpioa = FPIOA()
    fpioa.set_function(9, FPIOA.UART1_TXD, ie=0, oe=1)
    fpioa.set_function(10, FPIOA.UART1_RXD, ie=1, oe=0)

    return UART(
        UART.UART1,
        baudrate=UART_BAUD_RATE,
        bits=UART.EIGHTBITS,
        parity=UART.PARITY_NONE,
        stop=UART.STOPBITS_ONE,
        timeout=0
    )


def send_ball_center(uart, center_x, center_y):
    """Send one ASCII coordinate frame and discard the previous MCU echo."""
    uart.read(128)

    if center_x >= 0 and center_y >= 0:
        x = int(center_x + 0.5)
        y = int(center_y + 0.5)
        frame = "BALL,{:03d},{:03d}\r\n".format(x, y)
    elif SEND_NO_TARGET_FRAME:
        frame = "BALL,-1,-1\r\n"
    else:
        return

    uart.write(frame.encode())


def detection():
    print("det_infer start")

    sensor = None
    osd_img = None
    rgb888p_img = None

    ai2d_input_tensor = None
    ai2d_output_tensor = None

    ai2d = None
    ai2d_builder = None
    kpu = None
    uart = None

    try:
        uart = init_ball_uart()
        send_ball_center(uart, -1, -1)

        # ----------------------------------------------------
        # 读取部署配置
        # ----------------------------------------------------

        deploy_conf = read_deploy_config(config_path)

        kmodel_name = deploy_conf["kmodel_path"]
        labels = deploy_conf["categories"]

        confidence_threshold = deploy_conf["confidence_threshold"]
        nms_threshold = deploy_conf["nms_threshold"]

        img_size = deploy_conf["img_size"]
        num_classes = deploy_conf["num_classes"]
        nms_option = deploy_conf["nms_option"]
        model_type = deploy_conf["model_type"]

        anchors = None

        if model_type == "AnchorBaseDet":
            anchors = (
                deploy_conf["anchors"][0]
                + deploy_conf["anchors"][1]
                + deploy_conf["anchors"][2]
            )

        # 模型输入尺寸：[宽, 高]
        model_width = int(img_size[0])
        model_height = int(img_size[1])

        kmodel_frame_size = [
            model_width,
            model_height
        ]

        # 后处理使用的原始摄像头尺寸
        frame_size = [
            AI_FRAME_WIDTH,
            AI_FRAME_HEIGHT
        ]

        strides = [8, 16, 32]

        print(
            "LCD size: {}x{}".format(
                DISPLAY_WIDTH,
                DISPLAY_HEIGHT
            )
        )

        print(
            "AI frame size: {}x{}".format(
                AI_FRAME_WIDTH,
                AI_FRAME_HEIGHT
            )
        )

        print(
            "Model input size: {}x{}".format(
                model_width,
                model_height
            )
        )

        # ----------------------------------------------------
        # 计算 letterbox padding
        # ----------------------------------------------------

        ori_w = AI_FRAME_WIDTH
        ori_h = AI_FRAME_HEIGHT

        ratio_w = float(model_width) / float(ori_w)
        ratio_h = float(model_height) / float(ori_h)

        ratio = min(ratio_w, ratio_h)

        new_w = int(ratio * ori_w)
        new_h = int(ratio * ori_h)

        dw = float(model_width - new_w) / 2.0
        dh = float(model_height - new_h) / 2.0

        top = int(round(dh - 0.1))
        bottom = int(round(dh + 0.1))
        left = int(round(dw - 0.1))
        right = int(round(dw + 0.1))

        print(
            "ai2d padding: top={}, bottom={}, left={}, right={}".format(
                top,
                bottom,
                left,
                right
            )
        )

        # ----------------------------------------------------
        # 初始化 KPU 和 ai2d
        # ----------------------------------------------------

        kpu = nn.kpu()
        kpu.load_kmodel(root_path + kmodel_name)

        ai2d = nn.ai2d()

        ai2d.set_dtype(
            nn.ai2d_format.NCHW_FMT,
            nn.ai2d_format.NCHW_FMT,
            np.uint8,
            np.uint8
        )

        ai2d.set_pad_param(
            True,
            [
                0, 0,
                0, 0,
                top, bottom,
                left, right
            ],
            0,
            [114, 114, 114]
        )

        ai2d.set_resize_param(
            True,
            nn.interp_method.tf_bilinear,
            nn.interp_mode.half_pixel
        )

        # ai2d 输入、输出均为 NCHW
        ai2d_builder = ai2d.build(
            [
                1,
                3,
                AI_FRAME_HEIGHT,
                AI_FRAME_WIDTH
            ],
            [
                1,
                3,
                model_height,
                model_width
            ]
        )

        # 正确顺序：N、C、H、W
        output_data = np.ones(
            (
                1,
                3,
                model_height,
                model_width
            ),
            dtype=np.uint8
        )

        ai2d_output_tensor = nn.from_numpy(output_data)

        # ----------------------------------------------------
        # 初始化摄像头
        # ----------------------------------------------------

        sensor = Sensor()
        sensor.reset()

        sensor.set_hmirror(False)
        sensor.set_vflip(False)

        # 通道0：LCD 视频输出
        sensor.set_framesize(
            width=DISPLAY_WIDTH,
            height=DISPLAY_HEIGHT,
            chn=CAM_CHN_ID_0
        )

        sensor.set_pixformat(
            PIXEL_FORMAT_YUV_SEMIPLANAR_420,
            chn=CAM_CHN_ID_0
        )

        # 通道2：AI RGB888 planar 输入
        sensor.set_framesize(
            width=AI_FRAME_WIDTH,
            height=AI_FRAME_HEIGHT,
            chn=CAM_CHN_ID_2
        )

        sensor.set_pixformat(
            PIXEL_FORMAT_RGB_888_PLANAR,
            chn=CAM_CHN_ID_2
        )

        # 通道0绑定到视频层
        sensor_bind_info = sensor.bind_info(
            x=0,
            y=0,
            chn=CAM_CHN_ID_0
        )

        Display.bind_layer(
            **sensor_bind_info,
            layer=Display.LAYER_VIDEO1
        )

        # ----------------------------------------------------
        # 初始化显示
        # ----------------------------------------------------

        if display_mode == "lcd":
            Display.init(
                Display.ST7701,
                width=DISPLAY_WIDTH,
                height=DISPLAY_HEIGHT,
                osd_num=1,
                to_ide=True
            )
        else:
            Display.init(
                Display.LT9611,
                width=DISPLAY_WIDTH,
                height=DISPLAY_HEIGHT,
                osd_num=1,
                to_ide=True
            )

        # OSD尺寸必须与显示分辨率一致
        osd_img = image.Image(
            DISPLAY_WIDTH,
            DISPLAY_HEIGHT,
            image.ARGB8888
        )

        # ----------------------------------------------------
        # 启动媒体系统
        # ----------------------------------------------------

        MediaManager.init()
        sensor.run()

        fps = 0.0
        last_ticks = time.ticks_ms()

        # AI坐标到LCD坐标的缩放比例
        scale_x = float(DISPLAY_WIDTH) / float(AI_FRAME_WIDTH)
        scale_y = float(DISPLAY_HEIGHT) / float(AI_FRAME_HEIGHT)

        # ----------------------------------------------------
        # 主循环
        # ----------------------------------------------------

        while True:
            os.exitpoint()

            with ScopedTiming("total", debug_mode > 0):
                rgb888p_img = sensor.snapshot(
                    chn=CAM_CHN_ID_2
                )

                if rgb888p_img is None:
                    continue

                if rgb888p_img.format() != image.RGBP888:
                    print(
                        "Unexpected image format:",
                        rgb888p_img.format()
                    )
                    rgb888p_img = None
                    continue

                # RGBP888图像转为共享内存的NumPy数组
                ai2d_input = rgb888p_img.to_numpy_ref()

                # 正常情况下形状为：
                # (3, AI_FRAME_HEIGHT, AI_FRAME_WIDTH)
                ai2d_input_tensor = nn.from_numpy(ai2d_input)

                # AI预处理
                ai2d_builder.run(
                    ai2d_input_tensor,
                    ai2d_output_tensor
                )

                # KPU推理
                kpu.set_input_tensor(
                    0,
                    ai2d_output_tensor
                )

                kpu.run()

                # 获取模型输出
                results = []

                for i in range(kpu.outputs_size()):
                    out_tensor = kpu.get_output_tensor(i)
                    result = out_tensor.to_numpy()

                    # 将输出展平
                    result = result.reshape(
                        (
                            result.shape[0]
                            * result.shape[1]
                            * result.shape[2]
                            * result.shape[3]
                        )
                    )

                    results.append(result)
                    del out_tensor

                # ------------------------------------------------
                # 后处理
                # ------------------------------------------------

                if model_type == "AnchorBaseDet":
                    det_boxes = aicube.anchorbasedet_post_process(
                        results[0],
                        results[1],
                        results[2],
                        kmodel_frame_size,
                        frame_size,
                        strides,
                        num_classes,
                        confidence_threshold,
                        nms_threshold,
                        anchors,
                        nms_option
                    )

                elif model_type == "GFLDet":
                    det_boxes = aicube.gfldet_post_process(
                        results[0],
                        results[1],
                        results[2],
                        kmodel_frame_size,
                        frame_size,
                        strides,
                        num_classes,
                        confidence_threshold,
                        nms_threshold,
                        nms_option
                    )

                else:
                    det_boxes = aicube.anchorfreedet_post_process(
                        results[0],
                        results[1],
                        results[2],
                        kmodel_frame_size,
                        frame_size,
                        strides,
                        num_classes,
                        confidence_threshold,
                        nms_threshold,
                        nms_option
                    )

                det_boxes = merge_overlap_boxes(det_boxes)

                # ------------------------------------------------
                # 绘制OSD
                # ------------------------------------------------

                osd_img.clear()

                # LCD中心点
                center_x = DISPLAY_WIDTH // 2
                center_y = DISPLAY_HEIGHT // 2

                osd_img.draw_circle(
                    center_x,
                    center_y,
                    2,
                    color=(0, 255, 0),
                    fill=True
                )

                max_area = 0
                ax = -1
                ay = -1

                if det_boxes:
                    for det_box in det_boxes:
                        class_id = int(det_box[0])
                        confidence = float(det_box[1])

                        # 限制AI坐标，避免异常检测框导致绘图越界
                        x1 = clamp(
                            float(det_box[2]),
                            0,
                            AI_FRAME_WIDTH - 1
                        )

                        y1 = clamp(
                            float(det_box[3]),
                            0,
                            AI_FRAME_HEIGHT - 1
                        )

                        x2 = clamp(
                            float(det_box[4]),
                            0,
                            AI_FRAME_WIDTH - 1
                        )

                        y2 = clamp(
                            float(det_box[5]),
                            0,
                            AI_FRAME_HEIGHT - 1
                        )

                        if x2 <= x1 or y2 <= y1:
                            continue

                        # 映射到LCD坐标
                        lcd_x1 = int(x1 * scale_x)
                        lcd_y1 = int(y1 * scale_y)
                        lcd_x2 = int(x2 * scale_x)
                        lcd_y2 = int(y2 * scale_y)

                        lcd_x1 = clamp(
                            lcd_x1,
                            0,
                            DISPLAY_WIDTH - 1
                        )

                        lcd_y1 = clamp(
                            lcd_y1,
                            0,
                            DISPLAY_HEIGHT - 1
                        )

                        lcd_x2 = clamp(
                            lcd_x2,
                            0,
                            DISPLAY_WIDTH - 1
                        )

                        lcd_y2 = clamp(
                            lcd_y2,
                            0,
                            DISPLAY_HEIGHT - 1
                        )

                        rect_w = lcd_x2 - lcd_x1
                        rect_h = lcd_y2 - lcd_y1

                        if rect_w <= 0 or rect_h <= 0:
                            continue

                        # 防止类别索引越界
                        if class_id >= 0 and class_id < len(color_four):
                            draw_color = color_four[class_id][1:]
                        else:
                            draw_color = (255, 0, 0)

                        if class_id >= 0 and class_id < len(labels):
                            label = labels[class_id]
                        else:
                            label = "class_" + str(class_id)

                        score = str(round(confidence, 2))

                        osd_img.draw_rectangle(
                            lcd_x1,
                            lcd_y1,
                            rect_w,
                            rect_h,
                            color=draw_color,
                            thickness=2
                        )

                        text_y = lcd_y1 - 32

                        if text_y < 0:
                            text_y = lcd_y1

                        osd_img.draw_string_advanced(
                            lcd_x1,
                            text_y,
                            24,
                            label + " " + score,
                            color=draw_color
                        )

                        # 使用AI坐标面积寻找最大检测框
                        area = (x2 - x1) * (y2 - y1)

                        if area > max_area:
                            max_area = area
                            ax = (x1 + x2) / 2.0
                            ay = (y1 + y2) / 2.0

                # ------------------------------------------------
                # 绘制最大目标中心连线
                # ------------------------------------------------

                send_ball_center(uart, ax, ay)

                if ax >= 0 and ay >= 0:
                    lcd_ax = int(ax * scale_x)
                    lcd_ay = int(ay * scale_y)

                    lcd_ax = clamp(
                        lcd_ax,
                        0,
                        DISPLAY_WIDTH - 1
                    )

                    lcd_ay = clamp(
                        lcd_ay,
                        0,
                        DISPLAY_HEIGHT - 1
                    )

                    osd_img.draw_string_advanced(
                        2,
                        30,
                        24,
                        "ax=" + str(int(ax)),
                        color=(0, 255, 0)
                    )

                    # 水平线：
                    # (目标中心X, 屏幕中心Y) -> 屏幕中心
                    osd_img.draw_line(
                        lcd_ax,
                        center_y,
                        center_x,
                        center_y,
                        color=(0, 255, 0),
                        thickness=1
                    )

                    # 竖直线：
                    # 目标中心 -> (目标中心X, 屏幕中心Y)
                    osd_img.draw_line(
                        lcd_ax,
                        lcd_ay,
                        lcd_ax,
                        center_y,
                        color=(0, 255, 0),
                        thickness=1
                    )

                # ------------------------------------------------
                # FPS
                # ------------------------------------------------

                now = time.ticks_ms()
                dt_ms = time.ticks_diff(now, last_ticks)
                last_ticks = now

                if dt_ms > 0:
                    current_fps = 1000.0 / float(dt_ms)
                    fps = 0.9 * fps + 0.1 * current_fps

                osd_img.draw_string_advanced(
                    2,
                    2,
                    24,
                    "FPS=" + str(int(fps)),
                    color=(0, 255, 255)
                )

                # 使用OSD0，因为初始化时只申请了一个OSD层
                Display.show_image(
                    osd_img,
                    0,
                    0,
                    Display.LAYER_OSD0
                )

                # ------------------------------------------------
                # 释放当前帧引用
                # ------------------------------------------------

                ai2d_input_tensor = None
                rgb888p_img = None

                del results
                gc.collect()

    except KeyboardInterrupt:
        print("User stopped")

    except Exception as e:
        print("Detection exception:")
        sys.print_exception(e)

    finally:
        print("Releasing resources...")

        ai2d_input_tensor = None
        ai2d_output_tensor = None
        rgb888p_img = None
        osd_img = None

        if uart is not None:
            try:
                uart.deinit()
            except Exception as e:
                print("uart.deinit error:")
                sys.print_exception(e)

        if sensor is not None:
            try:
                sensor.stop()
            except Exception as e:
                print("sensor.stop error:")
                sys.print_exception(e)

        try:
            Display.deinit()
        except Exception as e:
            print("Display.deinit error:")
            sys.print_exception(e)

        try:
            os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        except Exception:
            pass

        time.sleep_ms(100)

        try:
            MediaManager.deinit()
        except Exception as e:
            print("MediaManager.deinit error:")
            sys.print_exception(e)

        ai2d_builder = None
        ai2d = None
        kpu = None

        gc.collect()
        time.sleep(1)

        try:
            nn.shrink_memory_pool()
        except Exception:
            pass

    print("det_infer end")
    return 0


# ============================================================
# 程序入口
# ============================================================

if __name__ == "__main__":
    os.exitpoint(os.EXITPOINT_ENABLE)
    detection()
