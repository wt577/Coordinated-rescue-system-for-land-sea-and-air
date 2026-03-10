import argparse
import time
from pathlib import Path
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from numpy import random
import threading
from queue import Queue
import rospy
from std_msgs.msg import String

# 添加量化支持
from torch.quantization import quantize_dynamic

from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized


MODEL_NAME = "beset"  # 融合模型
model_alert_status = {"fall": False, "drown": False}  # 警报状态

def load_model(weights, device, half, quantize=False):
    if not Path(weights).exists():
        print(f"模型文件不存在: {weights}")
        return None
    try:
        model = attempt_load(weights, map_location=device, weights_only=True)
    except:
        model = attempt_load(weights, map_location=device)
    # 应用动态量化 (INT8)
    if quantize and device.type != 'cpu':
        model = quantize_dynamic(model, {torch.nn.Linear}, dtype=torch.qint8)
        print(f"应用INT8量化到 {MODEL_NAME}")
    model_img_size = getattr(model, 'img_size', 640)
    stride = int(model.stride.max())
    if half:
        model.half()
    model_info = {
        "model": model,
        "name": MODEL_NAME,
        "names": model.module.names if hasattr(model, 'module') else model.names,
        "colors": [[random.randint(0, 255) for _ in range(3)] for _ in (model.module.names if hasattr(model, 'module') else model.names)],
        "img_size": model_img_size,
        "stride": stride
    }
    print(f"加载模型: {MODEL_NAME} | 步长: {stride} | 尺寸: {model_img_size}")
    return model_info

# ---------------------- 图像预处理函数 ----------------------
def preprocess_image(img, img_size, device, half, stride):
    img_resized = letterbox(img, img_size, stride=stride)[0]
    img_resized = img_resized[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, HWC to CHW
    img_resized = np.ascontiguousarray(img_resized)
    img_tensor = torch.from_numpy(img_resized).to(device)
    img_tensor = img_tensor.half() if half else img_tensor.float()
    img_tensor /= 255.0
    if img_tensor.ndimension() == 3:
        img_tensor = img_tensor.unsqueeze(0)
    return img_tensor

# ---------------------- 辅助函数: 图像填充 ----------------------
def letterbox(img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
    shape = img.shape[:2]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:
        r = min(r, 1.0)
    ratio = r, r
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    if auto:
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)
    elif scaleFill:
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]
    dw /= 2
    dh /= 2
    if shape[::-1] != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return img, ratio, (dw, dh)

# ---------------------- 并行推理处理类 ----------------------
class ModelProcessor(threading.Thread):
    def __init__(self, model_info, input_queue, output_queue, img_size, device, half, opt):
        super().__init__()
        self.model_info = model_info
        self.input_queue = input_queue
        self.output_queue = output_queue
        self.img_size = img_size
        self.device = device
        self.half = half
        self.opt = opt
        self.daemon = True
    def run(self):
        while True:
            task = self.input_queue.get()
            if task is None:
                break
            img_tensor, im0, frame_id = task
            try:
                with torch.no_grad():
                    pred = self.model_info["model"](img_tensor, augment=self.opt.augment)[0]
                pred = non_max_suppression(pred, self.opt.conf_thres, self.opt.iou_thres, 
                                          classes=self.opt.classes, agnostic=self.opt.agnostic_nms)
                detections = []
                for i, det in enumerate(pred):
                    if len(det):
                        det[:, :4] = scale_coords(img_tensor.shape[2:], det[:, :4], im0.shape).round()
                        detections.append(det.cpu().numpy())
                self.output_queue.put((self.model_info, detections, im0, frame_id))
            except Exception as e:
                print(f"{self.model_info['name']} 处理失败: {e}")
            finally:
                self.input_queue.task_done()

# ---------------------- 运动检测类 ----------------------
class MotionDetector:
    def __init__(self, min_area=500):
        self.min_area = min_area
        self.prev_gray = None
        self.fgbg = cv2.createBackgroundSubtractorMOG2(history=100, varThreshold=16, detectShadows=False)
    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
        if self.prev_gray is None:
            self.prev_gray = gray
            return False, None
        frame_diff = cv2.absdiff(self.prev_gray, gray)
        _, thresh = cv2.threshold(frame_diff, 25, 255, cv2.THRESH_BINARY)
        thresh = cv2.dilate(thresh, None, iterations=2)
        contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        motion_detected = False
        motion_roi = None
        for contour in contours:
            if cv2.contourArea(contour) < self.min_area:
                continue
            (x, y, w, h) = cv2.boundingRect(contour)
            motion_detected = True
            if motion_roi is None:
                motion_roi = (x, y, x+w, y+h)
            else:
                x1, y1, x2, y2 = motion_roi
                motion_roi = (min(x1, x), min(y1, y), max(x2, x+w), max(y2, y+h))
        self.prev_gray = gray
        return motion_detected, motion_roi

# ---------------------- 主检测函数----------------------
def detect_single_model_optimized(save_img=False):
    global model_alert_status
    # 添加缺失参数的默认值
    opt.augment = getattr(opt, 'augment', False)
    opt.agnostic_nms = getattr(opt, 'agnostic_nms', False)
    opt.classes = getattr(opt, 'classes', None)
    opt.project = getattr(opt, 'project', 'runs/detect')
    opt.name = getattr(opt, 'name', 'exp')
    opt.exist_ok = getattr(opt, 'exist_ok', False)
    source = opt.source
    weights = opt.weights
    view_img = opt.view_img
    save_txt = opt.save_txt
    base_img_size = opt.img_size
    save_img = not opt.nosave and not source.endswith('.txt')
    webcam = source.isnumeric() or source.endswith('.txt') or source.lower().startswith(
        ('rtsp://', 'rtmp://', 'http://', 'https://'))
    # 目录设置
    save_dir = Path(increment_path(Path(opt.project) / opt.name, exist_ok=opt.exist_ok))
    (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)
    # 初始化
    set_logging()
    device = select_device(opt.device)
    half = device.type != 'cpu'
    # 加载模型
    try:
        model_info = load_model(weights, device, half, quantize=opt.quantize)
        if model_info is None:
            print("模型加载失败")
            return
        img_size = model_info["img_size"]
        stride = model_info["stride"]
        print(f"输入尺寸: {img_size}px | 步长: {stride}")
    except Exception as e:
        print(f"模型加载失败: {e}")
        return
    # 检查图像尺寸
    base_img_size = check_img_size(base_img_size, s=stride)
    # 数据加载器
    vid_path, vid_writer = None, None
    if webcam:
        view_img = check_imshow()
        cudnn.benchmark = True
        dataset = LoadStreams(source, img_size=img_size, stride=stride)
    else:
        dataset = LoadImages(source, img_size=img_size, stride=stride)
    # 创建处理队列
    input_queue = Queue(maxsize=5)
    output_queue = Queue(maxsize=5)
    # 启动模型处理线程
    processor = ModelProcessor(
        model_info, input_queue, output_queue, 
        img_size, device, half, opt
    )
    processor.start()
    # 模型warmup
    dummy = torch.zeros(1, 3, img_size, img_size).to(device)
    if half:
        dummy = dummy.half()
    with torch.no_grad():
        model_info["model"](dummy)
    # 运动检测器
    motion_detector = MotionDetector(min_area=2000)
    # 性能计数器
    t0 = time.time()
    frame_count = 0
    processed_count = 0
    fps = 0
    last_time = time.time()
    # 动态处理间隔
    base_process_interval = 8
    current_interval = base_process_interval
    motion_frames = 0
    # 显示线程相关
    from threading import Thread
    import queue as thread_queue
    display_queue = thread_queue.Queue(maxsize=5)
    def display_thread_func():
        while True:
            try:
                display_im0 = display_queue.get(timeout=2)
            except:
                continue
            if display_im0 is None:
                break
            cv2.imshow('Beset Detection', display_im0)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    display_thread = None
    if view_img:
        display_thread = Thread(target=display_thread_func, daemon=True)
        display_thread.start()
    for path, img, im0s, vid_cap in dataset:
        frame_count += 1
        current_im0 = im0s[0].copy() if webcam else im0s.copy()
        # 计算FPS
        current_time = time.time()
        if current_time - last_time >= 1.0:
            fps = processed_count
            processed_count = 0
            last_time = current_time
            if frame_count % (current_interval*2) == 0:
                print(f"FPS: {fps} | 间隔: {current_interval}帧")
        # 运动检测
        motion_detected, motion_roi = motion_detector.detect(current_im0)
        # 动态调整处理频率
        if motion_detected:
            motion_frames += 1
            if motion_frames > 10:
                current_interval = max(1, base_process_interval - 3)
        else:
            motion_frames = max(0, motion_frames - 1)
            if motion_frames < 5:
                current_interval = base_process_interval
        # 按间隔处理帧
        if frame_count % current_interval != 0:
            if view_img:
                display_im0 = current_im0.copy()
                cv2.putText(display_im0, f"FPS: {fps}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(display_im0, f"Interval: 1/{current_interval}", (10, 70), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                status = f"Motion: {'YES' if motion_detected else 'NO'}"
                cv2.putText(display_im0, status, (10, 110), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, 
                            (0, 255, 0) if motion_detected else (0, 0, 255), 2)
                if motion_roi:
                    x1, y1, x2, y2 = motion_roi
                    cv2.rectangle(display_im0, (x1, y1), (x2, y2), (0, 255, 255), 2)
                try:
                    display_queue.put_nowait(display_im0)
                except:
                    pass
            continue
        processed_count += 1
        # 只处理运动区域 (ROI)
        roi_im0 = current_im0
        if motion_roi:
            x1, y1, x2, y2 = motion_roi
            expand = 50
            h, w = current_im0.shape[:2]
            x1 = max(0, x1 - expand)
            y1 = max(0, y1 - expand)
            x2 = min(w, x2 + expand)
            y2 = min(h, y2 + expand)
            roi_im0 = current_im0[y1:y2, x1:x2]
        img_tensor = preprocess_image(roi_im0, img_size, device, half, stride)
        input_queue.put((img_tensor, roi_im0, frame_count))
        try:
            model_info, detections, roi_im, fid = output_queue.get(timeout=2)
        except:
            continue  # 超时则跳过，防止主线程卡死
        if fid != frame_count:
            continue
        # 处理并绘制结果
        detected_fall = False
        detected_drown = False
        for det in detections:
            for *xyxy, conf, cls in det:
                class_name = model_info["names"][int(cls)]
                # 调整ROI坐标到原始图像
                if motion_roi:
                    x1, y1, _, _ = motion_roi
                    xyxy = [xyxy[0] + x1, xyxy[1] + y1, xyxy[2] + x1, xyxy[3] + y1]
                label = f'{model_info["name"]}: {class_name} {conf:.2f}'
                plot_one_box(xyxy, current_im0, label=label, 
                            color=model_info["colors"][int(cls)], line_thickness=2)
                # 发布警报
                if class_name == "fall":
                    detected_fall = True
                if class_name == "drown" or class_name == "drowning":
                    detected_drown = True
        if detected_fall:
            rospy.loginfo("fall")
            pub_fall.publish("fall")
        if detected_drown:
            rospy.loginfo("drown")
            pub_drown.publish("drown")
        # 显示结果
        if view_img:
            cv2.putText(current_im0, f"FPS: {fps}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(current_im0, f"Interval: 1/{current_interval}", (10, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            status = f"Motion: {'YES' if motion_detected else 'NO'}"
            cv2.putText(current_im0, status, (10, 110), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, 
                       (0, 255, 0) if motion_detected else (0, 0, 255), 2)
            try:
                display_queue.put_nowait(current_im0)
            except:
                pass
        # 保存结果
        if save_img:
            try:
                if webcam:
                    timestamp = time.strftime("%Y%m%d_%H%M%S")
                    save_path = str(save_dir / f"output_{timestamp}.mp4")
                else:
                    save_path = str(save_dir / Path(path).name)
                if dataset.mode == 'image':
                    cv2.imwrite(save_path, current_im0)
                else:
                    if vid_path != save_path:
                        vid_path = save_path
                        if vid_writer:
                            vid_writer.release()
                        fpsv = vid_cap.get(cv2.CAP_PROP_FPS) if vid_cap else 30
                        w, h = current_im0.shape[1], current_im0.shape[0]
                        vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fpsv, (w, h))
                    vid_writer.write(current_im0)
            except Exception as e:
                print(f"⚠️ 保存失败: {e}")
    # 清理线程
    input_queue.put(None)
    processor.join()
    if view_img:
        display_queue.put(None)
        if display_thread:
            display_thread.join()
    if vid_writer:
        vid_writer.release()
    print(f'\n检测完成 | 总耗时: ({time.time() - t0:.3f}s) | 保存路径: {save_dir}')

# ---------------------- 主程序入口 (增加量化选项) ----------------------
if __name__ == '__main__':
    current_dir = Path(__file__).parent
    models_dir = current_dir / 'models'
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, 
                        default=str(models_dir / 'beset.pt'),
                        help='模型路径')
    parser.add_argument('--quantize', action='store_true', help='使用INT8量化模型加速')
    parser.add_argument('--source', type=str, default='0', help='输入源（摄像头/视频路径/图片路径）')
    parser.add_argument('--img-size', type=int, default=416, help='基础推理尺寸（像素）')
    parser.add_argument('--conf-thres', type=float, default=0.4, help='置信度阈值')
    parser.add_argument('--iou-thres', type=float, default=0.5, help='NMS IoU阈值')
    parser.add_argument('--device', default='', help='计算设备（如0或0,1或cpu）')
    parser.add_argument('--view-img', action='store_true', help='显示实时画面')
    parser.add_argument('--nosave', action='store_true', help='禁止保存图像/视频')
    parser.add_argument('--save-txt', action='store_true', help='保存检测结果到txt文件')
    parser.add_argument('--save-conf', action='store_true', help='在txt中保存置信度')
    parser.add_argument('--augment', action='store_true', help='增强推理（较慢）')
    opt = parser.parse_args()
    print(f'\n配置参数: {opt}\n')
    try:
        check_requirements(exclude=('pycocotools', 'thop'))
    except Exception as e:
        print(f"依赖检查失败: {e}")
        print("继续运行，假设所有依赖已安装...")
    # ROS初始化
    rospy.init_node('beset_detector', anonymous=True, log_level=rospy.INFO)
    pub_fall = rospy.Publisher('fall_alert', String, queue_size=10)
    pub_drown = rospy.Publisher('drowning_alert', String, queue_size=10)
    with torch.no_grad():
        detect_single_model_optimized() 