import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
import sounddevice as sd
from scipy.io.wavfile import write
import speech_recognition as sr
import requests
import base64
import json
import os
from pynput import keyboard

# 全局变量
dist_tavolo = None
distance_printed = False
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

def get_dist_tavolo(depth):
    global dist_tavolo, distance_printed
    dist_tavolo = np.nanmax(depth)
    if not distance_printed:
        print(f"桌子到相机的距离: {dist_tavolo}")
        distance_printed = True

def depth_callback(image_depth):
    bridge = CvBridge()
    try:
        depth = bridge.imgmsg_to_cv2(image_depth, "32FC1")
        get_dist_tavolo(depth)
    except Exception as e:
        print(f"转换图像时出错: {e}")

# 配置参数
SAMPLE_RATE = 44100
CHUNK_SIZE = 1024
TEMP_AUDIO_FILE = "/home/zzq/voice_qwen/outputs/temp_audio.wav"
IMAGE_SAVE_DIR = "/home/zzq/voice_qwen/outputs/VLMpictures"

API_KEY = 
BASE_URL = 

pickup_coords_pub = rospy.Publisher('/pickup_coords', Point, queue_size=10)

cam_point = (-0.44, -0.5, 1.58)
height_tavolo = 0.74
DEFAULT_Z = 0.777777

class GazeboImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        self.image = None
        self.cv_image = None

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image = self.cv_image.copy()
        except Exception as e:
            rospy.logerr(f"转换图像失败: {e}")

    def capture_image(self):
        if self.image is None:
            print("无法捕获图像：未接收到图像数据")
            return None

        os.makedirs(IMAGE_SAVE_DIR, exist_ok=True)
        timestamp = int(cv.getTickCount())
        image_file = os.path.join(IMAGE_SAVE_DIR, f"captured_image_{timestamp}.jpg")
        cv.imwrite(image_file, self.image)
        print(f"图像已保存为：{image_file}")
        return image_file

def record_audio():
    print("开始录音... 按 'q' 键停止")
    frames = []
    recording = True

    def on_press(key):
        nonlocal recording
        if key.char == 'q':
            recording = False

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    try:
        with sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=1,
            dtype=np.int16,
            blocksize=CHUNK_SIZE
        ) as stream:
            while recording:
                data, _ = stream.read(CHUNK_SIZE)
                frames.append(data)
    except KeyboardInterrupt:
        recording = False

    listener.stop()

    if frames:
        data = np.concatenate(frames, axis=0)
        os.makedirs(os.path.dirname(TEMP_AUDIO_FILE), exist_ok=True)
        write(TEMP_AUDIO_FILE, SAMPLE_RATE, data)
        print(f"录音保存为：{TEMP_AUDIO_FILE}")
        return TEMP_AUDIO_FILE
    else:
        print("未录制到音频数据")
        return None

def recognize_speech(audio_file):
    recognizer = sr.Recognizer()
    with sr.AudioFile(audio_file) as source:
        audio_data = recognizer.record(source)
    try:
        text = recognizer.recognize_google(audio_data, language="zh-CN")
        print("语音识别结果：", text)
        return text
    except sr.UnknownValueError:
        print("语音识别失败：无法识别音频内容")
        return None
    except sr.RequestError as e:
        print(f"语音识别失败：{e}")
        return None

def encode_image_to_base64(image_path):
    with open(image_path, "rb") as image_file:
        encoded_string = base64.b64encode(image_file.read()).decode("utf-8")
    return encoded_string

def query_vlm(image_base64, question):
    headers = {
        "Authorization": f"Bearer {API_KEY}",
        "Content-Type": "application/json"
    }
    payload = {
        "model": "qwen-vl-max-latest",
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": f"请分析图像，直接返回{question}在640x480图像中的中心点坐标，格式为(x, y)。注意坐标系原点在左上角，x轴向右，y轴向下。。不要包含任何额外的分析或解释只有坐标。"},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_base64}"}}
                ]
            }
        ],
        "max_tokens": 100
    }

    try:
        response = requests.post(BASE_URL, headers=headers, data=json.dumps(payload))
        response.raise_for_status()
        result = response.json()
        return result.get("choices", [{}])[0].get("message", {}).get("content", "未能获取有效回答")
    except requests.exceptions.RequestException as e:
        print(f"调用 DashScope API 失败：{e}")
        return None

def extract_coordinates(answer):
    import re
    pattern = r'\((\d+), (\d+)\)'
    match = re.search(pattern, answer)
    if match:
        x = float(match.group(1))
        y = float(match.group(2))
        return x, y
    else:
        print("未能从回答中提取坐标信息")
        return None, None

def convert_pixel_to_arm_coords(x_pixel, y_pixel):
    global dist_tavolo
    if dist_tavolo is None:
        print("桌子到相机的距离未计算，请等待深度图像处理完成。")
        return None, None, None

    xyz = np.array((x_pixel, y_pixel, 0))
    xyz[:2] /= IMAGE_WIDTH, IMAGE_HEIGHT
    xyz[:2] -= 0.5
    xyz[:2] *= (-0.968, 0.691)
    xyz[:2] *= dist_tavolo / 0.84
    xyz[:2] += cam_point[:2]
    xyz[2] = DEFAULT_Z

    return xyz[0], xyz[1], xyz[2]

def publish_pickup_coords(x, y, z):
    point = Point()
    point.x = x
    point.y = y
    point.z = z
    pickup_coords_pub.publish(point)
    print(f"已发布 /pickup_coords 话题：x={x}, y={y}, z={z}")

def main():
    rospy.init_node('gazebo_image_subscriber')
    subscriber = GazeboImageSubscriber()
    rospy.Subscriber('/camera/color/image_raw', Image, subscriber.image_callback)
    rospy.Subscriber('/camera/depth/image_raw', Image, depth_callback)

    print("程序启动，准备接收 Gazebo 相机图像...")

    while not rospy.is_shutdown():
        print("\n请等待图像窗口弹出，并按下 'q' 键确认画面显示...")
        confirmation_received = False
        while not rospy.is_shutdown() and not confirmation_received:
            if subscriber.cv_image is not None:
                cv.imshow("Gazebo Camera", subscriber.cv_image)
                key = cv.waitKey(1) & 0xFF
                if key == ord('q'):
                    confirmation_received = True
                    print("已确认画面显示，继续执行后续步骤...")
            else:
                rospy.sleep(0.1)

        if not confirmation_received:
            print("未确认画面显示，程序退出。")
            break

        # 捕获当前图像
        image_file = subscriber.capture_image()
        if not image_file:
            print("捕获图像失败，跳过本次循环")
            continue  # 添加明确的continue

        # 录音与语音识别
        input("按下回车键开始语音输入，按 'q' 键结束录音...")
        audio_file = record_audio()
        if not audio_file:
            continue

        question = recognize_speech(audio_file)
        if not question:
            continue

        print(f"\n你的问题是：{question}")

        # 调用 VLM 模型
        try:
            image_base64 = encode_image_to_base64(image_file)
        except Exception as e:
            print(f"编码图像失败：{e}")
            continue  # 捕获异常并跳过

        answer = query_vlm(image_base64, question)
        if answer:
            print(f"\nDashScope 回答：{answer}\n{'=' * 40}")
            x_pixel, y_pixel = extract_coordinates(answer)
            if x_pixel is not None and y_pixel is not None:
                x_arm, y_arm, z_arm = convert_pixel_to_arm_coords(x_pixel, y_pixel)
                if x_arm is not None and y_arm is not None and z_arm is not None:
                    publish_pickup_coords(x_arm, y_arm, z_arm)
        else:
            print("DashScope 未能生成有效回答")

    cv.destroyAllWindows()
    rospy.signal_shutdown("任务完成")

if __name__ == "__main__":
    main()
    
